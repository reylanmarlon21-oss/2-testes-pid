// rev.1 2-testes-pid: Uno escravo I2C 0x08 + PID real do 1-teste-pid
// rev.2: reenvio nao zera rampa + 4o byte PWM + sanidade 0-30 PSI
// rev.3: trava em SP+0.2 (passa do ponto e para) + reenvio nao rearma ponto feito
// rev.4: NUCLEO PID (Kp,Ki,Kd) intocado + camada de atuador com teto proporcional
//        G*erro -> a forca DESCE suave ate os 77 (30%) e so entao corta (curva 2)
// rev.5: revisao atual de bancada, sincronizada com o ESP32 rev.ESP32-5
// Mestre (ESP32) envia: [cmd, sp_lo, sp_hi] setpoint em centi-PSI
//   cmd 1 = ir para setpoint | cmd 2 = parar | cmd 3 = ventilar
// Escravo responde 4 bytes: [pv_lo, pv_hi, status, pwm]
//   status 0=subindo 1=no ponto 2=parado 3=ventilando
// Protecoes: PWM 0 ou 77-242, freio a 0.5, histerese 1.0, overshoot ventila,
// rampa +1/5ciclos, mira +1.0 PSI, solenoide abre junto com motor.

#include <Arduino.h>
#include <Wire.h>

#define SLAVE_ADDR      0x08
#define PINO_PWM        9
#define PINO_SOLENOIDE  8
#define PINO_PRESSAO    A0

#define PRESSAO_MIN_V   0.5
#define PRESSAO_MAX_V   4.5
#define PRESSAO_MAX_PSI 30.0
#define PWM_MAXIMO      242
#define PWM_MINIMO      77
#define MARGEM_MIRA     1.0
#define GANHO_APROX     48.0   // teto de aproximacao: teto = GANHO_APROX * erro (so limita a chegada)

float Kp = 25.0, Ki = 4.0, Kd = 6.0;
float integral = 0, ultimoErro = 0, derivFilt = 0;

float setpoint = 0;          // PSI
uint8_t statusNut = 2;       // comeca parado
volatile uint16_t pvCache = 0; // pressao em centi-PSI p/ resposta I2C
volatile uint8_t stCache = 2;  // status p/ resposta I2C
bool temAlvo = false;
bool freioProx = false;
int pwmAnt = 0, divRampa = 0, pwmAtual = 0;
int uDebug = 0; float tetoDebug = 0; // so p/ log (afinacao do teto proporcional)
unsigned long tFaixa = 0;

float lerPressaoPSI() {
  long s = 0;
  for (int i = 0; i < 16; i++) { s += analogRead(PINO_PRESSAO); delay(2); }
  float v = (s / 16) * 5.0 / 1023.0;
  float psi = (v - PRESSAO_MIN_V) * PRESSAO_MAX_PSI / (PRESSAO_MAX_V - PRESSAO_MIN_V);
  return constrain(psi, 0, PRESSAO_MAX_PSI);
}

// ================== NUCLEO PID ==================
// u(t) = Kp*e(t) + Ki*Integral(e dt) + Kd*de/dt      <<< FORMULA INTOCAVEL
// Retorna a saida BRUTA do controlador. Piso/corte de motor NAO moram aqui:
// isso e responsabilidade do acondicionamento (acondicionarPWM), logo abaixo.
int calcularPID(float sp, float pv, float dt) {
  if (dt <= 0) dt = 0.05;
  float erro = sp - pv;
  if (pv > sp * 1.05) { digitalWrite(PINO_SOLENOIDE, HIGH); return 0; } // overshoot: ventila
  float dBruto = (erro - ultimoErro) / dt;
  derivFilt = 0.6 * dBruto + 0.4 * derivFilt;        // derivada filtrada (anti-ruido)
  ultimoErro = erro;
  if (fabs(erro) > 0.3) integral += erro * dt;       // so integra fora de 0.3 (anti-windup)
  integral = constrain(integral, -200.0 / Ki, 200.0 / Ki);
  if (fabs(erro) <= 0.01 * sp) integral = 0;
  float u = Kp * erro + Ki * integral + Kd * derivFilt;   // <<< PID
  if (!isfinite(u)) u = 0;
  if (u < 0) u = 0;
  if (u > PWM_MAXIMO) u = PWM_MAXIMO;
  return (int)u;
}

// ============ ACONDICIONAMENTO DO ATUADOR ============
// Camada SEPARADA do PID. So traduz a saida u para o mundo real do motor:
//  - teto proporcional: teto = GANHO_APROX*erro. Quanto mais perto do alvo,
//    menor o teto -> pwm = min(u, teto) faz a forca DESCER suave (curva 2).
//  - piso fisico: o motor nao gira entre 1..76, entao mantem PWM_MINIMO.
//  - o degrau final 77 -> 0 e inevitavel (motor nao faz menos que 30%).
int acondicionarPWM(int u, float erro) {
  if (u <= 0) return 0;
  float teto = GANHO_APROX * erro;
  if (teto < u) u = (int)teto;          // min(u, teto): PID decide, teto limita
  if (u > PWM_MAXIMO) u = PWM_MAXIMO;
  if (u < PWM_MINIMO) u = PWM_MINIMO;   // nunca 1..76
  return u;
}

void irPara(float sp) {
  if (sp <= 0 || sp > 30) return; // sanidade: ignora pacote espurio
  if (fabs(sp - setpoint) < 0.01 && (temAlvo || statusNut == 1)) return; // reenvio: mantem estado, nao rearma
  setpoint = sp; integral = 0; ultimoErro = 0; derivFilt = 0;
  freioProx = false; pwmAnt = 0; divRampa = 0;
  tFaixa = 0; temAlvo = true; statusNut = 0;
  digitalWrite(PINO_SOLENOIDE, HIGH);
  Serial.print(F(">>> Alvo ")); Serial.println(sp);
}

void parar(bool ventilar) {
  analogWrite(PINO_PWM, 0);
  digitalWrite(PINO_PWM, LOW);
  digitalWrite(PINO_SOLENOIDE, ventilar ? HIGH : LOW);
  pwmAtual = 0; temAlvo = false;
  statusNut = ventilar ? 3 : 2;
  Serial.println(ventilar ? F("Ventilando") : F("Parado"));
}

void aoReceber(int n) {
  if (n < 3) { while (Wire.available()) Wire.read(); return; }
  uint8_t cmd = Wire.read();
  uint16_t sp = Wire.read() | (Wire.read() << 8);
  while (Wire.available()) Wire.read();
  if (cmd == 1) irPara(sp / 100.0);
  else if (cmd == 2) parar(false);
  else if (cmd == 3) parar(true);
}

void aoPedir() {
  // rapido: so envia cache (leitura do ADC fica no loop, nunca na ISR)
  Wire.write(pvCache & 0xFF);
  Wire.write((pvCache >> 8) & 0xFF);
  Wire.write(stCache);
  Wire.write((uint8_t)pwmAtual); // 4o byte: PWM injetado p/ a tela
}

void setup() {
  pinMode(PINO_PWM, OUTPUT);
  pinMode(PINO_SOLENOIDE, OUTPUT);
  digitalWrite(PINO_PWM, LOW);
  digitalWrite(PINO_SOLENOIDE, LOW);
  Serial.begin(115200);
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(aoReceber);
  Wire.onRequest(aoPedir);
  Serial.println(F("2-testes-pid rev.5 escravo 0x08 pronto"));
}

void loop() {
  static unsigned long tAnt = 0;
  unsigned long agora = millis();
  float dt = (tAnt == 0) ? 0.1 : (agora - tAnt) / 1000.0;
  tAnt = agora;
  float psi = lerPressaoPSI();
  pvCache = psi * 100;
  stCache = statusNut;

  if (temAlvo) {
    float mira = setpoint + MARGEM_MIRA;
    int u = calcularPID(mira, psi, dt);          // saida BRUTA do PID (intocada)
    float erro = mira - psi;
    uDebug = u; tetoDebug = GANHO_APROX * erro;
    // histerese de seguranca (nao e o corte principal): parou perto, so religa se cair >1.0
    if (erro <= 0.5) freioProx = true;
    else if (erro > 1.0) freioProx = false;
    int pwm = 0;
    if (!freioProx && tFaixa == 0) {
      pwm = acondicionarPWM(u, erro);            // teto proporcional + piso 77
      // rampa (slew-rate): limita SOMENTE a subida; a descida e livre (curva 2)
      if (pwm > 0) {
        if (pwmAnt < PWM_MINIMO) { pwm = PWM_MINIMO; divRampa = 0; }
        else if (pwm > pwmAnt) {
          if (++divRampa >= 5) { divRampa = 0; pwm = pwmAnt + 1; }
          else pwm = pwmAnt;
        }
      }
    }
    pwmAnt = pwm;
    digitalWrite(PINO_SOLENOIDE, HIGH);
    analogWrite(PINO_PWM, pwm);
    pwmAtual = pwm;
    statusNut = 0;
    if (tFaixa == 0 && psi >= setpoint + 0.2) { // passa do SP e para
      tFaixa = agora;
      Serial.print(F("No ponto ")); Serial.println(psi);
    }
    if (tFaixa != 0 && agora - tFaixa >= 2000) {
      analogWrite(PINO_PWM, 0);
      pwmAtual = 0;
      digitalWrite(PINO_SOLENOIDE, LOW);
      temAlvo = false;
      statusNut = 1;
      Serial.println(F("*** NO PONTO, segurando"));
    }
  }

  static unsigned long ultLog = 0;
  if (agora - ultLog >= 1000) {
    ultLog = agora;
    Serial.print(F("sp=")); Serial.print(setpoint, 1);
    Serial.print(F(" pv=")); Serial.print(psi, 2);
    Serial.print(F(" pwm=")); Serial.print(pwmAtual);
    Serial.print(F(" u=")); Serial.print(uDebug);
    Serial.print(F(" teto=")); Serial.print(tetoDebug, 0);
    Serial.print(F(" st=")); Serial.println(statusNut);
  }
}
