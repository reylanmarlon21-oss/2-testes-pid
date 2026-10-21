// rev.1 2-testes-pid: Uno escravo I2C 0x08 + PID real do 1-teste-pid
// rev.2: reenvio nao zera rampa + 4o byte PWM + sanidade 0-30 PSI
// rev.3: trava em SP+0.2 (passa do ponto e para) + reenvio nao rearma ponto feito
// rev.4: NUCLEO PID (Kp,Ki,Kd) intocado + camada de atuador com teto proporcional
//        G*erro -> a forca DESCE suave ate os 77 (30%) e so entao corta (curva 2)
// rev.5: revisao de bancada, sincronizada com o ESP32 rev.ESP32-5
// rev.6: FAIXA VINDA DA ESP32 (nao tem mais 0-30 PSI fixo). O mestre manda
//        min/max (em PSI) + Vmin/Vmax do sensor (0-5V ou 4-20mA) via cmd 4.
//        Aceita faixa negativa (ex.: -1..5 bar). O PID continua trabalhando em
//        PSI com a MESMA formula (intocada); so o MAPEAMENTO do A0 usa a faixa.
// rev.7: (a) TETO PROPORCIONAL SO NA APROXIMACAO (erro > 2 PSI). Perto do alvo
//            o teto e liberado: antes ele travava o PWM em 77 e, como a bomba
//            nao tem valvula de retencao, o fluxo voltava e o SP nunca chegava.
//            Agora quem manda perto do alvo e o PID/integrador (busca sozinho o
//            PWM que vence a perda). Longe do alvo a descida suave continua.
//        (b) Falso "No ponto" apos troca de alvo: ignora o teste por 300 ms
//            (a leitura psi do ciclo anterior ainda era da faixa antiga).
//        (c) Comandos de bancada 's'/'p' so funcionam com o sistema parado.
// rev.8: aproximacao por cima (sobe SP+delta e sangra), freio em 77, trava de
//        pvNoPonto, cmd 5 (PWM manual) e faixa gravada na EEPROM.
// rev.9: DESCIDA COM TAXA CONTROLADA. Antes era so "motor em 77" e a queda
//        vinha brusca (perto do alvo o 77 quase nao segurava). Agora, a cada
//        250 ms, mede-se a taxa de queda (PSI/s) e ajusta-se o PWM do freio
//        para descer devagar (alvo ~0,25..0,8 PSI/s). O solenoide segue ABERTO
//        (escape) e o motor freia a saida. Rampa 77 -> 0 mantida.
// rev.10: AJUSTE DE ZERO (cmd 6 + float em PSI): soma na leitura do A0 antes do
//         constrain (corrige o offset do transmissor/shunt). Guardado na EEPROM
//         junto da faixa (magic 0xCA12).
// rev.11: DUAS VALVULAS.
//         D8 = PORTA: abre enquanto o motor gira (gerando pressao) e FECHA quando
//              o motor para -> segura a pressao.
//         D6 = DESCARGA (solenoide nova): descer a pressao, despressurizar, aliviar
//              e alivio de seguranca. Na descida o motor fica parado e a descarga
//              pulsa (largura de pulso numa janela de 1 s) para a queda ser gradual.
//         PROTECAO: se pv > limite (5o float do cmd 4 = maior faixa de A1..A4),
//              para o motor, abre a descarga e reporta status 4. Fecha sozinha
//              quando a pressao cai 0,5 PSI abaixo do limite. Alvos sao ignorados
//              enquanto a protecao esta ativa.
// Mestre (ESP32) envia:
//   cmd 1 + float sp(PSI)                     -> ir para setpoint
//   cmd 2                                     -> parar
//   cmd 3                                     -> despressurizar/ventilar (D6)
//   cmd 4 + 5 floats {min,max,Vmin,Vmax,lim}  -> faixa do sensor (PSI/V) + limite
//   cmd 5 + byte pwm                          -> PWM manual (0 = para)
//   cmd 6 + float off(PSI)                    -> ajuste de zero do A0
// Escravo responde 6 bytes: [float pv(PSI), status, pwm]
//   status 0=subindo 1=no ponto 2=parado 3=despressurizando 4=LIMITE ULTRAPASSADO
// Protecoes: PWM 0 ou 77-242, rampa, porta fecha com motor parado, descarga so
// abre em descida/alivio, limite de pressao com histerese de 0,5 PSI.

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define SLAVE_ADDR      0x08
#define PINO_PWM        9
#define PINO_PORTA      8    // porta do ar: aberta enquanto o motor gera pressao
#define PINO_DESCARGA   6    // solenoide de descarga/alivio (nova, igual a do D8)
#define PINO_PRESSAO    A0

#define PWM_MAXIMO      242
#define PWM_MINIMO      77
#define MARGEM_MIRA     1.0
#define GANHO_APROX     48.0   // teto de aproximacao: teto = GANHO_APROX * erro

float Kp = 25.0, Ki = 4.0, Kd = 6.0;
float integral = 0, ultimoErro = 0, derivFilt = 0;

float setpoint = 0;          // PSI
uint8_t statusNut = 2;       // comeca parado
volatile float pvCacheF = 0; // pressao (PSI) p/ resposta I2C
volatile uint8_t stCache = 2;
bool temAlvo = false;
bool freioProx = false;
int pwmAnt = 0, divRampa = 0, pwmAtual = 0;
int uDebug = 0; float tetoDebug = 0; // so p/ log (afinacao do teto proporcional)
unsigned long tFaixa = 0;
unsigned long tAlvoNovo = 0; // instante do ultimo irPara (evita falso "no ponto")
int  pwmRampa = 0;           // PWM atual da rampa (global: parar() zera na hora)
int  manualPwm = -1;         // >=0 = modo manual (cmd 5); -1 = inativo
bool faseDescer = false;     // true = ja passou do topo, agora sangra ate o SP
float overshootAlvo = 1.5;   // quanto acima do SP subir antes de sangrar
float psiTopo = -999;        // maior pressao vista na subida (vigia de estagnacao)
unsigned long tTopo = 0;     // instante da ultima subida de pressao
float pvNoPonto = 0;         // leitura congelada no instante em que a descarga fechou
int  dutyDescarga = 0;       // % de abertura da descarga na janela atual (descida)
float psiTaxa = 0;           // pressao no inicio da janela de medida da taxa
unsigned long tTaxa = 0;     // instante do inicio dessa janela
unsigned long tJanela = 0;   // inicio da janela de pulso da descarga
float taxaQueda = 0;         // PSI/s medidos (positivo = caindo) - so para o log
bool limiteAtivo = false;    // protecao de pressao disparada (status 4)

#define JANELA_DESCARGA 1000UL   // ms: janela de medida da taxa e de pulso da descarga

// ---- Faixa do sensor (chega do mestre via cmd 4). Default = comportamento antigo ----
float cfgMinPSI = 0.0, cfgMaxPSI = 30.0; // faixa de engenharia em PSI
float cfgVmin  = 0.5, cfgVmax  = 4.5;    // tensao do A0 no minimo e no maximo
float cfgOffsetPSI = 0.0f;               // ajuste de zero (cmd 6), somado a leitura
float cfgLimitePSI = 0.0f;               // protecao (cmd 4): abre a descarga acima disto
volatile bool cfgNova = false;           // so p/ logar no loop (a aplicacao e na ISR)
volatile bool cfgSalvar = false;         // grava a config na EEPROM (fora da ISR)
volatile bool cfgOffsetNova = false;     // so p/ logar o novo zero

// A config vive na EEPROM: se o Uno reiniciar sozinho (ou o mestre estiver
// desligado), ele continua lendo com a ULTIMA faixa em vez dos defaults
// (0..30 PSI / 0,5..4,5 V), que davam leitura errada — ex.: 9,5 PSI com o
// sistema aberto em 0.
#define CFG_MAGIC 0xCA13
struct ConfigEEPROM { uint16_t magic; float mn, mx, vmn, vmx, off, lim; };

int adcUltimo = 0;            // media crua do A0 (conta 0..1023) - so diagnostico
float vUltimo = 0;            // tensao no shunt (V) - so diagnostico
float pvBrutoPSI = 0;         // leitura SEM o clamp da faixa (protecao de sobrepressao)
bool solTeste = false, pwmTeste = false; // diagnostico de bancada (serial: s / p)
unsigned long diagAte = 0;

float lerPressaoPSI() {
  noInterrupts(); // snapshot atomico da config (pode estar sendo escrita na ISR)
  float mn = cfgMinPSI, mx = cfgMaxPSI, vmn = cfgVmin, vmx = cfgVmax;
  float off = cfgOffsetPSI;
  interrupts();
  long s = 0;
  for (int i = 0; i < 16; i++) { s += analogRead(PINO_PRESSAO); delay(2); }
  float v = (s / 16.0) * 5.0 / 1023.0;
  adcUltimo = (int)(s / 16);
  vUltimo = v;
  // mapeamento linear usando a FAIXA RECEBIDA (serve p/ negativo e p/ qualquer unidade)
  float psi = mn + (v - vmn) * (mx - mn) / (vmx - vmn);
  psi += off;   // ajuste de zero vindo do mestre (cmd 6)
  pvBrutoPSI = psi;                       // sem clamp: e o que a protecao vigia
  return constrain(psi, mn, mx);
}

// ================== NUCLEO PID ==================
// u(t) = Kp*e(t) + Ki*Integral(e dt) + Kd*de/dt      <<< FORMULA INTOCAVEL
// Retorna a saida BRUTA do controlador. Piso/corte de motor NAO moram aqui:
// isso e responsabilidade do acondicionamento (acondicionarPWM), logo abaixo.
int calcularPID(float sp, float pv, float dt) {
  if (dt <= 0) dt = 0.05;
  float erro = sp - pv;
  // overshoot: ventila. Limite relativo abaixo (vale tb p/ sp negativo ou zero).
  float limOvershoot = sp + 0.05f * fmax(fabs(sp), 1.0f);
  if (pv > limOvershoot) return 0;   // passou do teto: corta a bomba (porta fecha junto)
  float dBruto = (erro - ultimoErro) / dt;
  derivFilt = 0.6 * dBruto + 0.4 * derivFilt;        // derivada filtrada (anti-ruido)
  ultimoErro = erro;
  if (fabs(erro) > 0.3) integral += erro * dt;       // so integra fora de 0.3 (anti-windup)
  integral = constrain(integral, -200.0 / Ki, 200.0 / Ki);
  if (fabs(erro) <= 0.01 * fabs(sp)) integral = 0;
  float u = Kp * erro + Ki * integral + Kd * derivFilt;   // <<< PID
  if (!isfinite(u)) u = 0;
  if (u < 0) u = 0;
  if (u > PWM_MAXIMO) u = PWM_MAXIMO;
  return (int)u;
}

// ============ ACONDICIONAMENTO DO ATUADOR ============
// Camada SEPARADA do PID. So traduz a saida u para o mundo real do motor:
//  - teto proporcional: teto = GANHO_APROX*erro, mas SO na APROXIMACAO (erro>2).
//    Perto do alvo o teto e liberado: senao ele trava o PWM em 77 e a bomba
//    (sem valvula de retencao) perde fluxo e NUNCA chega no SP. Perto do alvo
//    quem manda e o PID/integrador, que acha sozinho o PWM que vence a perda.
//  - piso fisico: o motor nao gira entre 1..76, entao mantem PWM_MINIMO.
//  - o degrau final 77 -> 0 e inevitavel (motor nao faz menos que 30%).
int acondicionarPWM(int u, float erro) {
  if (u <= 0) return 0;
  if (erro > 2.0) {                     // teto so enquanto esta longe do alvo
    float teto = GANHO_APROX * erro;
    if (teto < u) u = (int)teto;        // min(u, teto): PID decide, teto limita
  }
  if (u > PWM_MAXIMO) u = PWM_MAXIMO;
  if (u < PWM_MINIMO) u = PWM_MINIMO;   // nunca 1..76
  return u;
}

// Subida ate SP+Delta: MESMA camada, mas SEM o teto proporcional.
// O teto 48*erro limita o PWM justamente nos ultimos PSI (erro 2,9 -> PWM 138)
// e a bomba estaciona ANTES de chegar em SP+Delta (foi o que travou o alvo
// de 30 PSI em 30,12). Piso 77 / maximo / rampa continuam valendo.
int acondicionarPWMSubida(int u) {
  if (u <= 0) return 0;
  if (u > PWM_MAXIMO) u = PWM_MAXIMO;
  if (u < PWM_MINIMO) u = PWM_MINIMO;
  return u;
}

void irPara(float sp) {
  if (sp < cfgMinPSI || sp > cfgMaxPSI) { // sanidade: so dentro da faixa configurada
    Serial.print(F("Alvo fora da faixa, ignorado: ")); Serial.println(sp);
    return;
  }
  if (limiteAtivo) { // protecao ativa: nao gera pressao nenhuma
    Serial.print(F("Alvo ignorado (limite de pressao ativo): ")); Serial.println(sp);
    return;
  }
  if (fabs(sp - setpoint) < 0.01 && (temAlvo || statusNut == 1)) return; // reenvio: mantem estado
  setpoint = sp; integral = 0; ultimoErro = 0; derivFilt = 0;

  // ---- subir ACIMA do alvo e depois sangrar ate ele ----
  // Evita a zona de PWM baixo, onde a bomba (sem valvula de retencao) perde
  // vazao e o SP nunca chega. Delta = 10% do alvo, entre 1.5 e 5 PSI.
  overshootAlvo = 0.10f * fabs(sp);
  if (overshootAlvo < 1.5f) overshootAlvo = 1.5f;
  if (overshootAlvo > 5.0f) overshootAlvo = 5.0f;
  if (sp + overshootAlvo > cfgMaxPSI) overshootAlvo = fmax(0.0f, cfgMaxPSI - sp);
  // se ja esta acima do topo, nem sobe: vai direto para a sangria
  faseDescer = (pvCacheF > sp + overshootAlvo);
  psiTopo = pvCacheF; tTopo = millis();   // vigia de estagnacao da subida
  dutyDescarga = 25; psiTaxa = pvCacheF; tTaxa = millis(); tJanela = millis(); taxaQueda = 0;

  manualPwm = -1;                       // qualquer alvo cancela o modo manual
  freioProx = false; pwmAnt = 0; divRampa = 0;
  tFaixa = 0; temAlvo = true; statusNut = 0;
  tAlvoNovo = millis();          // ignora o teste de "no ponto" por 300ms (psi antigo)
  Serial.print(F(">>> Alvo ")); Serial.print(sp);
  Serial.print(F(" (sobe ate ")); Serial.print(sp + overshootAlvo, 2);
  Serial.println(faseDescer ? F(", ja acima: so sangra)") : F(")"));
}

void parar(bool despressurizar) {
  analogWrite(PINO_PWM, 0);
  digitalWrite(PINO_PWM, LOW);
  pwmAtual = 0; pwmRampa = 0; temAlvo = false; manualPwm = -1; faseDescer = false;
  dutyDescarga = 0; taxaQueda = 0;
  psiTopo = -999; tTopo = 0;
  statusNut = despressurizar ? 3 : 2;   // 3 = descarga aberta (quem abre e o loop)
  Serial.println(despressurizar ? F("Despressurizando (descarga aberta)") : F("Parado"));
}

void aplicarConfig(float mn, float mx, float vmn, float vmx, float lim) {
  // aplica direto (roda na ISR do I2C): sem Serial aqui. Valida antes de aceitar.
  if (mx <= mn || vmx <= vmn) return;
  // o mestre reenvia a cada 5 s: se nada mudou, nao mexe na EEPROM nem no log
  if (fabs(mn - cfgMinPSI) < 0.001f && fabs(mx - cfgMaxPSI) < 0.001f &&
      fabs(vmn - cfgVmin) < 0.001f && fabs(vmx - cfgVmax) < 0.001f &&
      fabs(lim - cfgLimitePSI) < 0.001f) return;
  cfgMinPSI = mn; cfgMaxPSI = mx; cfgVmin = vmn; cfgVmax = vmx;
  cfgLimitePSI = (isfinite(lim) && lim != 0.0f) ? lim : 0.0f;   // 0 = sem protecao
  cfgNova = true; cfgSalvar = true;
}

void aoReceber(int n) {
  if (n < 1) { while (Wire.available()) Wire.read(); return; }
  uint8_t cmd = Wire.read();

  if (cmd == 4) { // configuracao: 5 floats (20 bytes) = min, max, Vmin, Vmax, limite
    if (Wire.available() < 20) { while (Wire.available()) Wire.read(); return; }
    uint8_t b[20];
    for (int i = 0; i < 20; i++) b[i] = Wire.read();
    while (Wire.available()) Wire.read();
    float mn, mx, vmn, vmx, lim;
    memcpy(&mn,  b,      4);
    memcpy(&mx,  b + 4,  4);
    memcpy(&vmn, b + 8,  4);
    memcpy(&vmx, b + 12, 4);
    memcpy(&lim, b + 16, 4);
    aplicarConfig(mn, mx, vmn, vmx, lim);
    return;
  }

  if (cmd == 1) { // ir para setpoint (float, PSI)
    if (Wire.available() < 4) { while (Wire.available()) Wire.read(); return; }
    uint8_t b[4];
    for (int i = 0; i < 4; i++) b[i] = Wire.read();
    while (Wire.available()) Wire.read();
    float sp; memcpy(&sp, b, 4);
    irPara(sp);
    return;
  }

  if (cmd == 5) { // PWM manual (0..255): 0 = para, 1..76 vai para 77
    if (!Wire.available()) return;
    int p = Wire.read();
    while (Wire.available()) Wire.read();
    if (limiteAtivo) { Serial.println(F("Manual ignorado (limite ativo)")); return; }
    temAlvo = false; faseDescer = false;
    if (p <= 0) { manualPwm = 0; statusNut = 2; }
    else        { manualPwm = constrain(p, PWM_MINIMO, PWM_MAXIMO); statusNut = 0; }
    return;
  }

  if (cmd == 6) { // ajuste de zero do A0 (float, PSI): soma na leitura
    if (Wire.available() < 4) { while (Wire.available()) Wire.read(); return; }
    uint8_t b[4];
    for (int i = 0; i < 4; i++) b[i] = Wire.read();
    while (Wire.available()) Wire.read();
    float off; memcpy(&off, b, 4);
    if (isfinite(off) && fabs(off) <= 50.0f && fabs(off - cfgOffsetPSI) > 0.001f) {
      cfgOffsetPSI = off; cfgOffsetNova = true; cfgSalvar = true;
    }
    return;
  }

  while (Wire.available()) Wire.read();
  if (cmd == 2) parar(false);
  else if (cmd == 3) parar(true);
}

void aoPedir() {
  // rapido: so envia cache (leitura do ADC fica no loop, nunca na ISR)
  // Com o solenoide fechado no ponto (status 1), devolve a leitura CONGELADA:
  // com o transmissor no lado da bomba, a linha sangra ao fechar e o pv ao vivo
  // cairia para ~0, gravando um valor errado.
  float pv = (statusNut == 1) ? pvNoPonto : pvCacheF;
  uint8_t b[4]; memcpy(b, &pv, 4);
  Wire.write(b, 4);
  Wire.write(stCache);
  Wire.write((uint8_t)pwmAtual); // PWM injetado p/ a tela
}

void setup() {
  pinMode(PINO_PWM, OUTPUT);
  pinMode(PINO_PORTA, OUTPUT);
  pinMode(PINO_DESCARGA, OUTPUT);
  digitalWrite(PINO_PWM, LOW);
  digitalWrite(PINO_PORTA, LOW);
  digitalWrite(PINO_DESCARGA, LOW);

  // restaura a ultima faixa/limite recebidos do mestre (sobrevive a reset do Uno)
  ConfigEEPROM cfgE;
  EEPROM.get(0, cfgE);
  if (cfgE.magic == CFG_MAGIC && cfgE.mx > cfgE.mn && cfgE.vmx > cfgE.vmn) {
    cfgMinPSI = cfgE.mn; cfgMaxPSI = cfgE.mx; cfgVmin = cfgE.vmn; cfgVmax = cfgE.vmx;
    cfgOffsetPSI = cfgE.off;
    cfgLimitePSI = (isfinite(cfgE.lim) && cfgE.lim != 0.0f) ? cfgE.lim : 0.0f;
  }

  Serial.begin(115200);
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(aoReceber);
  Wire.onRequest(aoPedir);
  Serial.println(F("2-testes-pid rev.11 escravo 0x08 pronto (porta D8 + descarga D6)"));
}

void loop() {
  static unsigned long tAnt = 0;
  unsigned long agora = millis();
  float dt = (tAnt == 0) ? 0.1 : (agora - tAnt) / 1000.0;
  tAnt = agora;

  // ---- diagnostico de bancada (nao mexe no PID: use com o sistema parado) ----
  while (Serial.available()) {
    char c = Serial.read();
    if (temAlvo) continue;              // nunca durante a calibracao (o PID manda)
    if (c == 's') {
      solTeste = !solTeste;
      digitalWrite(PINO_DESCARGA, solTeste ? HIGH : LOW);
      Serial.print(F(">> DESCARGA ")); Serial.println(solTeste ? F("ON") : F("OFF"));
      diagAte = millis() + 4000;
    } else if (c == 'p') {
      pwmTeste = !pwmTeste;
      analogWrite(PINO_PWM, pwmTeste ? 120 : 0);
      if (!pwmTeste) { digitalWrite(PINO_PWM, LOW); }
      Serial.print(F(">> PWM ")); Serial.println(pwmTeste ? 120 : 0);
      diagAte = millis() + 4000;
    } else if (c == 'l') {   // teste da protecao: limite 2 PSI abaixo da leitura atual
      cfgLimitePSI = pvBrutoPSI - 2.0f;
      Serial.print(F(">> LIMITE DE TESTE ")); Serial.print(cfgLimitePSI, 2);
      Serial.println(F(" PSI (o mestre devolve o limite real em ate 5 s)"));
      diagAte = millis() + 4000;
    }
  }

  if (cfgNova) { // so loga (a config ja foi aplicada na ISR do I2C)
    cfgNova = false;
    Serial.print(F("Config recebida: ")); Serial.print(cfgMinPSI, 2); Serial.print(F("..")); Serial.print(cfgMaxPSI, 2);
    Serial.print(F(" PSI | V ")); Serial.print(cfgVmin, 3); Serial.print(F("..")); Serial.println(cfgVmax, 3);
  }
  if (cfgOffsetNova) { // so loga (o valor ja foi aplicado na ISR do I2C)
    cfgOffsetNova = false;
    Serial.print(F("Zero do A0: offset ")); Serial.print(cfgOffsetPSI, 2); Serial.println(F(" PSI"));
  }
  if (cfgSalvar) { // grava na EEPROM (put so escreve o que mudou)
    cfgSalvar = false;
    ConfigEEPROM cfgE;
    noInterrupts(); // snapshot consistente (a ISR pode estar trocando os floats)
    cfgE.magic = CFG_MAGIC; cfgE.mn = cfgMinPSI; cfgE.mx = cfgMaxPSI;
    cfgE.vmn = cfgVmin; cfgE.vmx = cfgVmax; cfgE.off = cfgOffsetPSI; cfgE.lim = cfgLimitePSI;
    interrupts();
    EEPROM.put(0, cfgE);
    Serial.println(F("Config gravada na EEPROM"));
  }

  float psi = lerPressaoPSI();
  pvCacheF = psi;
  stCache = statusNut;

  // ---- PROTECAO DE PRESSAO (descarga como alivio) ----
  // Limite = maior faixa entre A1..A4, mandado pelo mestre no cmd 4 (0 = desligada).
  if (cfgLimitePSI != 0.0f) {   // 0 = sem protecao (limite pode ser negativo: vacuo)
    if (!limiteAtivo && pvBrutoPSI > cfgLimitePSI) {
      limiteAtivo = true;
      temAlvo = false; manualPwm = -1; faseDescer = false;
      pwmRampa = 0; pwmAtual = 0; divRampa = 0;
      analogWrite(PINO_PWM, 0); digitalWrite(PINO_PWM, LOW);
      statusNut = 4;                          // 4 = LIMITE ULTRAPASSADO
      Serial.print(F("!!! LIMITE ULTRAPASSADO (")); Serial.print(pvBrutoPSI, 2);
      Serial.print(F(" > ")); Serial.print(cfgLimitePSI, 2);
      Serial.println(F(" PSI): motor parado, descarga ABERTA"));
    } else if (limiteAtivo && pvBrutoPSI < cfgLimitePSI - 0.5f) {   // histerese de 0,5
      limiteAtivo = false;
      statusNut = 2;
      Serial.print(F("Limite normalizado em ")); Serial.print(pvBrutoPSI, 2);
      Serial.println(F(" PSI: descarga fechada"));
    }
  }

  int alvoPwm = 0;

  if (temAlvo) {
    // ---- fase 1: SUBIR (PID manda) ate SP + overshoot ----
    if (!faseDescer) {
      float mira = setpoint + overshootAlvo;
      int u = calcularPID(mira, psi, dt);          // saida BRUTA do PID (intocada)
      uDebug = u; tetoDebug = 999.0f;              // subida sem teto proporcional
      alvoPwm = acondicionarPWMSubida(u);          // piso 77 + maximo (sem travar)
      if (psi >= mira) {
        faseDescer = true; dutyDescarga = 25;
        psiTaxa = psi; tTaxa = agora; tJanela = agora; taxaQueda = 0;
        Serial.print(F(">>> Topo ")); Serial.print(psi);
        Serial.print(F(" (alvo ")); Serial.print(setpoint);
        Serial.println(F("): motor para e a descarga sangra ate o alvo"));
      } else if (psi > psiTopo + 0.10f) {
        psiTopo = psi; tTopo = agora;              // ainda subindo: reinicia o vigia
      } else if (agora - tTopo >= 2500 && psi >= setpoint - 0.2f) {
        // bomba no limite: nao sobe mais. Considera que chegou no topo.
        faseDescer = true; dutyDescarga = 25;
        psiTaxa = psi; tTaxa = agora; tJanela = agora; taxaQueda = 0;
        Serial.print(F(">>> Topo por estagnacao em ")); Serial.print(psi);
        Serial.print(F(" (alvo ")); Serial.print(setpoint);
        Serial.print(F(", mira ")); Serial.print(mira);
        Serial.print(F(", pwm ")); Serial.print(pwmRampa);
        Serial.println(F("): motor para e a descarga sangra"));
      }
    }
    // ---- fase 2: DESCER pela DESCARGA (motor parado; D6 pulsa em janelas) ----
    if (faseDescer) {
      if (psi <= setpoint + 0.2) {                 // cruzou o alvo VINDO DE CIMA
        pvNoPonto = psi;                           // congela ANTES de fechar (leitura valida)
        temAlvo = false;
        statusNut = 1;
        dutyDescarga = 0;                          // fecha a descarga na hora
        Serial.print(F("*** NO PONTO (descendo), segurando ")); Serial.println(pvNoPonto);
      } else {
        alvoPwm = 0;                               // motor parado: quem sangra e a D6
        // Janela de 1 s: mede a taxa de queda e ajusta a % de abertura da descarga.
        // Janela longa = taxa menos contaminada pelo ruido do A0 (+-0,3 PSI).
        if (agora - tTaxa >= JANELA_DESCARGA) {
          float seg = (agora - tTaxa) / 1000.0f;
          if (seg > 0.05f) {
            float dAlvo = psi - setpoint;
            float alvoTaxa = 0.25f + 0.08f * dAlvo;   // PSI/s (bem devagar)
            if (alvoTaxa > 0.8f) alvoTaxa = 0.8f;
            if (alvoTaxa < 0.15f) alvoTaxa = 0.15f;
            float taxaNova = (psiTaxa - psi) / seg;   // > 0 = caindo
            taxaQueda = 0.5f * taxaNova + 0.5f * taxaQueda;  // filtra o ruido
            float erroTaxa = taxaQueda - alvoTaxa;    // > 0 = caindo rapido demais
            if (erroTaxa > 0.05f || erroTaxa < -0.05f) {
              int passo = (int)(-erroTaxa * 120.0f);  // % de abertura por (PSI/s) de erro
              if (passo > 35) passo = 35;
              if (passo < -35) passo = -35;
              dutyDescarga += passo;
              if (dutyDescarga < 0) dutyDescarga = 0;
              if (dutyDescarga > 100) dutyDescarga = 100;
              if (dutyDescarga > 0 && dutyDescarga < 5) dutyDescarga = 5;
              Serial.print(F("descarga ")); Serial.print(dutyDescarga);
              Serial.print(F("% (queda ")); Serial.print(taxaQueda, 2);
              Serial.print(F(" PSI/s, alvo ")); Serial.print(alvoTaxa, 2);
              Serial.print(F(", psi ")); Serial.print(psi, 2);
              Serial.println(F(")"));
            }
          }
          psiTaxa = psi; tTaxa = agora; tJanela = agora;
        }
      }
    }
    if (temAlvo) statusNut = 0;
  } else if (manualPwm >= 0) {                     // ---- modo MANUAL (cmd 5) ----
    alvoPwm = manualPwm;
  }

  // ---- rampa: nunca corta de um valor alto direto para 0 ----
  if (alvoPwm < 0) alvoPwm = 0;
  if (alvoPwm > PWM_MAXIMO) alvoPwm = PWM_MAXIMO;
  if (alvoPwm > 0 && alvoPwm < PWM_MINIMO) alvoPwm = PWM_MINIMO;
  if (alvoPwm > pwmRampa) {                        // subida suave: +1 a cada 5 ciclos
    if (pwmRampa < PWM_MINIMO) { pwmRampa = PWM_MINIMO; divRampa = 0; }
    else if (faseDescer) pwmRampa = alvoPwm;       // na sangria o freio responde direto
    else if (++divRampa >= 5) { divRampa = 0; pwmRampa++; }
  } else if (alvoPwm < pwmRampa) {                 // descida: -1 por ciclo...
    divRampa = 0;
    int passo = faseDescer ? 4 : 1;                // no DESC o motor para rapido
    if (pwmRampa - passo > PWM_MINIMO) pwmRampa -= passo;
    else if (pwmRampa > PWM_MINIMO) pwmRampa = PWM_MINIMO;
    else pwmRampa = alvoPwm;                       // ...e so entao 77 -> 0
  }
  pwmAnt = pwmRampa;
  analogWrite(PINO_PWM, pwmRampa);
  pwmAtual = pwmRampa;

  // ---- VALVULAS ----
  // D8 (PORTA): abre enquanto o motor gira (gerando pressao) e fecha quando o
  // motor para -> segura a pressao. Vale para subida, manual e parada.
  digitalWrite(PINO_PORTA, (pwmRampa > 0) ? HIGH : LOW);
  // D6 (DESCARGA): so abre na descida (pulsando), no despressurizar/ventilar
  // (status 3) ou na protecao de pressao.
  bool abrirDescarga = false;
  if (limiteAtivo) abrirDescarga = true;
  else if (statusNut == 3) abrirDescarga = true;
  else if (temAlvo && faseDescer) {
    unsigned long dentro = agora - tJanela;
    abrirDescarga = (dentro * 100UL) < ((unsigned long)dutyDescarga * JANELA_DESCARGA);
  }
  digitalWrite(PINO_DESCARGA, abrirDescarga ? HIGH : LOW);

  static unsigned long ultLog = 0;
  if (agora - ultLog >= 1000) {
    ultLog = agora;
    Serial.print(F("adc=")); Serial.print(adcUltimo);
    Serial.print(F(" v=")); Serial.print(vUltimo, 3);
    Serial.print(F(" sol=")); Serial.print(digitalRead(PINO_PORTA));
    Serial.print(F(" sp=")); Serial.print(setpoint, 1);
    Serial.print(F(" pv=")); Serial.print(psi, 2);
    Serial.print(F(" pwm=")); Serial.print(pwmAtual);
    Serial.print(F(" tx=")); Serial.print(taxaQueda, 2);
    Serial.print(F(" u=")); Serial.print(uDebug);
    Serial.print(F(" teto=")); Serial.print(tetoDebug, 0);
    Serial.print(F(" st=")); Serial.print(statusNut);
    Serial.print(F(" duty=")); Serial.print(dutyDescarga);
    Serial.print(F(" desc=")); Serial.print(digitalRead(PINO_DESCARGA));
    Serial.print(F(" lim=")); Serial.println(cfgLimitePSI, 2);
  }

  static unsigned long ultDiag = 0;
  if (agora < diagAte && agora - ultDiag >= 100) {
    ultDiag = agora;
    Serial.print(F("  adc=")); Serial.print(adcUltimo);
    Serial.print(F(" v=")); Serial.print(vUltimo, 4);
    Serial.print(F(" sol=")); Serial.print(digitalRead(PINO_PORTA));
    Serial.print(F(" desc=")); Serial.print(digitalRead(PINO_DESCARGA));
    Serial.print(F(" pv=")); Serial.println(psi, 2);
  }
}
