# 2-testes-pid — Uno escravo I2C + PID de pressão

Arduino Uno que atua como **escravo I2C** (`0x08`) e controla a pressão de um sistema
via **PID real** (`Kp·e + Ki·∫e dt + Kd·de/dt`), acionando compressor (PWM em D9) e
solenoide (D8) a partir da leitura do transmissor em **A0**.

Recebe os setpoints do mestre (ESP32) e responde com a pressão lida, o status e o PWM atual.

> Mestre correspondente: [esp32-calibracao](https://github.com/reylanmarlon21-oss/esp32-calibracao)

## Hardware

- **Arduino Uno** (ATmega328P)
- Compressor 12V acionado por módulo MOSFET **HW-517** em **D9** (PWM)
- Solenoide 12V por módulo MOSFET **HW-517** em **D8**
- Transmissor de pressão 0–30 PSI por **A0** (0,5 V = 0 PSI · 4,5 V = 30 PSI)

> Módulos HW-517 são *low-side*: `VIN+`/`OUT+` ficam em continuidade.
> Recomendado: **diodo de roda livre** em cada carga indutiva, GND comum com a fonte
> de 12V e **pull-down de 10k** nos gatilhos D8/D9.

## Controle (PID)

Saída bruta do controlador (intocável):

```
u = Kp*e + Ki*integral + Kd*derivada
```

com `Kp = 25`, `Ki = 4`, `Kd = 6`, derivada filtrada (0.6/0.4) e integral com anti-windup.

A camada de **acondicionamento do atuador** (separada do PID) traduz `u` para o mundo real:

- **Teto proporcional** `teto = GANHO_APROX * erro` → quanto mais perto do alvo, menor a
  força (`pwm = min(u, teto)`), fazendo o PWM **descer suave** na chegada (evita o corte seco).
- **Piso físico:** o motor não gira entre 1 e 76 → usa-se `PWM_MINIMO = 77` (30%).
- **Degrau final 77 → 0** é inevitável (o motor não faz menos de 30%).

Outras proteções: `PWM_MAXIMO = 242` (95%, limite da fonte), mira `+1,0 PSI`,
histerese de 1,0 PSI, ventilação em overshoot, rampa de subida e pausa de 2 s no ponto.

## Protocolo I2C (`0x08`)

- **Recebe:** `[cmd, sp_lo, sp_hi]` — setpoint em centi-PSI
  - `cmd 1` = ir para setpoint · `2` = parar · `3` = ventilar
- **Responde:** 4 bytes `[pv_lo, pv_hi, status, pwm]`
  - `status`: `0`=subindo · `1`=no ponto · `2`=parado · `3`=ventilando

A resposta usa valores em **cache** (a leitura do ADC fica no `loop`, nunca dentro da ISR),
para não travar o barramento.

## Build / Upload

Requer [PlatformIO](https://platformio.org/). Da pasta do projeto:

```bash
pio run                 # compila
pio run -t upload       # compila e grava (opcional: --upload-port COMx)
```

> Atenção: abrir o monitor serial reseta o Uno e trava a porta (impede o upload).

## Revisões

- **rev.1:** Uno escravo I2C 0x08 + PID real do `1-teste-pid`
- **rev.2:** reenvio não zera a rampa + 4º byte (PWM) + sanidade 0–30 PSI
- **rev.3:** trava em `SP+0.2` (passa do ponto e para) + reenvio não rearma ponto feito
- **rev.4:** núcleo PID intocado + teto proporcional (`G*erro`) → descida suave até 77
- **rev.5:** revisão atual de bancada, sincronizada com o ESP32 rev.ESP32-5
