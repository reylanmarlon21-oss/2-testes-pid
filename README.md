# 2-testes-pid — Uno escravo I2C + PID de pressão

Arduino Uno que atua como **escravo I2C** (`0x08`) e controla a pressão de um sistema
via **PID real** (`Kp·e + Ki·∫e dt + Kd·de/dt`), acionando compressor (PWM em D9) e
solenoide (D8) a partir da leitura do transmissor em **A0**.

**A faixa do sensor não é fixa:** o mestre (ESP32) informa o mínimo, o máximo e as
tensões de referência (`Vmin`/`Vmax`), e o Uno passa a mapear o A0 nessa faixa. Aceita
faixa **negativa** (ex.: −1 a 5 bar) e qualquer unidade — o PID continua trabalhando em
**PSI** internamente, com a mesma fórmula.

> Mestre correspondente: [esp32-calibracao](https://github.com/reylanmarlon21-oss/esp32-calibracao)

## Hardware

- **Arduino Uno** (ATmega328P)
- Compressor 12V acionado por módulo MOSFET **HW-517** em **D9** (PWM)
- Solenoide 12V por módulo MOSFET **HW-517** em **D8**
- Transmissor de pressão em **A0** — faixa e tipo definidos pelo mestre

> Módulos HW-517 são *low-side*: `VIN+`/`OUT+` ficam em continuidade.
> Recomendado: **diodo de roda livre** em cada carga indutiva, GND comum com a fonte
> de 12V e **pull-down de 10k** nos gatilhos D8/D9.

### Sensores suportados

| Tipo | Tensão no A0 | Como |
|---|---|---|
| **0–5 V** | 0,0 → 5,0 V | o próprio sensor |
| **4–20 mA** | 4 mA·R → 20 mA·R | sobre o shunt (`R = 250 Ω` → **1,0 → 5,0 V**) |

O mapeamento da pressão é linear:

```
psi = min + (V_lido − Vmin) · (max − min) / (Vmax − Vmin)
```

> Com shunt de **250 Ω** o 4–20 mA cai exatamente em 1–5 V. Se o shunt medir mais que
> 250 Ω (ex.: 264 Ω), o topo passa de 5 V e o ADC do Uno satura — ajuste o shunt.

## Controle (PID)

Saída bruta do controlador (intocável):

```
u = Kp*e + Ki*integral + Kd*derivada
```

com `Kp = 25`, `Ki = 4`, `Kd = 6`, derivada filtrada (0.6/0.4) e integral com anti-windup.

A camada de **acondicionamento do atuador** (separada do PID) traduz `u` para o mundo real:

- **Teto proporcional** `teto = GANHO_APROX * erro`, aplicado **só na aproximação**
  (`erro > 2 PSI`) → longe do alvo a força desce suave (`pwm = min(u, teto)`).
- Perto do alvo o **teto é liberado**: quem manda é o PID/integrador. Motivo: sem válvula
  de retenção, com o teto ativo o PWM ficava travado em 77, o fluxo voltava e o SP
  **nunca era alcançado**. O integrador agora acha sozinho o PWM que vence a perda.
- **Piso físico:** o motor não gira entre 1 e 76 → usa-se `PWM_MINIMO = 77` (30%).
- **Degrau final 77 → 0** é inevitável (o motor não faz menos de 30%).
- **Faixa na EEPROM:** a faixa recebida do mestre fica gravada e é restaurada no `setup()`.
  Sem isso, um reset do Uno voltava para 0..30 PSI / 0,5..4,5 V e o A0 mostrava ~9,5 PSI
  com o sistema aberto em 0.
- **Rampa sem corte seco:** sobe +1 a cada 5 ciclos; desce −1 por ciclo e **só chega a 0
  depois de passar por 77** — nunca salta de um valor alto direto para 0.
- **Sobe acima e sangra (approach from above):** o alvo é `SP + Δ` (Δ = 10% do SP,
  limitado a 1,5…5 PSI). A subida vai com **PWM livre** (sem o teto `48·erro`, que travava
  a bomba antes do topo — com alvo 30 PSI ela estacionava em 30,12 e não parava nunca).
  Ao chegar em `SP+Δ`, o motor vira **freio** (a válvula fica aberta como **escape**) e a
  pressão sangra até **SP+0,2**, aí a válvula fecha e segura. O freio não é mais um 77 fixo:
  a cada **250 ms** o Uno mede a **taxa de queda** (`(psi_antes − psi)/Δt`, em PSI/s) e ajusta
  o PWM para descer devagar (alvo de ~0,25 PSI/s perto do alvo, até 0,8 PSI/s longe). Queda
  rápida → sobe o freio; queda fraca → solta o freio (0 = cai livre). Alvos abaixo da
  pressão atual entram direto na sangria (é o que faz a metade de descida da sequência).
  Se a bomba **estagnar** acima do alvo por 2,5 s, o topo é assumido naquele ponto
  (`>>> Topo por estagnacao ...`) em vez de ficar empurrando para sempre.
- **Modo manual (`cmd 5`):** PWM fixo vindo do mestre, sem PID.

Outras proteções: `PWM_MAXIMO = 242` (95%, limite da fonte), mira `+1,0 PSI`,
histerese de 1,0 PSI, ventilação em overshoot, rampa de subida e pausa de 2 s no ponto.

## Protocolo I2C (`0x08`)

**Mestre → Escravo:**

| cmd | Payload | Ação |
|---|---|---|
| `1` | float `sp` (PSI) | ir para o setpoint |
| `2` | — | parar (segura) |
| `3` | — | ventilar |
| `4` | 4 floats: `min, max, Vmin, Vmax` | configura a faixa do sensor |
| `5` | 1 byte `pwm` (0..255) | PWM fixo (modo manual; 0 = para) |
| `6` | float `offset` (PSI) | ajuste de zero, somado a toda leitura do A0 |

- `min`/`max`: faixa de engenharia em **PSI** (aceita negativo). O mestre já converte a
  unidade escolhida (Bar, kgf/cm², …) para PSI antes de enviar.
- `Vmin`/`Vmax`: tensão que o A0 entrega no mínimo e no máximo.
- `offset` (cmd 6): corrige o zero do transmissor/shunt. Fica na **EEPROM** junto da faixa
  (`magic 0xCA12`) e o mestre reenvia a cada 5 s (e ao resetar), então sobrevive ao boot.

**Escravo → Mestre:** 6 bytes `[float pv(PSI), status, pwm]`

- `status`: `0`=subindo · `1`=no ponto · `2`=parado · `3`=ventilando

A resposta usa valores em **cache** (a leitura do ADC fica no `loop`, nunca dentro da ISR),
para não travar o barramento. A config da faixa é aplicada na própria ISR, para valer já no
primeiro alvo da sequência.

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
- **rev.5:** revisão de bancada, sincronizada com o ESP32 rev.ESP32-5
- **rev.6:** faixa vinda do mestre (sem 0–30 PSI fixo), aceita negativo e qualquer unidade;
  setpoint e resposta em **float**; `cmd 4` configura min/max + Vmin/Vmax
- **rev.7:** teto proporcional só na aproximação (`erro > 2 PSI`) — perto do alvo o
  integrador assume e o PWM chega ao valor que vence a perda de fluxo (antes travava em 77
  e o SP nunca era alcançado); ignora o teste de "no ponto" por 300 ms após troca de alvo
  (evitava falso ponto com a leitura da faixa antiga); comandos de bancada `s`/`p` só com
  o sistema parado
- **rev.8:** sobe até `SP+Δ` e **sangra** até SP+0,2 (fecha e segura) — resolve a zona de
  PWM baixo onde a bomba perde vazão; subida sem teto proporcional (o teto travava o alvo
  em 30,12 PSI); queda **freada em 77** (leve, com solta automática se travar); válvula
  aberta como escape; vigia de estagnação do topo; leitura **congelada** ao fechar a
  válvula; faixa persistida na **EEPROM**; `cmd 5` = PWM fixo para o modo manual
- **rev.9:** **descida com taxa controlada** — o freio fixo em 77 ainda deixava a queda
  brusca perto do alvo. Agora, a cada 250 ms: mede a taxa de queda (PSI/s), compara com o
  alvo (0,25 + 0,08·distância, limitado a 0,15…0,8 PSI/s) e ajusta o PWM do freio
  (`passo = 5·erro_de_taxa`, ±18 por janela). Queda rápida → freia mais; queda fraca →
  solta (0 = livre). O solenoide continua abrindo como escape e a rampa 77→0 é mantida.
  O log ganhou o campo `tx=` (taxa de queda medida).
- **rev.10:** **ajuste de zero** (`cmd 6`): o offset em PSI é somado à leitura do A0 antes
  do `constrain` e gravado na **EEPROM** com a faixa (`magic 0xCA12`, campo `off`). Config e
  zero iguais aos atuais não regravam a EEPROM nem poluem o log (o mestre reenvia a cada 5 s).
