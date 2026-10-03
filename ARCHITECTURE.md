# Arquitetura do Sistema ROCSAR

O **ROCSAR** e um sistema embarcado de apontamento autonomo e compensacao de
movimento para antenas a bordo de uma plataforma aerea/gondola. O objetivo e
manter as antenas orientadas continuamente para um alvo (*Target Heading*),
compensando as rotacoes e a turbulencia da plataforma em tempo real.

Este documento tem duas partes: os **requisitos** (o que o sistema tem de fazer)
e as **decisoes de implementacao** (o porque de cada escolha). O `README.md`
cobre o *como usar*.

---

## 1. Visao geral

```
+-----------------------------------------------------------------------------------+
|                               GROUND STATION (GS)                                 |
|  - Envia CommandRequest (Protobuf) via ZMQ DEALER                                  |
|  - Recebe TelemetryFrame (Protobuf) via ZMQ SUB                                    |
+-----------------------------------------------------------------------------------+
                                    |
                          Ethernet (Linux tc)
                                    |
+-----------------------------------------------------------------------------------+
|                       OBC (Raspberry Pi 4B) - SERVIDOR                            |
|                                                                                   |
|  +------------------------+  +------------------------+  +---------------------+  |
|  | ZMQ ROUTER (Port 5555) |  | ZMQ PUB (Port 5556)    |  | HTTP (Port 8080)    |  |
|  | (Descodifica Protobuf) |  | (Serializa Protobuf)   |  | (Download de SSD)   |  |
|  +------------------------+  +------------------------+  +---------------------+  |
|               |                          ^                          |             |
|               +------------+-------------+                          |             |
|                            |                                        v             |
|             +------------------------------+              +------------------+    |
|             | Thread Leitora UDP (GNSS)    |              | Armazenamento SSD|    |
|             +------------------------------+              +------------------+    |
|                            |                                                      |
|             +------------------------------+                                      |
|             | Processador SAR & Camera      |                                      |
|             +------------------------------+                                      |
|                            |                                                      |
|               Serial USB (ASCII @ 115200)                                         |
+-----------------------------------------------------------------------------------+
                                    |
+-----------------------------------------------------------------------------------+
|                        RASPBERRY PI PICO (Firmware C++)                           |
|  - Controlo a 50 Hz | BNO055 (IMU) | Servos ST3215 | Aquecedores GPIO 4/5          |
+-----------------------------------------------------------------------------------+
```

---

## 2. Requisitos

### 2.1. Funcionais (RF)

- **RF01 - Apontamento Continuo:** calcular a posicao dos servos a 50 Hz para
  compensar a rotacao da gondola.
- **RF02 - Rececao de Comandos (Protobuf):** a GS envia novos alvos e ordens
  codificados em Protocol Buffers via ZeroMQ.
- **RF03 - Transmissao de Telemetria:** a OBC emite `TelemetryFrame` a 5 Hz com
  orientacao, servos, GNSS, aquecedores e saude do sistema.
- **RF04 - Leitura de Posicao (BNO055):** a Pico le a bussola/IMU por I2C.
- **RF05 - Controlo de Servos ST3215:** dois servos acoplados a gearboxes 5:1.
- **RF06 - Aquecimento:** ligar/desligar dois aquecedores (GPIO 4 e 5).
- **RF07 - Ingestao GNSS:** a OBC le as coordenadas de receptores GNSS em portas
  UDP locais e integra-as na telemetria.
- **RF08 - SAR (Ettus B210):** disparar e gerir a aquisicao e guardar no SSD.
- **RF09 - Download de Ficheiros:** a GS transfere ficheiros do SSD via HTTP.

### 2.2. Nao-funcionais (RNF)

- **RNF01 - Tempo Real:** o loop de controlo na Pico corre a 50 Hz (20 ms).
- **RNF02 - Limitacao Dinamica de Banda:** via `tc`.
- **RNF03 - Autonomia do Controlo:** se a ligacao OBC -> Pico falhar, a Pico
  continua em modo autonomo na ultima direccao valida.
- **RNF04 - Eficiencia de Rede:** mensagens binarias compactas via Protobuf v3.

---

## 3. Hardware

```
+-----------------------------------------------------------------------------------+
|                             OBC (Raspberry Pi 4B)                                 |
|  - SSD USB (Armazenamento de dados, logs e imagens)                               |
|  - SDR Ettus B210 USB (Modulo SAR)                                                |
|  - Camera USB (Fotos das antenas)                                                 |
|  - Receptores GNSS USB (Read_uB, UDP local 2000-2004)                             |
+-----------------------------------------------------------------------------------+

Cabo USB (Serial ASCII @ 115200)

+-----------------------------------------------------------------------------------+
|                           RASPBERRY PI PICO (Firmware C++)                        |
|  - BNO055 (IMU/Bussola via I2C nos pinos SDA 16 / SCL 17)                         |
|  - 2x Servos ST3215 (UART Half-Duplex via GPIO 0 e 1 + Adaptacao)                |
|  - 2x Modulos de Aquecimento (GPIO 4 e GPIO 5)                                   |
+-----------------------------------------------------------------------------------+
```

---

## 4. Comunicacoes

### 4.1. GS <-> OBC

| Direccao | Transporte | Porta | Conteudo |
|---|---|---|---|
| Comandos | ZMQ ROUTER/DEALER | 5555 | `CommandRequest` / `CommandResponse` |
| Telemetria | ZMQ PUB/SUB | 5556 | `TelemetryFrame` a 5 Hz |
| Ficheiros | HTTP | 8080 | listagem e download do SSD |

Comandos: `[b"", <CommandRequest>]`. Telemetria: `[b"TELEMETRY", <frame>]`.

O primeiro frame e o topico, para que a GS possa subscrever so o que lhe
interessa. Em ZMQ, `SUBSCRIBE` filtra por **prefixo de frame**; como o topico e
um frame completo e a carga util vem a seguir, o filtro e exacto.

### 4.2. OBC <-> Pico

Serial ASCII a 115200, uma linha por mensagem, terminada em `\n`.

| Para a Pico | Resposta |
|---|---|
| `TARGET <graus>` | `OK` |
| `JOG <id> <tick>` | `OK` |
| `HEAT1 <0\|1>` / `HEAT2 <0\|1>` | `OK` |
| `STATUS` | `TELEM,...` |
| `PING` | `PONG` |

`JOG` e um acrescento nosso a `CMD_JOG_SERVO`: move um servo para um `tick` em
modo manual, em ticks absolutos validos de `0..4095`. Sem ele, o RF de comando
manual de servo nao tinha caminho.

O `TELEM` v1 (posicao e temperatura, 9 campos) foi mantido por compatibilidade.
O **TELEM v2** acrescenta os campos que a GS precisa: `IMU`, e por servo
`id,tick,speed,load,voltage,temp,online`. O parser aceita os dois.

```
TELEM,<heading>,<target>,<imu_ok>,<n>,(id,tick,speed,load,voltage,temp,online)*n
```

O `online` por servo e o que permite ao operador distinguir "servo parado" de
"servo desligado" -- sem isso, um servo que caiu do bus aparece como um servo
saudavel a reportar zeros.

### 4.3. GNSS

Duas fontes, ambas em **modo escuta** (`bind` + `recvfrom`), nunca `connect`:
e o OBC que e servido, nao o cliente. Um socket UDP ligado so recebe de quem
ja estava a falar para ele -- e o erro classico que faz um feed "parecer morto".

- **JSON** em `127.0.0.1:9000`, para depuracao e receptores sem o protocolo
  Read_uB.
- **Read_uB** em `127.0.0.1:2000` (primaria; `2001-2004` como redundancia).
  `NavData` tem 420 bytes, `#pragma pack(1)`, little-endian:

  | Campo | Offset | Tipo |
  |---|---|---|
  | `Latitude` | 24 | `double` |
  | `Longitude` | 32 | `double` |
  | `Altitude` | 40 | `float` |
  | `flags` | 120 | `uint16` |
  | `stage` | 122 | `uint16` |

  `fix_ok = flags & 0x0001`.

O registo no Read_uB e um datagrama de 2 bytes little-endian
(`struct.pack("<H", 125)`), **renovado periodicamente**: o servidor poda
registos expirados, e um registo unico que nao se renova deixa de receber
silenciosamente. A renovacao e separada da recepcao, e um timeout de leitura
**nao** significa registo perdido -- tratar isso como tal fazia a fonte
desregistar-se sozinha e parar para sempre.

### 4.4. Banda

Filtro TBF em `eth0` via `tc`:

```
sudo tc qdisc del  dev eth0 root
sudo tc qdisc add dev eth0 root tbf rate 512kbit burst 32kbit latency 400ms
```

---

## 5. Decisoes de design

| Componente | Escolha | Justificacao |
| :--- | :--- | :--- |
| **Pico** | C++ (Arduino / RP2040) | Determinismo no loop a 50 Hz. |
| **OBC** | Python 3 | Integracao nativa com ZMQ, SDR, threads e comandos Linux. |
| **Rede** | ZeroMQ (ROUTER/DEALER, PUB/SUB) | Baixa latencia, assincrono, sem broker. |
| **Serializacao** | Protobuf v3 | Tamanho compacto na ligacao aerea. |
| **Banda** | Linux `tc` | Limites em tempo real, ja no kernel. |
| **Ficheiros** | `http.server` | Sem dependencias, integra-se em qualquer GS. |

### 5.1. Por que Protobuf e nao JSON

A ligacao aerea e cara (RF02/RNF04). `TelemetryFrame` a 5 Hz com dois servos e
GNSS tem ~40 bytes em Protobuf contra ~200 em JSON -- a 5 Hz a diferenca e
marginal, mas o `NavData` binario de 420 bytes *nao* tem equivalente JSON
razoavel, e RF07 exige os receptores reais. Protobuf nos dois lados da ligacao
serial e de rede, para que a GS e a OBC partilhem um unico ficheiro de contrato
(`proto/rocsar_messages.proto`) em vez de duas convencoes que divergem.

### 5.2. Por que ROUTER e nao REP

`REP` exige alternancia estrita e state; `ROUTER` entrega a origem em cada
frame, o que permite responder **a quem perguntou** mesmo com varios clientes
ligados. Nao ha lock-in a um unico operador.

### 5.3. Politica de mock: nada e simulado sem flag

O default e hardware real. Simular por omissao e o modo mais facil de fazer
voar um plataforma com telemetria fabricada, e `--auto-mock` existe como
extensao **opt-in**, fora da letra da especificacao, precisamente por isso.

A alternativa -- cair para mock quando o hardware falta -- e tentadora em
desenvolvimento e perigosa em voo. Por isso existem tres modos distintos:

- **real** (default);
- **simulado**, so por flag explicita, com avisos grandes no log;
- **degradado**, quando nao ha mock e o hardware falta: `online=False`,
  `fix_ok=False`, mas o servidor arranca.

O modo degradado e uma escolha, nao uma concessao: uma OBC que nao arranca nao
telemetrizada e pior do que uma OBC que telemetriza a dizer que esta avariada.
`--require-hardware` inverte a decisao e faz o servidor falhar em vez de
degradar.

### 5.4. Por que `tc` nunca levanta excecao

`tc` falha por dezenas de razoes nao relacionadas com a logica -- `sudo` sem
palavra-passe, `tc` ausente, interface errada, kernel sem suporte. Nenhuma
justifica derrubar o servidor de telemetria. O `BandwidthManager` devolve
sempre `(ok, mensagem)`, e o comando traduz para
`CommandResponse(success=False)` com o erro real.

Duas subtilezas que custaram bugs:

- `tc qdisc del` sem regra devolve erro -- e **sucesso** para o OBC. A deteccao
  tem de ser ancorada no texto exacto (`RTNETLINK answers: No such file or
  directory`, `Cannot delete qdisc`): a mensagem de "executavel nao encontrado"
  contem a mesma substring e seria lida como "ja nao havia regra", reportando
  sucesso quando `tc` nao existe sequer.
- `sudo` a pedir palavra-passe bloqueia. Daqui o timeout de 5 s por comando: um
  `sudo` pendurado seria um OBC que deixa de responder.

### 5.5. Por que um dispatcher para o SAR

`CMD_TRIGGER_SAR` tem de apontar para *aquisicao real da missao*, que nao e
conhecida a tempo de scripting. A cadeia e:

```
OBC --CMD_TRIGGER_SAR--> sar_runner --(delegacao)--> ./sdr-ettus-b200mini/run.sh
```

O dispatcher existe para que a missao substitua um comando sem tocar no OBC. E
tem de recusar configuracoes que apontem para ele proprio -- seria recursao
infinita, que nao se manifesta num erro mas num `fork`` sem limite a encher
disco e memoria.

### 5.6. Por que o GNSS valida o dado

`NavData` nao tem magic nem versao: o unico sinal de formato valido e o
**tamanho exacto** de 420 bytes. E ainda assim, um payload do tamanho certo
pode estar corrompido ou em endianness errado, produzindo coordenadas absurdas
mas "validas" para o tipo.

Publicar `(lat=90.0, lon=1e300)` e pior do que nao publicar nada: a GS nao tem
como distinguir de um erro real. Por isso o parser tambem rejeita valores nao
finitos e coordenadas fora de range. Um fix que se considera perdido apos
`--gnss-stale-s` sem actualizacao e marcado `fix_ok=False`, mesmo que o ultimo
valor recebido fosse valido -- um fix antigo e pior do que nenhum.

---

## 6. Tolerancia a falhas

| Falha | Comportamento |
|---|---|
| **Rede GS <-> OBC** | a OBC continua a gravar no SSD de forma autonoma |
| **OBC -> Pico** | a Pico mantem o loop autonomo na ultima direccao valida (RNF03) |
| **Pico ausente** | modo degradado, `online=False`, servidor arrancado |
| **GNSS mudo** | `fix_ok=False` apos `--gnss-stale-s`; o resto da telemetria continua |
| **`tc` indisponivel** | `CommandResponse(success=False)` com a razao; sem impacto na telemetria |
| **Sem permissao no SSD** | fallback para `./data` com aviso explicito no log |
| **Processo SAR a correr** | novo disparo recusado, com o pid actual na mensagem |

Em todos os casos o principio e o mesmo: **degradar e avisar, nunca cair**. A
unica excepcao deliberada e `--require-hardware`, que existe para quem prefere
falhar cedo.

### 6.1. Slow-joiner do PUB/SUB

Quem subscreve o `PUB` recebe em fila as mensagens publicadas antes de a
subscricao propagar pela rede. A consequencia pratica: os primeiros frames
chegam ao mesmo tempo, e medir a taxa sem os descartar da valores errados --
tipicamente 10x a taxa real, seguida de um silencio.

`tests/test_client_sim.py` descarta `SLOW_JOINER_FRAMES` frames antes de medir,
e mede jitter. Este e um protocolo de medicao, nao do software: se a GS deixar de
o fazer, vai reportar picos de telemetria que nao existem.

---

## 7. O que nao esta coberto por testes

Sao limites do ambiente, e vale a pena saber o que **nao** foi verificado:

- Pico de hardware real (serial, servos, IMU, aquecedores).
- Receptores Read_uB reais: registo UDP e renovacao.
- `sudo tc` sem palavra-passe configurada.
- `/mnt/ssd` montado num Pi com o SSD real.

Os testes cobrem a logica, o framing e o parsing com injectando doubles e
servidores falsos; a integracao fisica assume-se correcta ate prova em contrario
em voo.