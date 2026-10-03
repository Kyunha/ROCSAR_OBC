# ROCSAR On-Board Computer

Servidor do **On-Board Computer** (Raspberry Pi 4B) da plataforma ROCSAR.

O OBC fica no planador e e o unico processo que fala com o hardware: a Pico
(servos e IMU), o receptor GNSS, a interface de rede e o disco de dados. A
Ground Station nunca toca nisto directamente -- fala com o OBC por ZMQ e HTTP.

```
   Ground Station                Raspberry Pi 4B (OBC)
  +---------------+              +--------------------------+
  |  comandos     |<-- ZMQ ----->|  ROUTER  :5555           |
  |  telemetria   |<-- ZMQ ----->|  PUB    :5556            |
  |  ficheiros    |<-- HTTP ---->|  HTTP   :8080            |
  +---------------+              |                          |
                                 |  serial --> Pico         |
                                 |  UDP    --> GNSS         |
                                 |  tc     --> eth0         |
                                 +--------------------------+
```

## Sumario

- [Arranque](#arranque)
- [Modos: real, simulado e degradado](#modos-real-simulado-e-degradado)
- [Contratos](#contratos)
- [Comandos](#comandos)
- [GNSS](#gnss)
- [Largura de banda](#largura-de-banda)
- [Dados](#dados)
- [Desenvolvimento](#desenvolvimento)
- [Testes](#testes)
- [Referencia de flags](#referencia-de-flags)

## Arranque

```bash
python3 -m src.obc_server
```

Ou, pelo script, que cria o directório de dados e avisa quando ha mocks activos:

```bash
scripts/run_server.sh
```

Para desligar o OBC: `SIGINT` ou `SIGTERM`. O servidor drena as threads
telemetria, HTTP e GNSS antes de sair.

## Modos: real, simulado e degradado

O principio e simples: **nada e simulado sem que se peça**. O OBC opera com
hardware real por omissao.

| Modo | Como se chega | O que muda |
|---|---|---|
| **Real** | omissao | hardware real; se faltar, degradado |
| **Simulado** | `--mock`, `--mock-pico`, `--mock-sar`, `--mock-gnss` | telemetria **artificial**, avisos grandes no log |
| **Degradado** | hardware ausente sem mock | `online=False`, `fix_ok=False`, mas o servidor arranca e responde |

O modo degradado existe porque uma OBC que nao arranca nao telemetrizada e pior
que uma OBC que telemetriza a dizer que esta avariada. O comando responde com
`success=False` e a razao real.

Para nunca haver telemetria fabricada, use `--require-hardware`: proibe o
fallback para mock e, sem hardware, o servidor falha em vez de inventar dados.

> `--auto-mock` e uma **extensao nossa**, deliberadamente opt-in. Cai para mock
> quando o hardware falta, o que viola a letra da especificacao. Desligue-o em
> voo.

## Contratos

| Canal | Endereco | Transporte |
|---|---|---|
| Comandos (GS -> OBC) | `tcp://0.0.0.0:5555` | ZMQ ROUTER, multipart |
| Telemetria (OBC -> GS) | `tcp://0.0.0.0:5556` | ZMQ PUB, multipart, topico `TELEMETRY` |
| Ficheiros | `http://0.0.0.0:8080/` | HTTP |
| Pico | `/dev/ttyACM0` @ 115200 | ASCII, `\n` |
| GNSS JSON | `127.0.0.1:9000` | UDP, o OBC **escuta** |
| GNSS Read_uB | `127.0.0.1:2000-2004` | UDP, `NavData` binario de 420 bytes |
| Banda | `eth0` | `tc` via `sudo` |

Os payloads de comandosao **Protobuf**, gerados a partir de
`proto/rocsar_messages.proto`:

```bash
scripts/build_proto.sh     # regenera src/rocsar_messages_pb2.py
```

### Framing

- **Comandos**: `[b"", <CommandRequest>]`. O `ROUTER` preenche a identidade de
  origem; a GS envia um frame vazio como envelope.
- **Telemetria**: `[b"TELEMETRY", <TelemetryFrame>]`. O topico e o primeiro
  frame, para que a GS possa subscrever so o que lhe interessa.

> **Slow-joiner**: quem subscreve o `PUB` recebe em fila as mensagens publicadas
> antes de a subscricao propagar. Os primeiros frames sao, por isso,ao mesmo
> tempo. Descarta-os antes de medir taxa -- ver `SLOW_JOINER_FRAMES` em
> `tests/test_client_sim.py`.

## Comandos

Seis comandos, todos com resposta `CommandResponse`:

| Comando | Efeito |
|---|---|
| `CMD_SET_TARGET` | alvo de orientacao da gondola |
| `CMD_SET_BANDWIDTH` | limita a banda com `tc` (ver abaixo) |
| `CMD_CONTROL_HEATER` | liga/desliga um aquecedor |
| `CMD_JOG_SERVO` | move um servo para um tick, em modo manual |
| `CMD_TRIGGER_CAMERA` | `count` fotografias com `spacing_ms` entre elas |
| `CMD_TRIGGER_SAR` | dispara a aquisicao SAR |

`CMD_JOG_SERVO` exige que o firmware aceite `JOG <id> <tick>`, com `tick` valido
em `0..4095`.

### SAR

O comando SAR por omissao e `python3 -m sar_runner`, um **dispatcher** que delega
na aquisicao real (`./sdr-ettus-b200mini/run.sh` por omissao):

```bash
python3 -m sar_runner --dry-run              # mostra o que seria executado
python3 -m sar_runner --sar-command './meu-sdr/run.sh' --data-dir /tmp
```

O dispatcher recusa configuracoes que apontem para ele proprio -- isso seria recursao
infinita, e um erro de configuracao que nao se manifesta ate encher o disco.

## GNSS

Duas fontes, ambas em modo escuta:

1. **Feed JSON** em `127.0.0.1:9000` -- texto, para depuracao e para receptores
   que nao falem o protocolo Read_uB.
2. **Read_uB binario** em `127.0.0.1:2000` (primaria; `2001-2004` como
   redundancia). `NavData` tem 420 bytes, `#pragma pack(1)`, little-endian, sem
   magic nem versao:

   | Campo | Offset | Tipo |
   |---|---|---|
   | `Latitude` | 24 | `double` |
   | `Longitude` | 32 | `double` |
   | `Altitude` | 40 | `float` |
   | `flags` | 120 | `uint16` |
   | `stage` | 122 | `uint16` |

   `fix_ok = flags & 0x0001`.

   Como o formato nao tem versao, o **tamanho exacto** e o unico criterio de
   versao: datagramas que nao tenham 420 bytes sao descartados. Alem disso
   rejeitamos valores nao finitos e coordenadas fora de range, em vez de publicar
   posicoes inventadas.

O registo no Read_uB e um datagrama de 2 bytes little-endian
(`struct.pack("<H", 125)`), renovado periodicamente. Sem esse registo o receptor
nao envia nada.

Um fix com mais de `--gnss-stale-s` (5 s) sem actualizacao e dado como perdido
(`fix_ok=False`), mesmo que o ultimo valor recebido fosse valido.

## Largura de banda

`CMD_SET_BANDWIDTH` aplica um filtro TBF:

```bash
sudo tc qdisc del dev eth0 root
sudo tc qdisc add dev eth0 root tbf rate 512kbit burst 32kbit latency 400ms
```

`tc` precisa de root, e falhar **nunca** derruba o servidor de telemetria: o
`BandwidthManager` nao levanta excecao e devolve sempre `(ok, mensagem)`. O
chamador traduz isso para `CommandResponse(success=False)` com o erro real. A
ausencia de regra previa conta como sucesso -- `rate_kbps <= 0` remove o limite.

Configure `--sudo-prefix` (`sudo`, `doas tc`, ou vazio se ja corre como root).
Com `--no-tc` a gestao fica desligada.

## Dados

Por omissao `/mnt/ssd/rocsar_data`. Se nao for utilizavel -- SSD nao montado,
sem permissao -- o OBC cria `./data` e **avisa no log**. Em voo esse aviso e um
problema de montagem, nao algo a ignorar: confirme o que o OBC esta a usar no
arranque, na primeira linha do log.

```
/mnt/ssd/rocsar_data/
  sar-<timestamp>.bin     aquisicoes SAR
  cam-<timestamp>.png     fotografias
  cam-<timestamp>.log     logs dos processos
```

## Desenvolvimento

Ambiente Nix:

```bash
nix-shell          # python, pytest, ruff, mypy, iproute2
```

Instalacao manual:

```bash
pip install -e '.[dev]'
```

Verificacao:

```bash
pytest -q          # testes
ruff check .       # lint
mypy src           # tipos
```

## Testes

```bash
pytest -q                              # suite completa
pytest tests/test_serial_pico.py -q    # um modulo
```

Os testes de integracao (`tests/test_client_sim.py`) correm contra um OBC vivo e
**saltam automaticamente** se nao houver ninguem a escutar. Para os correr de
verdade:

```bash
# terminal 1
ROCSAR_MOCK=1 python3 -m src.obc_server

# terminal 2 -- simulador da Ground Station
python3 tests/test_client_sim.py --duration 6
```

O simulador tambem se usa sozinho, para diagnosticar uma GS real:

```bash
python3 tests/test_client_sim.py --host 192.168.1.20
python3 tests/test_client_sim.py --no-commands --no-http --duration 10
```

Ele mede a taxa de telemetria (descartando os frames de slow-joiner), envia os
seis comandos, confirma que os seus efeitos aparecem na telemetria seguinte, e
lista o directorio HTTP.

O que **nao** se pode testar aqui: o pico de hardware real, um Read_uB real,
`sudo tc` sem palavra-passe, e o `/mnt/ssd` montado.

## Referencia de flags

| Flag | Default | Efeito |
|---|---|---|
| `--cmd-bind` | `tcp://0.0.0.0:5555` | socket ROUTER de comandos |
| `--pub-bind` | `tcp://0.0.0.0:5556` | socket PUB de telemetria |
| `--http-host` / `--http-port` | `0.0.0.0` / `8080` | servidor de ficheiros |
| `--telemetry-hz` | `5.0` | frequencia de telemetria |
| `--serial-port` / `--serial-baud` | `/dev/ttyACM0` / `115200` | Pico |
| `--serial-poll-hz` | `10.0` | interogacao `STATUS` |
| `--mock` | desligado | todos os mocks de uma vez |
| `--mock-pico` / `--mock-sar` / `--mock-gnss` | desligado | mock individual |
| `--auto-mock` | desligado | cair para mock se faltar hardware |
| `--require-hardware` | desligado | proibir qualquer mock |
| `--gnss-json-port` | `9000` | porta do feed JSON |
| `--no-gnss-json` / `--no-readub` | desligado | desligar uma fonte GNSS |
| `--readub-ports` | `2000-2004` | portas Read_uB |
| `--readub-primary-port` | `2000` | porta que alimenta a telemetria |
| `--gnss-stale-s` | `5.0` | segundos sem fix antes de o dar por perdido |
| `--no-tc` | desligado | desligar a gestao de banda |
| `--tc-iface` | `eth0` | interface a limitar |
| `--sudo-prefix` | `sudo` | como obter privilegi |
| `--tc-burst` / `--tc-latency` | `32kbit` / `400ms` | parametros do bucket |
| `--data-dir` | `/mnt/ssd/rocsar_data` | directório de dados |
| `--sar-command` | `python3 -m sar_runner` | aquisicao SAR |
| `--camera-command` | `ffmpeg ...` | fotografia |
| `--log-level` | `INFO` | `DEBUG`/`INFO`/`WARNING`/`ERROR` |

Qualquer flag pode ser dada por ambiente com o prefixo `ROCSAR_`, maiusculas e
`_` no lugar de `-`: `--telemetry-hz` -> `ROCSAR_TELEMETRY_HZ=10`.

Ver `python3 -m src.obc_server --help` para a lista completa.

## Estrutura

```
proto/rocsar_messages.proto   contrato Protobuf
src/config.py                 flags, ambiente, resolucao do data-dir
src/system_health.py          CPU, disco, uptime, limite de banda
src/serial_pico.py            TELEM v1/v2, reconexao, mock, degradado
src/gnss_listener.py          JSON e NavData Read_uB
src/bandwidth_manager.py      tc, sempre fail-soft
src/command_runner.py         processos SAR/camara de longa duracao
src/sar_runner.py             dispatcher da aquisicao SAR
src/obc_server.py             ROUTER, PUB, HTTP, dispatch
scripts/build_proto.sh        regenera os stubs
scripts/run_server.sh         arranque
pico_firmware/                firmware da Pico
ARCHITECTURE.md               decisoes de arquitectura
```

Ver `ARCHITECTURE.md` para o *porque* das decisoes.