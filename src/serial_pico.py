"""Interface de comunicacao com a Raspberry Pi Pico (Serial USB, ASCII @ 115200).

Protocolo
---------
Comandos OBC -> Pico (uma linha ASCII terminada em ``\\n``)::

    TARGET <graus>     define o alvo de orientacao e sai do modo manual
    HEAT1 <0|1>        aquecedor 1 (GPIO 4)
    HEAT2 <0|1>        aquecedor 2 (GPIO 5)
    JOG <id> <tick>    move o servo <id> para <tick> e entra em modo manual
    STATUS             pede uma linha de telemetria

Respostas Pico -> OBC::

    OK: TARGET_SET / OK: HEAT1 / OK: HEAT2 / OK: JOG
    ERR: <motivo>
    TELEM,<heading>,<target>,<imu_ok>,<n>,(<id>,<tick>,<spd>,<load>,<v>,<t>,<online>)x n

O formato ``TELEM`` foi alargado para cobrir os dois servos completos. Para
compatibilidade com firmwares antigos tambem se aceita o formato antigo de 8
campos (``TELEM,h,t,imu,tick1,v1,t1,v2,t2``), onde so o servo 1 tem tick
conhecido.

Modos
-----
* **Real** (omissao): abre ``/dev/ttyACM0``.
* **Degradado**: sem device e sem ``--mock-pico``, publica ``online=False`` e
  ``imu_online=False``. Nunca inventa leituras.
* **Mock** (apenas com ``--mock-pico``): gera heading e telemetria plausiveis.
"""

from __future__ import annotations

import contextlib
import dataclasses
import logging
import math
import os
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from . import rocsar_messages_pb2 as pb

LOG = logging.getLogger(__name__)

#: Numero de servos da gandola (duas antenas).
NUM_SERVOS = 2

#: Ticks por volta completa do encoder de 12 bits.
TICKS_PER_TURN = 4096

#: Limites fisicos do servo ST3215.
MIN_TICK = 0
MAX_TICK = TICKS_PER_TURN - 1

#: Numero de campos por servo no formato TELEM v2: id,tick,spd,load,v,t,online
_FIELDS_PER_SERVO_V2 = 7

#: Cabecalho do TELEM v2: heading,target,imu_ok,n_servos
_FIELDS_HEADER_V2 = 4

#: Total de tokens (incluindo "TELEM") do TELEM v1 legado:
#: TELEM,h,t,imu,tick1,v1,t1,v2,t2  ->  1 + 8 = 9
_V1_TOKENS = 9

_DEFAULT_TIMEOUT_S = 0.2


class PicoError(RuntimeError):
    """Erro de comunicacao com a Pico (transmitido a CommandResponse)."""


@dataclass(frozen=True)
class ServoStatus:
    """Telemetria de um servo, ja no formato do proto."""

    id: int
    current_tick: int
    current_speed: int
    current_load: int
    voltage_v: float
    temperature_c: int
    online: bool

    def to_proto(self) -> pb.ServoData:
        s = pb.ServoData()
        s.id = self.id
        s.current_tick = self.current_tick
        s.current_speed = self.current_speed
        s.current_load = self.current_load
        s.voltage_v = self.voltage_v
        s.temperature_c = self.temperature_c
        s.online = self.online
        return s


@dataclass(frozen=True)
class TelemetrySnapshot:
    """Estado completo da gandola num instante.

    Imutavel de proposito: os leitores (thread de telemetria) podem segura-lo
    sem risco de ver campos a meio de uma actualizacao.
    """

    timestamp: float
    gondola_heading: float
    target_heading: float
    imu_online: bool
    servos: tuple[ServoStatus, ...]
    heater1_active: bool
    heater2_active: bool
    link_up: bool
    servo_count: int = field(default=NUM_SERVOS)

    @classmethod
    def offline(cls, *, target_heading: float = 0.0,
                heater1: bool = False, heater2: bool = False) -> TelemetrySnapshot:
        """Snapshot de ligacao em baixo: tudo desconhecido, nada inventado."""
        return cls(
            timestamp=time.time(),
            gondola_heading=0.0,
            target_heading=target_heading,
            imu_online=False,
            servos=tuple(
                ServoStatus(id=i + 1, current_tick=0, current_speed=0, current_load=0,
                            voltage_v=0.0, temperature_c=0, online=False)
                for i in range(NUM_SERVOS)
            ),
            heater1_active=heater1,
            heater2_active=heater2,
            link_up=False,
        )


# ---------------------------------------------------------------------------
# Parsing do TELEM
# ---------------------------------------------------------------------------
def _servo_offline(servo_id: int) -> ServoStatus:
    return ServoStatus(
        id=servo_id,
        current_tick=0,
        current_speed=0,
        current_load=0,
        voltage_v=0.0,
        temperature_c=0,
        online=False,
    )


def _parse_servo_fields(fields: list[str], servo_id: int) -> ServoStatus:
    """Converte os 7 campos de um servo (id,tick,spd,load,v,t,online)."""
    try:
        return ServoStatus(
            id=int(fields[0]),
            current_tick=int(fields[1]),
            current_speed=int(fields[2]),
            current_load=int(fields[3]),
            voltage_v=float(fields[4]),
            temperature_c=int(fields[5]),
            online=bool(int(fields[6])),
        )
    except (ValueError, IndexError) as exc:
        raise PicoError(f"campos de servo invalidos: {fields!r} ({exc})") from exc


def parse_telem(line: str) -> tuple[float, float, bool, tuple[ServoStatus, ...]] | None:
    """Faz o parse de uma linha ``TELEM``.

    Devolve ``(heading, target, imu_online, servos)`` ou ``None`` se a linha nao
    for um ``TELEM`` reconhecivel. Suporta os dois formatos de wire:

    * **v2** -- ``TELEM,h,t,imu,n,`` + ``n`` x 7 campos.
    * **v1** -- ``TELEM,h,t,imu,tick1,v1,t1,v2,t2`` (8 campos).

    O servo 2 no formato v1 fica com ``current_tick`` desconhecido (0) e
    ``online=False``, porque a Pico antiga nao o transmitia.
    """
    linha = line.strip()
    if not linha.upper().startswith("TELEM,"):
        return None

    campos = linha.split(",")
    if len(campos) < 2 or campos[0].strip().upper() != "TELEM":
        return None

    try:
        heading = float(campos[1])
        target = float(campos[2])
        imu_ok = bool(int(campos[3]))
    except (ValueError, IndexError):
        return None

    # --- Formato v2: numero de servos declarado no campo 4 ------------------
    if len(campos) > _FIELDS_HEADER_V2:
        try:
            n = int(campos[4])
        except ValueError:
            n = -1  # sinaliza "isto e um v1 com tick numerico"
        if 0 <= n <= NUM_SERVOS:
            esperado = _FIELDS_HEADER_V2 + 1 + _FIELDS_PER_SERVO_V2 * n
            if len(campos) == esperado:
                try:
                    servos = tuple(
                        _parse_servo_fields(
                            campos[_FIELDS_HEADER_V2 + 1 + i * _FIELDS_PER_SERVO_V2:
                                    _FIELDS_HEADER_V2 + 1 + (i + 1) * _FIELDS_PER_SERVO_V2],
                            i + 1,
                        )
                        for i in range(n)
                    )
                except PicoError:
                    return None
                return heading, target, imu_ok, servos
            # contagem declarada mas payload incompleto: linha corrompida

    # --- Formato v1 (legado) ------------------------------------------------
    if len(campos) == _V1_TOKENS:
        try:
            servo1 = ServoStatus(
                id=1,
                current_tick=int(campos[4]),
                current_speed=0,
                current_load=0,
                voltage_v=float(campos[5]),
                temperature_c=int(campos[6]),
                online=True,
            )
            # A Pico antiga so transmitia tensao/temperatura do servo 2.
            servo2 = ServoStatus(
                id=2,
                current_tick=0,
                current_speed=0,
                current_load=0,
                voltage_v=float(campos[7]),
                temperature_c=int(campos[8]),
                online=True,
            )
        except (ValueError, IndexError):
            return None
        return heading, target, imu_ok, (servo1, servo2)

    return None


# ---------------------------------------------------------------------------
# Fonte mock
# ---------------------------------------------------------------------------
class MockPicoSource:
    """Gera telemetria sintetica para desenvolvimento sem hardware.

    Nao substitui hardware em voo -- so e ativado com ``--mock-pico`` explicito.
    O heading varre lentamente (varrimento triangular) e os servos seguem a
    cinematica da gandola com alguma inercia e ruido, para que a telemetria
    seja uteis ao testar a GUI da GS.
    """

    def __init__(self) -> None:
        self._t0 = time.monotonic()
        self._target = 0.0
        self._heaters = {1: False, 2: False}
        self._ticks = {1: TICKS_PER_TURN // 2, 2: TICKS_PER_TURN // 2}
        self._lock = threading.Lock()

    def set_target(self, heading: float) -> None:
        with self._lock:
            self._target = heading

    def set_heater(self, heater_id: int, enable: bool) -> None:
        with self._lock:
            if heater_id in self._heaters:
                self._heaters[heater_id] = enable

    def jog(self, servo_id: int, tick: int) -> None:
        with self._lock:
            if servo_id in self._ticks:
                self._ticks[servo_id] = tick

    def poll(self) -> TelemetrySnapshot:
        agora = time.monotonic()
        with self._lock:
            target = self._target
            heaters = dict(self._heaters)
            manual = dict(self._ticks)

        t = agora - self._t0
        # Varrimento triangular +-60 graus em ~40 s, mais ruido de turbulencia.
        heading = 60.0 * math.sin(t * (2.0 * math.pi / 40.0)) + random.uniform(-1.5, 1.5)
        heading = heading % 360.0

        servos: list[ServoStatus] = []
        for servo_id in (1, 2):
            # Diferenca angular envolta nao normalizada para [-180, 180].
            delta = (target - heading + 180.0) % 360.0 - 180.0
            # Gearbox 5:1, tal como o firmware.
            ideal = TICKS_PER_TURN // 2 + int(
                delta * 5.0 * (TICKS_PER_TURN / 360.0) * (-1.0)
            )
            atual = manual.get(servo_id, TICKS_PER_TURN // 2)
            # Se o servo esta em modo manual (JOG), mantem o tick pedido.
            destino = atual if servo_id in manual else max(MIN_TICK, min(MAX_TICK, ideal))
            # ST3215: velocidade em RPM (~0-200), carga em 0.1% (0-1000).
            vel = max(-200, min(200, int(-delta * 0.6) + random.randint(-3, 3)))
            carga = max(0, min(1000, int(abs(delta) * 2.0) + random.randint(-5, 5)))
            servos.append(
                ServoStatus(
                    id=servo_id,
                    current_tick=destino,
                    current_speed=vel,
                    current_load=carga,
                    voltage_v=round(random.uniform(11.8, 12.4), 1),
                    temperature_c=random.randint(30, 40),
                    online=True,
                )
            )

        return TelemetrySnapshot(
            timestamp=agora,
            gondola_heading=heading,
            target_heading=target,
            imu_online=True,
            servos=tuple(servos),
            heater1_active=heaters.get(1, False),
            heater2_active=heaters.get(2, False),
            link_up=True,
        )


# ---------------------------------------------------------------------------
# Fonte real
# ---------------------------------------------------------------------------
class _Reconnect:
    """Backoff exponencial para reconexao do dispositivo USB.

    O numero do tty pode mudar quando o Pi reenumeriza o USB, por isso a
    procura aceita qualquer /dev/ttyACM*.
    """

    def __init__(self, *, base: float = 0.5, cap: float = 5.0) -> None:
        self._base = base
        self._cap = cap
        self._delay = base
        self._next_attempt = 0.0

    def ready(self) -> bool:
        return time.monotonic() >= self._next_attempt

    def fail(self) -> None:
        self._next_attempt = time.monotonic() + self._delay
        self._delay = min(self._delay * 2.0, self._cap)
        LOG.debug("Proxima tentativa de ligacao em %.1fs", self._delay)

    def succeed(self) -> None:
        self._delay = self._base
        self._next_attempt = 0.0


class PicoLink:
    """Ligacao serial real com a Pico.

    Envolve um ``serial.Serial`` quando disponivel. Todas as escritas passam por
    uma lock para que a thread de comandos nunca embaralhe com o poller de
    ``STATUS``.
    """

    def __init__(self, port: str, baud: int, *, timeout_s: float = _DEFAULT_TIMEOUT_S) -> None:
        self._port = port
        self._baud = baud
        self._timeout_s = timeout_s
        # `serial.Serial` e importado tardiamente; ate la, None.
        self._ser: Any = None
        self._write_lock = threading.Lock()
        self._reconnect = _Reconnect()
        self._rx_buffer = ""

    # -- ciclo de vida ----------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", True)

    def open(self) -> bool:
        """Tenta abrir a porta. Devolve True se ficou aberta."""
        if self.is_open:
            return True
        if not self._reconnect.ready():
            return False

        try:
            import serial  # import tardio: em modo mock nao e preciso
        except ImportError as exc:  # pragma: no cover - pyserial e dependencia
            raise PicoError(
                "pyserial nao instalado; instale com 'pip install pyserial'"
            ) from exc

        try:
            self._ser = serial.Serial(
                self._port,
                self._baud,
                timeout=self._timeout_s,
                write_timeout=self._timeout_s,
            )
        except (OSError, serial.SerialException) as exc:
            LOG.warning("Pico: nao foi possivel abrir %s (%s)", self._port, exc)
            self._reconnect.fail()
            return False

        LOG.info("Pico: ligada em %s @ %d baud", self._port, self._baud)
        self._reconnect.succeed()
        self._rx_buffer = ""
        return True

    def close(self) -> None:
        if self._ser is not None:
            with contextlib.suppress(Exception):  # melhor effort
                self._ser.close()
        self._ser = None

    # -- escrita ----------------------------------------------------------
    def write_line(self, line: str) -> None:
        """Escreve um comando. Lanza :class:`PicoError` em falha."""
        if not self.is_open:
            raise PicoError(f"Pico nao esta ligada ({self._port})")
        payload = (line.strip() + "\n").encode("ascii", errors="replace")
        with self._write_lock:
            try:
                assert self._ser is not None
                self._ser.write(payload)
                self._ser.flush()
            except Exception as exc:  # SerialException/OSError
                self.close()
                self._reconnect.fail()
                raise PicoError(f"escrita para a Pico falhou: {exc}") from exc

    def expect_ack(self, timeout_s: float | None = None) -> str | None:
        """Le a resposta a um comando. Devolve a primeira linha ou ``None``."""
        if not self.is_open:
            raise PicoError(f"Pico nao esta ligada ({self._port})")
        limite = time.monotonic() + (timeout_s if timeout_s is not None else self._timeout_s)
        while time.monotonic() < limite:
            linha = self._read_line()
            if linha is not None:
                return linha
        return None

    def _read_line(self) -> str | None:
        assert self._ser is not None
        try:
            bruto = self._ser.readline()
        except Exception as exc:  # SerialException/OSError
            self.close()
            self._reconnect.fail()
            raise PicoError(f"leitura da Pico falhou: {exc}") from exc
        if not bruto:
            return None
        linha = bruto.decode("ascii", errors="replace").strip()
        return linha or None

    # -- STATUS -----------------------------------------------------------
    def poll_status(self) -> TelemetrySnapshot | None:
        """Envia STATUS e devolve a telemetria, ou ``None`` se nao chegou."""
        self.write_line("STATUS")
        # A Pico responde com uma unica linha TELEM; damos uma janela curta
        # para a linha chegar.
        limite = time.monotonic() + self._timeout_s * 2
        while time.monotonic() < limite:
            linha = self._read_line()
            if linha is None:
                continue
            if linha.upper().startswith("ERR:"):
                LOG.warning("Pico respondeu ERR a STATUS: %s", linha)
                return None
            resultado = parse_telem(linha)
            if resultado is not None:
                heading, target, imu_ok, servos = resultado
                return TelemetrySnapshot(
                    timestamp=time.time(),
                    gondola_heading=heading,
                    target_heading=target,
                    imu_online=imu_ok,
                    servos=servos,
                    heater1_active=False,   # preenchido pelo estado do OBC
                    heater2_active=False,
                    link_up=True,
                    servo_count=len(servos),
                )
            LOG.debug("Pico: linha TELEM nao reconhecida: %r", linha)
        return None


# ---------------------------------------------------------------------------
# Fachada: escolhe mock / real / degradado
# ---------------------------------------------------------------------------
class PicoController:
    """Ponto de entrada unico para o servidor.

    Mantem o ultimo :class:`TelemetrySnapshot` conhecido e expoe os comandos de
    alto nivel. Uma thread interna faz o polling de ``STATUS`` a
    ``poll_hz``; os comandos modificam estado e sao aplicados na ligacao real
    (ou no mock).

    **Os comandos sao assincronos em relacao ao snapshot.** Um comando e
    confirmado pelo ACK da Pico imediatamente, mas o ``snapshot()`` so reflecte
    o novo estado no ciclo de poll seguinte (ate ``1/poll_hz`` segundos depois).
    Esta assimcronia e intencional: o snapshot tem de descrever o que o hardware
    *reportou*, nao o que foi pedido. Nao espere ver o efeito de um comando no
    snapshot sem aguardar um ciclo.
    """

    def __init__(
        self,
        *,
        port: str,
        baud: int,
        poll_hz: float = 10.0,
        mock: bool = False,
        auto_mock: bool = False,
        require_hardware: bool = False,
    ) -> None:
        self._port = port
        self._poll_period = 1.0 / poll_hz if poll_hz > 0 else 0.1
        self._require_hardware = require_hardware

        self._lock = threading.Lock()
        self._snapshot = TelemetrySnapshot.offline()
        self._heaters = {1: False, 2: False}
        self._target_heading = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # Decide o modo: mock so com flag explicita (ou --auto-mock se o
        # device nao existir).
        device_existe = _device_present(port)
        self.mock = mock
        if mock:
            LOG.warning("Pico: MODO MOCK ativo (--mock). Telemetria ARTIFICIAL.")
        elif not device_existe:
            if auto_mock:
                LOG.warning(
                    "Pico: %s ausente e --auto-mock ativo; a cair para MOCK.",
                    port,
                )
                self.mock = True
            elif require_hardware:
                raise PicoError(
                    f"Hardware exigido mas {port} nao existe. "
                    "Ligue a Pico ou remova --require-hardware."
                )
            else:
                LOG.warning(
                    "Pico: %s ausente; a arrancar DEGRADADO (online=False). "
                    "Use --mock-pico para telemetria artificial.",
                    port,
                )

        self._link = (
            None if self.mock
            else PicoLink(port, baud)
        )
        self._mock_source = MockPicoSource() if self.mock else None

    # -- ciclo de vida ----------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._poll_loop, name="pico-poll", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._link is not None:
            self._link.close()

    @property
    def link_up(self) -> bool:
        if self.mock:
            return True
        return self._link is not None and self._link.is_open

    # -- leitura ----------------------------------------------------------
    def snapshot(self) -> TelemetrySnapshot:
        """Ultimo estado conhecido (imutavel, seguro de ler sem lock)."""
        with self._lock:
            return self._snapshot

    # -- comandos ---------------------------------------------------------
    def set_target(self, heading: float) -> None:
        """Define o alvo de orientacao da gandola, em graus [0, 360)."""
        h = float(heading) % 360.0
        if self.mock:
            assert self._mock_source is not None
            self._mock_source.set_target(h)
        else:
            self._apply(f"TARGET {h:.2f}")
        with self._lock:
            self._target_heading = h

    def set_heater(self, heater_id: int, enable: bool) -> None:
        """Liga/desliga um aquecedor (1 ou 2)."""
        if heater_id not in (1, 2):
            raise PicoError(f"heater_id invalido: {heater_id} (esperado 1 ou 2)")
        if self.mock:
            assert self._mock_source is not None
            self._mock_source.set_heater(heater_id, enable)
        else:
            self._apply(f"HEAT{heater_id} {1 if enable else 0}")
        with self._lock:
            self._heaters[heater_id] = enable

    def jog_servo(self, servo_id: int, tick: int) -> None:
        """Move um servo para ``tick`` e entra em modo manual."""
        if servo_id not in (1, 2):
            raise PicoError(f"servo_id invalido: {servo_id} (esperado 1 ou 2)")
        if not (MIN_TICK <= tick <= MAX_TICK):
            raise PicoError(
                f"target_tick invalido: {tick} (intervalo {MIN_TICK}..{MAX_TICK})"
            )
        if self.mock:
            assert self._mock_source is not None
            self._mock_source.jog(servo_id, tick)
        else:
            self._apply(f"JOG {servo_id} {int(tick)}")

    # -- internos ---------------------------------------------------------
    def _apply(self, command: str) -> None:
        """Envia um comando e valida o ACK, reabrindo a ligacao se preciso."""
        if self._link is None:
            raise PicoError("Pico nao esta disponivel")
        if not self._link.is_open and not self._link.open():
            raise PicoError(f"nao foi possivel abrir {self._port}")
        self._link.write_line(command)
        ack = self._link.expect_ack()
        if ack is None:
            raise PicoError(f"sem resposta da Pico a '{command}'")
        if ack.upper().startswith("ERR"):
            raise PicoError(f"Pico rejeitou '{command}': {ack}")
        LOG.debug("Pico: '%s' -> %s", command, ack)

    def _poll_loop(self) -> None:
        """Faz polling de STATUS e renova o snapshot."""
        while not self._stop.is_set():
            try:
                snap: TelemetrySnapshot | None
                if self.mock:
                    assert self._mock_source is not None
                    snap = self._mock_source.poll()
                    self._store(snap)
                else:
                    assert self._link is not None
                    if not self._link.is_open:
                        self._link.open()
                    if self._link.is_open:
                        snap = self._link.poll_status()
                        if snap is not None:
                            self._store(snap)
            except PicoError as exc:
                LOG.warning("Pico: erro no polling: %s", exc)
            except Exception as exc:  # pragma: no cover - rede de seguranca
                LOG.exception("Pico: erro inesperado no polling: %s", exc)
            self._stop.wait(self._poll_period)

    def _store(self, snap: TelemetrySnapshot) -> None:
        with self._lock:
            self._snapshot = replace_snapshot_heaters(snap, self._heaters)


def replace_snapshot_heaters(snap: TelemetrySnapshot, heaters: dict[int, bool]) -> TelemetrySnapshot:
    """Devolve ``snap`` com o estado de aquecedores atualmente comandado.

    O formato TELEM da Pico nao transporta o estado dos aquecedores (a Pico
    antiga nem sequer tinha o comando de leitura), por isso a telemetria
    reporta o ultimo estado comandado pelo OBC.
    """
    return dataclasses.replace(
        snap, heater1_active=heaters.get(1, False), heater2_active=heaters.get(2, False)
    )


def _device_present(port: str) -> bool:
    """True se o dispositivo serie existe."""
    return os.path.exists(port)
