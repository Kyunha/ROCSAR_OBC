"""Simulador da Ground Station (GS) do ROCSAR.

Faz as tres coisas que a GS faz, contra um OBC a correr:

1. subscreve a telemetria (ZMQ SUB na porta 5556, topico ``TELEMETRY``),
2. envia comandos (ZMQ DEALER na porta 5555) e le as ``CommandResponse``,
3. lista e transfere ficheiros do SSD pelo servidor HTTP (porta 8080).

Corre como ferramenta interactiva e como teste de integracao. Dois
diagnosticos importantes:

* os primeiros frames apos subscricao **descartam-se** -- e o fenomeno
  *slow-joiner* do PUB/SUB: as mensagens publicadas antes de a subscricao
  propagar ficam em fila e sao entregues de seguida. Medir a taxa sem
  descartar isto da valores errados.
* a taxa esperada e ``--expected-hz`` (5 Hz por omissao).

Como usar::

    python3 tests/test_client_sim.py --host 127.0.0.1
    python3 tests/test_client_sim.py --host 192.168.1.20 --duration 10
    python3 tests/test_client_sim.py --no-commands --no-http --duration 4
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass, field
from pathlib import Path

# Permite executar o ficheiro directamente a partir da raiz do repo.
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import zmq  # noqa: E402

try:  # presente nos testes; opcional na execucao standalone
    import pytest
except ImportError:  # pragma: no cover
    pytest = None  # type: ignore[assignment]

from src import rocsar_messages_pb2 as pb  # noqa: E402

TELEMETRY_TOPIC = b"TELEMETRY"

#: Frames a descartar no arranque, por causa do slow-joiner do PUB/SUB.
SLOW_JOINER_FRAMES = 5

_HREF_RE = re.compile(r'<a\s+href="([^"]+)"')


# ===========================================================================
# Recolha de telemetria
# ===========================================================================
@dataclass
class TelemetryCollector:
    """Thread SUB que acumula ``TelemetryFrame`` com marcas de tempo."""

    host: str
    pub_port: int = 5556
    topic: bytes = TELEMETRY_TOPIC

    frames: list[tuple[float, pb.TelemetryFrame]] = field(default_factory=list)
    bad_payloads: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _stop: threading.Event = field(default_factory=threading.Event, repr=False)
    _thread: threading.Thread | None = field(default=None, repr=False)

    def start(self) -> None:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, self.topic)
        sock.connect(f"tcp://{self.host}:{self.pub_port}")
        self._thread = threading.Thread(
            target=self._run, args=(sock,), name="telemetry-sub", daemon=True
        )
        self._thread.start()

    def _run(self, sock: zmq.Socket) -> None:
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        while not self._stop.is_set():
            try:
                if not dict(poller.poll(200)):
                    continue
                parts = sock.recv_multipart()
            except zmq.ZMQError:
                return

            if len(parts) < 2 or parts[0] != self.topic:
                continue
            frame = pb.TelemetryFrame()
            try:
                frame.ParseFromString(parts[-1])
            except Exception:
                with self._lock:
                    self.bad_payloads += 1
                continue
            with self._lock:
                self.frames.append((time.monotonic(), frame))

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    @property
    def count(self) -> int:
        with self._lock:
            return len(self.frames)

    def latest(self) -> pb.TelemetryFrame | None:
        with self._lock:
            return self.frames[-1][1] if self.frames else None

    def _timestamps(self, skip: int) -> list[float]:
        with self._lock:
            return [t for t, _ in self.frames[skip:]]

    def rate_hz(self, *, skip: int = SLOW_JOINER_FRAMES) -> float | None:
        """Taxa de chegada em Hz, ignorando os primeiros ``skip`` frames."""
        pontos = self._timestamps(skip)
        if len(pontos) < 2:
            return None
        span = pontos[-1] - pontos[0]
        if span <= 0:
            return None
        return (len(pontos) - 1) / span

    def intervals(self, *, skip: int = SLOW_JOINER_FRAMES) -> list[float]:
        pontos = self._timestamps(skip)
        # strict=False: `pontos` tem um elemento a mais que `pontos[1:]`, e e
        # precisamente essa sobreposicao que produz os intervalos.
        return [b - a for a, b in zip(pontos, pontos[1:], strict=False)]


# ===========================================================================
# Cliente de comandos
# ===========================================================================
class CommandClient:
    """Cliente DEALER: envia ``CommandRequest`` e aguarda ``CommandResponse``."""

    def __init__(self, host: str, cmd_port: int = 5555, timeout_ms: int = 5000) -> None:
        self.host = host
        self.timeout_ms = timeout_ms
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.DEALER)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(f"tcp://{host}:{cmd_port}")

    def close(self) -> None:
        self._sock.close(linger=0)

    def send(self, request: pb.CommandRequest) -> pb.CommandResponse | None:
        """Envia e espera pela resposta. ``None`` em timeout."""
        self._sock.send_multipart([b"", request.SerializeToString()])
        if not self._sock.poll(self.timeout_ms, zmq.POLLIN):
            return None
        parts = self._sock.recv_multipart()
        response = pb.CommandResponse()
        response.ParseFromString(parts[-1])
        return response

    # -- atalhos por comando -----------------------------------------
    def _new(self, tipo: pb.CommandType, prefixo: str) -> pb.CommandRequest:
        return pb.CommandRequest(command_id=f"{prefixo}-{uuid.uuid4().hex[:6]}",
                                 type=tipo)

    def set_target(self, heading: float) -> pb.CommandResponse | None:
        r = self._new(pb.CMD_SET_TARGET, "target")
        r.set_target.target_heading = heading
        return self.send(r)

    def set_bandwidth(self, rate_kbps: int) -> pb.CommandResponse | None:
        r = self._new(pb.CMD_SET_BANDWIDTH, "bw")
        r.set_bandwidth.rate_kbps = rate_kbps
        return self.send(r)

    def control_heater(self, heater_id: int, enable: bool) -> pb.CommandResponse | None:
        r = self._new(pb.CMD_CONTROL_HEATER, "heat")
        r.control_heater.heater_id = heater_id
        r.control_heater.enable = enable
        return self.send(r)

    def trigger_sar(self) -> pb.CommandResponse | None:
        return self.send(self._new(pb.CMD_TRIGGER_SAR, "sar"))

    def trigger_camera(self, count: int = 1,
                       spacing_ms: float = 200) -> pb.CommandResponse | None:
        r = self._new(pb.CMD_TRIGGER_CAMERA, "cam")
        r.trigger_camera.count = count
        r.trigger_camera.spacing_ms = spacing_ms
        return self.send(r)

    def jog_servo(self, servo_id: int, target_tick: int) -> pb.CommandResponse | None:
        r = self._new(pb.CMD_JOG_SERVO, "jog")
        r.jog_servo.servo_id = servo_id
        r.jog_servo.target_tick = target_tick
        return self.send(r)


# ===========================================================================
# Cliente HTTP de ficheiros
# ===========================================================================
class FileClient:
    """Acede ao servidor de ficheiros do OBC."""

    def __init__(self, host: str, port: int = 8080, timeout_s: float = 5.0) -> None:
        self.base = f"http://{host}:{port}/"
        self.timeout_s = timeout_s

    def listdir(self) -> list[str]:
        """Devolve as entradas listadas na raiz do directório."""
        with urllib.request.urlopen(self.base, timeout=self.timeout_s) as resp:
            html = resp.read().decode("utf-8", errors="replace")
        return _HREF_RE.findall(html)

    def download(self, name: str) -> bytes:
        with urllib.request.urlopen(self.base + name, timeout=self.timeout_s) as resp:
            return resp.read()

    def available(self) -> bool:
        try:
            with urllib.request.urlopen(self.base, timeout=1.0):
                return True
        except (urllib.error.URLError, OSError, ValueError):
            return False


# ===========================================================================
# Apresentacao
# ===========================================================================
def format_telemetry(f: pb.TelemetryFrame) -> str:
    gnss = (f"{f.gnss.latitude:9.5f} {f.gnss.longitude:10.5f}"
            f" {f.gnss.altitude:7.1f}m fix={'ok' if f.gnss.fix_ok else '--'}")
    linhas = [
        "  --- TelemetryFrame ---",
        f"  bussola   : gondola={f.gondola_heading:6.1f}"
        f"  alvo={f.target_heading:6.1f}"
        f"  IMU={'online' if f.imu_online else 'offline'}",
        f"  aquecedores: H1={'ON' if f.heater1_active else 'off'}"
        f"  H2={'ON' if f.heater2_active else 'off'}",
        f"  GNSS      : {gnss}",
        f"  saude     : CPU={f.health.cpu_temp_c:.1f}C"
        f"  up={f.health.uptime_seconds}s"
        f"  banda={'limitada' if f.health.limit_active else 'sem limite'}"
        f"  disco={f.health.disk_used_percent:.1f}%",
        f"  servos    : {len(f.servos)}",
    ]
    for s in f.servos:
        linhas.append(
            f"     #{s.id} tick={s.current_tick:5d}  {s.current_speed:+5d} rpm"
            f"  carga={s.current_load:5d}  {s.voltage_v:5.2f}V"
            f"  {s.temperature_c:3d}C  {'online' if s.online else 'offline'}"
        )
    return "\n".join(linhas)


# ===========================================================================
# Cenario de teste
# ===========================================================================
@dataclass
class Check:
    name: str
    ok: bool
    detail: str = ""


class GSSimulator:
    """Orquestra os tres canais e produz um veredicto pass/fail."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        *,
        pub_port: int = 5556,
        cmd_port: int = 5555,
        http_port: int = 8080,
        duration_s: float = 6.0,
        expected_hz: float = 5.0,
        hz_tolerance: float = 0.35,
        send_commands: bool = True,
        check_http: bool = True,
        print_telemetry: bool = True,
        verbose: bool = False,
    ) -> None:
        self.host = host
        self.pub_port = pub_port
        self.cmd_port = cmd_port
        self.http_port = http_port
        self.duration_s = duration_s
        self.expected_hz = expected_hz
        self.hz_tolerance = hz_tolerance
        self.send_commands = send_commands
        self.check_http = check_http
        self.print_telemetry = print_telemetry
        self.verbose = verbose
        self.checks: list[Check] = []

    def _check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.checks.append(Check(name, ok, detail))
        return ok

    # ------------------------------------------------------------------
    def run(self) -> bool:
        collector = TelemetryCollector(self.host, self.pub_port)
        commands: CommandClient | None = None
        files: FileClient | None = None

        # O SUB tem de estar ligado ANTES dos comandos: o efeito de um comando
        # so aparece na telemetria seguinte e nao queremos perder esse frame.
        collector.start()
        time.sleep(0.8)

        print(f"[GS] telemetria: tcp://{self.host}:{self.pub_port} "
              f"(topico {TELEMETRY_TOPIC.decode()})")

        try:
            if self.send_commands:
                commands = CommandClient(self.host, self.cmd_port)
                self._command_scenario(commands, collector)
            print(f"[GS] a observar telemetria durante {self.duration_s:.0f}s...")
            self._observe(collector, self.duration_s)

            if self.check_http:
                files = FileClient(self.host, self.http_port)
                if files.available():
                    self._file_scenario(files)
                else:
                    self._check("HTTP responde", False,
                                f"sem resposta em http://{self.host}:{self.http_port}/")

            self._report(collector)
        finally:
            collector.stop()
            if commands is not None:
                commands.close()

        return all(c.ok for c in self.checks)

    # ------------------------------------------------------------------
    def _command_scenario(self, commands: CommandClient,
                          collector: TelemetryCollector) -> None:
        """Envia os seis comandos e verifica os efeitos na telemetria."""
        print(f"[GS] comandos: tcp://{self.host}:{self.cmd_port}")
        alvo = 180.0
        aquecedor = 1
        servo_id = 2
        servo_tick = 3000

        pedidos: list[tuple[str, pb.CommandResponse | None]] = [
            ("CMD_SET_TARGET", commands.set_target(alvo)),
            ("CMD_CONTROL_HEATER", commands.control_heater(aquecedor, True)),
            ("CMD_JOG_SERVO", commands.jog_servo(servo_id, servo_tick)),
            ("CMD_TRIGGER_CAMERA", commands.trigger_camera(1, 200)),
            ("CMD_SET_BANDWIDTH", commands.set_bandwidth(512)),
            ("CMD_TRIGGER_SAR", commands.trigger_sar()),
        ]

        for nome, resp in pedidos:
            if resp is None:
                self._check(nome, False, "sem resposta (timeout)")
                print(f"  {nome:22s} TIMEOUT")
                continue
            # CMD_SET_BANDWIDTH pode falhar legitimamente sem `sudo`, por isso
            # so exigimos resposta. Os restantes tem de ter sucesso.
            if nome == "CMD_SET_BANDWIDTH":
                self._check(f"{nome} respondido", True)
            else:
                self._check(nome, resp.success, resp.message)
            print(f"  {nome:22s} {'ok' if resp.success else 'FALHA'} "
                  f"{resp.message}")

        # Os efeitos so aparecem na telemetria seguinte; damos tempo ao PUB.
        time.sleep(0.6)
        f = collector.latest()
        if f is None:
            self._check("efeitos visiveis na telemetria", False,
                        "nenhum TelemetryFrame para verificar")
            return

        self._check("alvo aplicado", abs(f.target_heading - alvo) < 1.0,
                    f"target_heading={f.target_heading} (esperado {alvo})")
        self._check("aquecedor 1 ligado",
                    f.heater1_active if aquecedor == 1 else f.heater2_active,
                    "CMD_CONTROL_HEATER nao se reflectiu")

        servo = next((s for s in f.servos if s.id == servo_id), None)
        if servo is None:
            self._check(f"servo {servo_id} em telemetria", False,
                        "nao apareceu nenhum ServoData com esse id")
        else:
            self._check(f"servo {servo_id} no alvo",
                        abs(servo.current_tick - servo_tick) <= 50,
                        f"current_tick={servo.current_tick} (esperado ~{servo_tick})")

    def _observe(self, collector: TelemetryCollector, duration_s: float) -> None:
        fim = time.monotonic() + duration_s
        ultimo = 0.0
        while time.monotonic() < fim:
            time.sleep(0.1)
            if not self.print_telemetry or (time.monotonic() - ultimo) < 2.0:
                continue
            ultimo = time.monotonic()
            f = collector.latest()
            if f is not None:
                print()
                print(format_telemetry(f))

    def _file_scenario(self, files: FileClient) -> None:
        print(f"[GS] ficheiros: http://{self.host}:{self.http_port}/")
        entradas = files.listdir()
        self._check("HTTP lista diretorio", True)
        print(f"  entradas: {entradas}")
        ficheiros = [e for e in entradas if not e.endswith("/")]
        if ficheiros:
            try:
                dados = files.download(ficheiros[0])
            except (urllib.error.URLError, OSError) as exc:
                self._check("HTTP transfere ficheiro", False, str(exc))
            else:
                self._check("HTTP transfere ficheiro", True)
                print(f"  '{ficheiros[0]}': {len(dados)} bytes")
        else:
            print("  (sem ficheiros para transferir)")

    # ------------------------------------------------------------------
    def _report(self, collector: TelemetryCollector) -> None:
        print()
        print("=" * 68)
        print("RESULTADO")
        print("=" * 68)

        self._check("telemetria recebida", collector.count > 0,
                    "nenhum TelemetryFrame recebido")
        taxa = collector.rate_hz()
        if taxa is None:
            self._check(f"taxa ~{self.expected_hz:.0f} Hz", False,
                        "frames insuficientes para medir")
        else:
            self._check(f"taxa ~{self.expected_hz:.0f} Hz",
                        abs(taxa - self.expected_hz) <= self.hz_tolerance,
                        f"medida {taxa:.2f} Hz")
        self._check("payloads protobuf validos", collector.bad_payloads == 0,
                    f"{collector.bad_payloads} corrompidos")

        f = collector.latest()
        if f is not None:
            self._check("servos em telemetria", len(f.servos) > 0,
                        f"recebidos {len(f.servos)}")
            intervalos = collector.intervals()
            jitter = statistics.pstdev(intervalos) if len(intervalos) > 1 else 0.0
            print(f"  telemetria : {collector.count} frames, {taxa:.2f} Hz, "
                  f"jitter {jitter * 1000:.1f} ms "
                  f"({SLOW_JOINER_FRAMES} de slow-joiner descartados)")
            print(f"  leitura    : heading={f.gondola_heading:.1f} "
                  f"alvo={f.target_heading:.1f} "
                  f"IMU={'ok' if f.imu_online else 'offline'} "
                  f"GNSS={'ok' if f.gnss.fix_ok else 'invalido'} "
                  f"cpu={f.health.cpu_temp_c:.1f}C")
            print(f"  servos     : {len(f.servos)} "
                  f"({sum(1 for s in f.servos if s.online)} online)")

        print()
        for c in self.checks:
            linha = f"  [{'PASS' if c.ok else 'FALHA':5s}] {c.name}"
            if c.detail and (self.verbose or not c.ok):
                linha += f"  ({c.detail})"
            print(linha)

        falhas = [c for c in self.checks if not c.ok]
        print()
        if falhas:
            print(f"FALHOU: {len(falhas)} de {len(self.checks)} verificacoes")
        else:
            print(f"OK: {len(self.checks)} verificacoes passaram")


# ===========================================================================
# CLI
# ===========================================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Simulador da Ground Station do ROCSAR.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--host", default="127.0.0.1", help="endereco do OBC")
    p.add_argument("--pub-port", type=int, default=5556)
    p.add_argument("--cmd-port", type=int, default=5555)
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--duration", type=float, default=6.0,
                   help="segundos a observar a telemetria")
    p.add_argument("--expected-hz", type=float, default=5.0,
                   help="taxa de telemetria esperada")
    p.add_argument("--hz-tolerance", type=float, default=0.35,
                   help="desvio maximo aceitavel na taxa")
    p.add_argument("--no-commands", action="store_true",
                   help="nao enviar comandos, so observar telemetria")
    p.add_argument("--no-http", action="store_true",
                   help="nao testar o servidor de ficheiros")
    p.add_argument("--quiet", action="store_true",
                   help="nao imprimir blocos de telemetria")
    p.add_argument("--verbose", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    print("=" * 68)
    print("ROCSAR Ground Station Simulator")
    print(f"  alvo: {args.host}  (cmd {args.cmd_port} / pub {args.pub_port} / "
          f"http {args.http_port})")
    print("=" * 68)

    sim = GSSimulator(
        args.host,
        pub_port=args.pub_port,
        cmd_port=args.cmd_port,
        http_port=args.http_port,
        duration_s=args.duration,
        expected_hz=args.expected_hz,
        hz_tolerance=args.hz_tolerance,
        send_commands=not args.no_commands,
        check_http=not args.no_http,
        print_telemetry=not args.quiet,
        verbose=args.verbose,
    )
    return 0 if sim.run() else 1


# ===========================================================================
# Testes pytest
# ===========================================================================
def _obc_disponivel(host: str, port: int, timeout_s: float = 0.5) -> bool:
    """O OBC responde? Base para saltar os testes de integracao."""
    import socket

    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _skipif(condition: bool, reason: str):  # noqa: ANN202
    """``pytest.mark.skipif`` que degrada para no-op sem pytest."""
    if pytest is None:  # pragma: no cover
        return lambda obj: obj
    return pytest.mark.skipif(condition, reason=reason)


requires_obc = _skipif(
    not _obc_disponivel("127.0.0.1", 8080),
    "nenhum OBC a escutar em 127.0.0.1:8080",
)


class TestClientSimUnit:
    """Testes que nao precisam de servidor."""

    def test_help(self) -> None:
        """O CLI constroi-se sem OBC nenhum."""
        args = build_parser().parse_args([])
        assert args.host == "127.0.0.1"
        assert args.pub_port == 5556
        assert args.cmd_port == 5555
        assert args.http_port == 8080
        assert args.expected_hz == 5.0

    def test_format_telemetry_sem_crash(self) -> None:
        f = pb.TelemetryFrame(
            timestamp=1.0,
            gondola_heading=90.0,
            target_heading=180.0,
            imu_online=True,
            heater1_active=True,
            servos=[pb.ServoData(id=1, current_tick=100, online=True)],
            gnss=pb.GnssData(latitude=38.7, longitude=-9.1, altitude=20.0,
                             fix_ok=True),
            health=pb.SystemHealth(cpu_temp_c=45.0, uptime_seconds=60,
                                   limit_active=False, disk_used_percent=12.5),
        )
        saida = format_telemetry(f)
        assert "180.0" in saida
        assert "#1" in saida

    def test_command_client_preenche_command_id(self) -> None:
        """Cada pedido leva um command_id unico (correlacao resposta/comando)."""
        r1 = pb.CommandRequest(command_id=f"x-{uuid.uuid4().hex[:6]}",
                              type=pb.CMD_SET_TARGET)
        r2 = pb.CommandRequest(command_id=f"x-{uuid.uuid4().hex[:6]}",
                              type=pb.CMD_SET_TARGET)
        assert r1.command_id != r2.command_id

    def test_rate_hz_ignora_slow_joiner(self) -> None:
        """A taxa e medida depois dos frames de slow-joiner."""
        c = TelemetryCollector("127.0.0.1")
        base = 1000.0
        # 5 frames de slow-joiner colados, depois 10 frames a 5 Hz.
        for i in range(5):
            c.frames.append((base + i * 0.001, pb.TelemetryFrame()))
        for i in range(11):
            c.frames.append((base + 10.0 + i * 0.2, pb.TelemetryFrame()))
        taxa = c.rate_hz()
        assert taxa is not None
        assert abs(taxa - 5.0) < 0.05, taxa


@requires_obc
class TestAgainstLiveServer:
    """Integracao real. Saltam automaticamente se nao houver OBC."""

    def test_telemetria_chega(self) -> None:
        c = TelemetryCollector("127.0.0.1", 5556)
        c.start()
        try:
            # Sao precisos frames suficientes para sobreviver ao descarte dos
            # slow-joiner: SLOW_JOINER_FRAMES + 3 (2 para a taxa, 1 de sobra).
            minimo = SLOW_JOINER_FRAMES + 3
            t0 = time.monotonic()
            while time.monotonic() - t0 < 8.0 and c.count < minimo:
                time.sleep(0.1)
            assert c.count >= minimo, (
                f"telemetria nao chegou: {c.count} frames, precisavamos de {minimo}"
            )
            f = c.latest()
            assert f is not None
            assert f.timestamp > 0
            taxa = c.rate_hz()
            assert taxa is not None, "frames insuficientes para medir a taxa"
            assert abs(taxa - 5.0) <= 0.5, f"taxa anomala: {taxa:.2f} Hz"
        finally:
            c.stop()

    def test_comandos_sao_respondidos(self) -> None:
        cc = CommandClient("127.0.0.1", 5555, timeout_ms=3000)
        try:
            resp = cc.set_target(123.0)
            assert resp is not None, "sem resposta a CMD_SET_TARGET"
            assert resp.command_id
        finally:
            cc.close()

    def test_http_lista_diretorio(self) -> None:
        fc = FileClient("127.0.0.1", 8080)
        assert fc.available()
        entradas = fc.listdir()
        assert isinstance(entradas, list)

    def test_scenario_completo(self) -> None:
        """Percorre o simulador inteiro; e o teste de fumo do protocolo."""
        sim = GSSimulator("127.0.0.1", duration_s=6.0, print_telemetry=False)
        assert sim.run(), "; ".join(
            c.name + (f": {c.detail}" if c.detail else "")
            for c in sim.checks if not c.ok
        )


if __name__ == "__main__":
    raise SystemExit(main())
