"""Servidor principal do On-Board Computer (OBC) do ROCSAR.

Monta tres servicos numa unica thread de comandos:

===============  ========  ==========================================
Servico          Porta     Papel
===============  ========  ==========================================
ZMQ ROUTER       5555      Recebe ``CommandRequest`` (DEALER da GS)
ZMQ PUB          5556      Publica ``TelemetryFrame`` a 5 Hz
HTTP             8080      Serve o directorio de dados do SSD
===============  ========  ==========================================

Subsistemas auxiliares correm em threads daemon separados: polling da Pico
(:mod:`src.serial_pico`), GNSS (:mod:`src.gnss_listener`) e saude do sistema
(:mod:`src.system_health`).

Politica de simulacao
---------------------
Por omissao tudo opera com hardware real. As opcoes ``--mock*`` simulam
subsistemas e **tem de ser pedidas explicitamente**. Sem hardware e sem mock, o
servidor arranca em modo degradado e publica ``online=False`` / ``fix_ok=False``
em vez de fabricar leituras.
"""

from __future__ import annotations

import contextlib
import functools
import http.server
import logging
import signal
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import zmq

from . import rocsar_messages_pb2 as pb
from .bandwidth_manager import BandwidthManager
from .command_runner import CommandRunner
from .config import Config, load_config
from .gnss_listener import GnssListener
from .serial_pico import PicoController, PicoError, TelemetrySnapshot
from .system_health import SystemHealthMonitor

LOG = logging.getLogger(__name__)

#: Topico ZMQ da telemetria descendente.
TELEMETRY_TOPIC = b"TELEMETRY"

#: Numero maximo de bytes num comando (proteccao contra payloads absurdos).
MAX_COMMAND_BYTES = 64 * 1024

#: Tempo maximo de espera por uma resposta de comando (usado nos testes).
COMMAND_TIMEOUT_MS = 5000


# ===========================================================================
# Respostas
# ===========================================================================
@dataclass
class CommandOutcome:
    """Resultado de um comando, antes de ser serializado."""

    success: bool
    message: str


def ok(message: str) -> CommandOutcome:
    return CommandOutcome(True, message)


def fail(message: str) -> CommandOutcome:
    return CommandOutcome(False, message)


# ===========================================================================
# Servidor HTTP de ficheiros
# ===========================================================================
class _DataFileHandler(http.server.SimpleHTTPRequestHandler):
    """Handler HTTP que serve o directorio de dados e loga em modo DEBUG."""

    server_version = "ROCSAR-OBC/0.1"

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002
        LOG.debug("HTTP %s - %s", self.address_string(), format % args)

    def log_error(self, format: str, *args: object) -> None:  # noqa: A002
        LOG.warning("HTTP %s - %s", self.address_string(), format % args)


class FileServer:
    """Servidor HTTP de ficheiros do SSD, numa thread dedicada."""

    def __init__(self, directory: Path, host: str, port: int) -> None:
        self._directory = directory
        self._host = host
        self._port = port
        self._httpd: http.server.ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        handler = functools.partial(_DataFileHandler, directory=str(self._directory))
        try:
            self._httpd = http.server.ThreadingHTTPServer(
                (self._host, self._port), handler
            )
        except OSError as exc:
            # Perder o HTTP nao pode derrubar telemetria nem comandos.
            LOG.error(
                "HTTP: nao foi possivel escutar em %s:%d (%s). "
                "O download de ficheiros fica indisponivel.",
                self._host, self._port, exc,
            )
            self._httpd = None
            return

        self._httpd.daemon_threads = True
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            kwargs={"poll_interval": 0.5},
            name="http-files",
            daemon=True,
        )
        self._thread.start()
        LOG.info("HTTP: a servir %s em http://%s:%d/", self._directory, self._host, self._port)

    def stop(self) -> None:
        if self._httpd is not None:
            with contextlib.suppress(Exception):
                self._httpd.shutdown()
            with contextlib.suppress(Exception):
                self._httpd.server_close()
            self._httpd = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


# ===========================================================================
# Servidor OBC
# ===========================================================================
class OBCServer:
    """Agrega todos os subsistemas do OBC."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self._ctx = zmq.Context.instance()

        self.pico = PicoController(
            port=cfg.serial_port,
            baud=cfg.serial_baud,
            poll_hz=cfg.serial_poll_hz,
            mock=cfg.mock_pico,
            auto_mock=cfg.auto_mock,
            require_hardware=cfg.require_hardware,
        )

        self.gnss = GnssListener(
            json_port=cfg.gnss_json_port,
            json_enabled=cfg.gnss_json_enabled,
            readub_ports=cfg.readub_ports,
            readub_primary_port=cfg.readub_primary_port,
            readub_enabled=cfg.readub_enabled,
            stale_s=cfg.gnss_stale_s,
            mock=cfg.mock_gnss,
        )

        self.bandwidth = BandwidthManager(
            iface=cfg.tc_iface,
            sudo_prefix=cfg.sudo_prefix,
            burst=cfg.tc_burst,
            latency=cfg.tc_latency,
            enabled=cfg.tc_enabled,
        )

        self.health = SystemHealthMonitor(cfg.data_dir)

        self.sar = CommandRunner(
            "sar", cfg.sar_command,
            work_dir=cfg.data_dir, mock=cfg.mock_sar,
        )
        self.camera = CommandRunner(
            "camera", cfg.camera_command,
            work_dir=cfg.data_dir, mock=cfg.mock_sar,
        )

        self.files = FileServer(cfg.data_dir, cfg.http_host, cfg.http_port)

        # Estado comandado pelo operador (a Pico nao teletransmete aquecedores).
        self._heaters = {1: False, 2: False}
        self._target_heading = 0.0
        self._state_lock = threading.Lock()

        self._cmd_sock: zmq.Socket | None = None
        self._pub_sock: zmq.Socket | None = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.commands_handled = 0

    # ------------------------------------------------------------------
    # Ciclo de vida
    # ------------------------------------------------------------------
    def start(self) -> None:
        LOG.info("=" * 68)
        LOG.info("ROCSAR OBC a arrancar")
        LOG.info("  dados      : %s", self.cfg.data_dir)
        LOG.info("  comandos   : %s", self.cfg.cmd_bind)
        LOG.info("  telemetria : %s (%.1f Hz)", self.cfg.pub_bind, self.cfg.telemetry_hz)
        LOG.info("  HTTP       : http://%s:%d/", self.cfg.http_host, self.cfg.http_port)
        LOG.info("  Pico       : %s @ %d baud (mock=%s)", self.cfg.serial_port,
                 self.cfg.serial_baud, self.cfg.mock_pico)
        LOG.info("  GNSS       : json=%s readub=%s primaria=%s",
                 self.cfg.gnss_json_port, list(self.cfg.readub_ports),
                 self.cfg.readub_primary_port)
        LOG.info("  banda      : %s", self.cfg.tc_iface)
        LOG.info("  mocks      : %s", self.cfg.mock_summary())
        if self.cfg.using_mock:
            LOG.warning("*** MODO DE DESENVOLVIMENTO: telemetria ARTIFICIAL ***")
            LOG.warning("*** Nao use em voo. Ver README.md, secao 'MOCK'. ***")
        LOG.info("=" * 68)

        # Bind do PUB ANTES de publicar: o ZMQ PUB descarta mensagens enviadas
        # antes de um SUB se ligar (slow joiner).
        self._pub_sock = self._ctx.socket(zmq.PUB)
        self._pub_sock.setsockopt(zmq.LINGER, 0)
        self._pub_sock.setsockopt(zmq.SNDHWM, 100)
        self._pub_sock.bind(self.cfg.pub_bind)

        self._cmd_sock = self._ctx.socket(zmq.ROUTER)
        self._cmd_sock.setsockopt(zmq.LINGER, 0)
        self._cmd_sock.setsockopt(zmq.RCVHWM, 100)
        self._cmd_sock.bind(self.cfg.cmd_bind)
        LOG.info("Comandos: ROUTER ligado em %s", self.cfg.cmd_bind)

        self.pico.start()
        self.gnss.start()
        self.files.start()

        self._threads = [
            self._spawn("telemetry", self._telemetry_loop),
            self._spawn("reaper", self._reaper_loop),
        ]

    def _spawn(self, name: str, target: Callable[[], None]) -> threading.Thread:
        th = threading.Thread(target=target, name=name, daemon=True)
        th.start()
        return th

    def stop(self) -> None:
        if self._stop.is_set():
            return
        LOG.info("ROCSAR OBC a encerrar...")
        self._stop.set()

        for th in self._threads:
            th.join(timeout=2.0)
        self._threads.clear()

        with contextlib.suppress(Exception):
            self.sar.stop()
        with contextlib.suppress(Exception):
            self.camera.stop()

        self.gnss.stop()
        self.pico.stop()
        self.files.stop()

        for sock in (self._cmd_sock, self._pub_sock):
            if sock is not None:
                with contextlib.suppress(Exception):
                    sock.close(linger=0)
        self._cmd_sock = None
        self._pub_sock = None
        LOG.info("ROCSAR OBC terminado.")

    # ------------------------------------------------------------------
    # Threads auxiliares
    # ------------------------------------------------------------------
    def _reaper_loop(self) -> None:
        """Recolhe os processos externos terminados."""
        while not self._stop.wait(2.0):
            for runner in (self.sar, self.camera):
                try:
                    runner.reap()
                except Exception as exc:  # pragma: no cover
                    LOG.warning("reaper %s: %s", runner.name, exc)

    # ------------------------------------------------------------------
    # Telemetria
    # ------------------------------------------------------------------
    def build_telemetry(self) -> pb.TelemetryFrame:
        """Monta um TelemetryFrame a partir do estado de todos os subsistemas."""
        frame = pb.TelemetryFrame()
        frame.timestamp = time.time()

        snap: TelemetrySnapshot = self.pico.snapshot()
        frame.gondola_heading = snap.gondola_heading
        frame.target_heading = snap.target_heading
        frame.imu_online = snap.imu_online
        frame.heater1_active = snap.heater1_active
        frame.heater2_active = snap.heater2_active

        for status in snap.servos:
            frame.servos.append(status.to_proto())

        fix = self.gnss.snapshot()
        if fix is not None:
            frame.gnss.latitude = fix.latitude
            frame.gnss.longitude = fix.longitude
            frame.gnss.altitude = fix.altitude
            frame.gnss.fix_ok = fix.fix_ok
        else:
            # proto3 nao distingue 0 de ausente; a GS le fix_ok para saber.
            frame.gnss.fix_ok = False

        self.health.fill(frame.health, limit_active=self.bandwidth.limit_active)
        return frame

    def _telemetry_loop(self) -> None:
        periodo = 1.0 / self.cfg.telemetry_hz
        while not self._stop.is_set():
            inicio = time.monotonic()
            try:
                if self._pub_sock is not None:
                    self._pub_sock.send_multipart(
                        [TELEMETRY_TOPIC, self.build_telemetry().SerializeToString()]
                    )
            except zmq.ZMQError as exc:
                LOG.warning("Telemetria: erro ZMQ: %s", exc)
            except Exception as exc:  # pragma: no cover
                LOG.exception("Telemetria: erro inesperado: %s", exc)

            decorrido = time.monotonic() - inicio
            self._stop.wait(max(0.0, periodo - decorrido))

    # ------------------------------------------------------------------
    # Comandos
    # ------------------------------------------------------------------
    def handle_command(self, request: pb.CommandRequest) -> CommandOutcome:
        """Executa um ``CommandRequest`` e devolve o resultado.

        Nunca levanta excecao: qualquer falha vira um ``CommandResponse`` com
        ``success=False`` e o motivo.
        """
        tipo = request.type
        # Consistencia entre `type` e o oneof preenchido: um comando que diz
        # CMD_SET_TARGET mas traz set_bandwidth e um erro do cliente.
        esperado = {
            pb.CMD_SET_TARGET: "set_target",
            pb.CMD_SET_BANDWIDTH: "set_bandwidth",
            pb.CMD_CONTROL_HEATER: "control_heater",
            pb.CMD_TRIGGER_CAMERA: "trigger_camera",
            pb.CMD_JOG_SERVO: "jog_servo",
        }.get(tipo)

        preenchido = request.WhichOneof("payload")
        if esperado and preenchido and preenchido != esperado:
            return fail(
                f"payload incoerente: type={pb.CommandType.Name(tipo)} espera "
                f"'{esperado}' mas recebeu '{preenchido}'"
            )

        handler: dict[int, Callable[[pb.CommandRequest], CommandOutcome]] = {
            pb.CMD_SET_TARGET: self._cmd_set_target,
            pb.CMD_SET_BANDWIDTH: self._cmd_set_bandwidth,
            pb.CMD_CONTROL_HEATER: self._cmd_control_heater,
            pb.CMD_TRIGGER_SAR: self._cmd_trigger_sar,
            pb.CMD_TRIGGER_CAMERA: self._cmd_trigger_camera,
            pb.CMD_JOG_SERVO: self._cmd_jog_servo,
        }
        fn = handler.get(tipo)
        if fn is None:
            return fail(f"comando desconhecido ou nao implementado: {tipo}")
        if esperado and preenchido is None:
            return fail(
                f"{pb.CommandType.Name(tipo)} exige o payload '{esperado}'"
            )

        try:
            return fn(request)
        except PicoError as exc:
            return fail(f"Pico: {exc}")
        except Exception as exc:  # pragma: no cover - rede de seguranca
            LOG.exception("Comando %s falhou", request.command_id)
            return fail(f"erro interno a tratar {pb.CommandType.Name(tipo)}: {exc}")

    # -- handlers ------------------------------------------------------
    def _cmd_set_target(self, req: pb.CommandRequest) -> CommandOutcome:
        heading = float(req.set_target.target_heading)
        self.pico.set_target(heading)
        with self._state_lock:
            self._target_heading = heading % 360.0
        return ok(f"alvo de orientacao definido para {heading % 360.0:.1f} graus")

    def _cmd_set_bandwidth(self, req: pb.CommandRequest) -> CommandOutcome:
        rate = int(req.set_bandwidth.rate_kbps)
        ok_flag, msg = self.bandwidth.set_limit(rate)
        if ok_flag:
            return ok(msg)
        LOG.warning("CMD_SET_BANDWIDTH falhou: %s", msg)
        return fail(msg)

    def _cmd_control_heater(self, req: pb.CommandRequest) -> CommandOutcome:
        heater_id = int(req.control_heater.heater_id)
        enable = bool(req.control_heater.enable)
        self.pico.set_heater(heater_id, enable)
        with self._state_lock:
            self._heaters[heater_id] = enable
        estado = "LIGADO" if enable else "DESLIGADO"
        return ok(f"aquecedor {heater_id} {estado}")

    def _cmd_trigger_sar(self, req: pb.CommandRequest) -> CommandOutcome:
        self.sar.reap()
        ok_flag, msg = self.sar.start()
        return ok(msg) if ok_flag else fail(msg)

    def _cmd_trigger_camera(self, req: pb.CommandRequest) -> CommandOutcome:
        count = int(req.trigger_camera.count) or 1
        spacing_ms = float(req.trigger_camera.spacing_ms)
        if count < 1 or count > 1000:
            return fail(f"count invalido: {count} (intervalo 1..1000)")
        if spacing_ms < 0 or spacing_ms > 60_000:
            return fail(f"spacing_ms invalido: {spacing_ms} (intervalo 0..60000)")

        if self.cfg.mock_sar:
            ok_flag, msg = self.camera.start()
            return ok(f"{msg} (mock, {count} frame(s) pedidos)") if ok_flag else fail(msg)

        # Sequencia com espacamento: a camara e lenta e nao aceita sobrepor
        # disparos. Para count == 1 e' um unico comando.
        if count == 1:
            ok_flag, msg = self.camera.start(suffix="jpg")
            return ok(msg) if ok_flag else fail(msg)

        LOG.info("CMD_TRIGGER_CAMERA: %d frames com %.0f ms de espacamento", count, spacing_ms)
        disparados = 0
        for i in range(count):
            ok_flag, msg = self.camera.start(suffix="jpg")
            if not ok_flag:
                if disparados == 0:
                    return fail(msg)
                return fail(f"{disparados}/{count} frames capturados; ultimo erro: {msg}")
            disparados += 1
            if i + 1 < count:
                self._stop.wait(spacing_ms / 1000.0)
        return ok(f"{disparados} frame(s) capturado(s) em {self.cfg.data_dir}")

    def _cmd_jog_servo(self, req: pb.CommandRequest) -> CommandOutcome:
        servo_id = int(req.jog_servo.servo_id)
        tick = int(req.jog_servo.target_tick)
        self.pico.jog_servo(servo_id, tick)
        return ok(f"servo {servo_id} movido para o tick {tick} (modo manual)")

    # ------------------------------------------------------------------
    # Ciclo de comandos
    # ------------------------------------------------------------------
    def _serve_commands(self) -> None:
        """Laco principal de recepcao de comandos (executado na main thread)."""
        assert self._cmd_sock is not None
        poller = zmq.Poller()
        poller.register(self._cmd_sock, zmq.POLLIN)

        while not self._stop.is_set():
            # Poll com timeout curto para poder parar com SIGINT sem bloqueio.
            try:
                events = dict(poller.poll(200))
            except zmq.ZMQError as exc:
                if self._stop.is_set():
                    break
                LOG.warning("Comandos: erro no poll: %s", exc)
                continue

            if self._cmd_sock not in events:
                continue

            try:
                parts = self._cmd_sock.recv_multipart()
            except zmq.ZMQError as exc:
                LOG.warning("Comandos: erro no recv: %s", exc)
                continue

            self._dispatch(parts)

    def _dispatch(self, parts: list[bytes]) -> None:
        """Processa um multipart do ROUTER e responde ao remetente."""
        assert self._cmd_sock is not None
        if len(parts) < 2:
            LOG.warning("Comandos: frame inesperado com %d partes: %r", len(parts), parts)
            return

        # ROUTER: [identity, <delimitadores...>, payload]. O payload e sempre o
        # ultimo frame, por isso nao assumimos um numero fixo de partes.
        identity = parts[0]
        payload = parts[-1]
        command_id = ""

        if len(payload) > MAX_COMMAND_BYTES:
            LOG.warning(
                "Comandos: payload de %d bytes (max %d) recusado",
                len(payload), MAX_COMMAND_BYTES,
            )
            self._reply(identity, pb.CommandResponse(
                command_id="", success=False, message="payload demasiado grande"))
            return

        request = pb.CommandRequest()
        try:
            request.ParseFromString(payload)
        except Exception as exc:
            LOG.warning("Comandos: protobuf invalido: %s", exc)
            self._reply(identity, pb.CommandResponse(
                command_id="", success=False, message=f"protobuf invalido: {exc}"))
            return

        command_id = request.command_id
        LOG.info(
            "CMD %s: %s",
            command_id or "(sem id)",
            pb.CommandType.Name(request.type) if request.type else "CMD_UNKNOWN",
        )

        outcome = self.handle_command(request)
        self.commands_handled += 1

        if not outcome.success:
            LOG.warning("CMD %s falhou: %s", command_id, outcome.message)
        else:
            LOG.info("CMD %s ok: %s", command_id, outcome.message)

        self._reply(identity, pb.CommandResponse(
            command_id=command_id,
            success=outcome.success,
            message=outcome.message,
        ))

    def _reply(self, identity: bytes, response: pb.CommandResponse) -> None:
        assert self._cmd_sock is not None
        try:
            self._cmd_sock.send_multipart([identity, b"", response.SerializeToString()])
        except zmq.ZMQError as exc:
            LOG.warning("Comandos: falha ao responder: %s", exc)

    # ------------------------------------------------------------------
    def run(self) -> int:
        """Arranca e serve ate Ctrl-C / SIGTERM. Devolve o exit code."""
        self.start()
        stop_reason = "SIGINT"

        def _handle_signal(signum: int, _frame: object) -> None:
            nonlocal stop_reason
            stop_reason = signal.Signals(signum).name
            LOG.info("Recebido %s, a encerrar...", stop_reason)
            self._stop.set()

        anterior_int = signal.signal(signal.SIGINT, _handle_signal)
        anterior_term = signal.signal(signal.SIGTERM, _handle_signal)

        try:
            self._serve_commands()
        except KeyboardInterrupt:
            LOG.info("Ctrl-C, a encerrar...")
        finally:
            signal.signal(signal.SIGINT, anterior_int)
            signal.signal(signal.SIGTERM, anterior_term)
            self.stop()

        LOG.info(
            "Resumo: %d comandos tratados, %s",
            self.commands_handled,
            stop_reason,
        )
        return 0


# ===========================================================================
# main
# ===========================================================================
def main(argv: list[str] | None = None) -> int:
    cfg = load_config(argv)
    logging.basicConfig(
        level=getattr(logging, cfg.log_level, logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)-22s %(message)s",
    )
    if cfg.using_mock:
        logging.getLogger().warning(
            "MOCK ativo em: %s -- telemetria ARTIFICIAL, nao usar em voo",
            cfg.mock_summary(),
        )
    return OBCServer(cfg).run()


if __name__ == "__main__":
    raise SystemExit(main())
