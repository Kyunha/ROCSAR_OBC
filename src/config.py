"""Configuracao central do servidor OBC.

Toda a configuracao vive num unico objecto imutavel (:class:`Config`) obtido a
partir de argumentos de linha de comandos e/ou variaveis de ambiente.

Precedencia: **linha de comandos > ambiente > default**.

Nota sobre politica de simulacao
-------------------------------
Por omissao o servidor tenta falar com o hardware real. Os modos simulados
(``--mock-*``) so sao ativados por flag explicita, porque telemetria fabricada
em voo e perigoso. Se faltar hardware e nao for pedido mock, o servidor arranca
em **modo degradado**: publica ``online=False`` / ``fix_ok=False`` em vez de
inventar valores.
"""

from __future__ import annotations

import argparse
import logging
import os
import shlex
from dataclasses import dataclass, replace
from pathlib import Path

LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Defaults de fabricacao (o que se usa num OBC de voo)
# ---------------------------------------------------------------------------
DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_SERIAL_BAUD = 115200
DEFAULT_SERIAL_POLL_HZ = 10.0

DEFAULT_DATA_DIR = "/mnt/ssd/rocsar_data"
DEFAULT_FALLBACK_DATA_DIR = "./data"

#: Porta JSON do receptor GNSS (formato da spec ROCSAR).
DEFAULT_GNSS_JSON_PORT = 9000

#: Portas do `Read_uB`: 2000 = estado fundido, 2001-2003 = receptores, 2004 = DGPS.
DEFAULT_READUB_PORTS = (2000, 2001, 2002, 2003, 2004)
DEFAULT_READUB_PRIMARY_PORT = 2000

#: Segundos sem novidade GNSS antes de declarar o fix perdido.
DEFAULT_GNSS_STALE_S = 5.0

DEFAULT_TC_IFACE = "eth0"
DEFAULT_TC_BURST = "32kbit"
DEFAULT_TC_LATENCY = "400ms"

#: Entrada de comando de aquisicao SAR que o OBC executa. E o *dispatcher*,
#: nao a aquisicao em si: ver ``DEFAULT_SAR_DELEGATE``.
DEFAULT_SAR_COMMAND = "python3 -m sar_runner"

#: Aquisicao SAR real a que o dispatcher delega. `{out}` = ficheiro de destino.
#: Aponta para o script do SDR presente no repositorio. Tem de ser um comando
#: que NAO volte a chamar o dispatcher, senao obtem-se recursao infinita.
DEFAULT_SAR_DELEGATE = "./sdr-ettus-b200mini/run.sh"

#: Disparo de fotografia: `{out}` e substituido pelo caminho do ficheiro.
DEFAULT_CAMERA_COMMAND = (
    "ffmpeg -hide_banner -loglevel error -f v4l2 -i /dev/video0 -frames:v 1 -y {out}"
)

ENV_PREFIX = "ROCSAR_"


def _env(name: str, default: str) -> str:
    return os.environ.get(ENV_PREFIX + name, default)


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(ENV_PREFIX + name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(ENV_PREFIX + name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        LOG.warning("%s%s='%s' nao e inteiro; a usar %d", ENV_PREFIX, name, raw, default)
        return default


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(ENV_PREFIX + name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        LOG.warning("%s%s='%s' nao e numerico; a usar %s", ENV_PREFIX, name, raw, default)
        return default


def _split_ports(raw: str) -> tuple[int, ...]:
    ports: list[int] = []
    for chunk in raw.replace(",", " ").split():
        try:
            ports.append(int(chunk))
        except ValueError:
            LOG.warning("Porta GNSS invalida ignorada: '%s'", chunk)
    return tuple(ports)


@dataclass(frozen=True)
class Config:
    """Configuracao imutavel do servidor OBC."""

    # --- ZMQ / HTTP -------------------------------------------------------
    cmd_bind: str
    pub_bind: str
    http_host: str
    http_port: int
    telemetry_hz: float

    # --- Serial Pico ------------------------------------------------------
    serial_port: str
    serial_baud: int
    serial_poll_hz: float

    # --- Politica de simulacao -------------------------------------------
    mock_pico: bool
    mock_sar: bool
    mock_gnss: bool
    auto_mock: bool
    require_hardware: bool

    # --- GNSS -------------------------------------------------------------
    gnss_json_port: int
    gnss_json_enabled: bool
    readub_ports: tuple[int, ...]
    readub_primary_port: int
    readub_enabled: bool
    gnss_stale_s: float

    # --- Banda ------------------------------------------------------------
    tc_enabled: bool
    tc_iface: str
    sudo_prefix: tuple[str, ...]
    tc_burst: str
    tc_latency: str

    # --- Dados ------------------------------------------------------------
    data_dir: Path

    # --- Comandos externos ------------------------------------------------
    sar_command: tuple[str, ...]
    camera_command: tuple[str, ...]

    # --- Diagnostico -------------------------------------------------------
    log_level: str = "INFO"

    # ------------------------------------------------------------------
    @property
    def using_mock(self) -> bool:
        """True se pelo menos um subsistema esta em modo simulado."""
        return self.mock_pico or self.mock_sar or self.mock_gnss

    def mock_summary(self) -> str:
        """Descricao legivel dos mocks ativos (para log de arranque)."""
        ativos = [
            nome
            for nome, ligado in (
                ("pico", self.mock_pico),
                ("sar", self.mock_sar),
                ("gnss", self.mock_gnss),
            )
            if ligado
        ]
        if not ativos:
            return "nenhum (hardware real)"
        return ", ".join(ativos)


# ---------------------------------------------------------------------------
# Resolution do directorio de dados
# ---------------------------------------------------------------------------
def resolve_data_dir(preferido: Path | None = None) -> Path:
    """Devolve um diretorio de dados utilizavel, criando-o se necessario.

    Tenta ``/mnt/ssd/rocsar_data`` (o SSD de voo). Se nao for possivel criar
    esse caminho -- tipico na maquina de desenvolvimento -- usa
    ``./data`` e emite um aviso. Nunca lanca excecao: o servidor tem de arrancar
    mesmo sem SSD.
    """
    alvo = preferido if preferido is not None else Path(_env("DATA_DIR", DEFAULT_DATA_DIR))
    try:
        alvo.mkdir(parents=True, exist_ok=True)
        if not os.access(alvo, os.W_OK):
            raise PermissionError(f"sem permissao de escrita em {alvo}")
        return alvo
    except (OSError, PermissionError) as exc:
        fallback = Path(DEFAULT_FALLBACK_DATA_DIR)
        LOG.warning(
            "Data dir %s inutilizavel (%s); a usar fallback %s. "
            "Isto e normal em desenvolvimento, MAS em voo o fallback deve "
            "ser investigated antes de armar.",
            alvo,
            exc,
            fallback.resolve(),
        )
        fallback.mkdir(parents=True, exist_ok=True)
        return fallback


# ---------------------------------------------------------------------------
# Parser de argumentos
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rocsar-obc",
        description=(
            "Servidor On-Board Computer do ROCSAR. Por omissao opera com "
            "hardware real; simulacoes exigem flag explicita."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    g = p.add_argument_group("Rede / ZMQ")
    g.add_argument("--cmd-bind", default=_env("CMD_BIND", "tcp://0.0.0.0:5555"),
                   help="Endereco do socket ROUTER de comandos")
    g.add_argument("--pub-bind", default=_env("PUB_BIND", "tcp://0.0.0.0:5556"),
                   help="Endereco do socket PUB de telemetria")
    g.add_argument("--http-host", default=_env("HTTP_HOST", "0.0.0.0"),
                   help="Interface do servidor HTTP de ficheiros")
    g.add_argument("--http-port", type=int, default=_env_int("HTTP_PORT", 8080),
                   help="Porta do servidor HTTP de ficheiros")
    g.add_argument("--telemetry-hz", type=float, default=_env_float("TELEMETRY_HZ", 5.0),
                   help="Frequencia de publicacao de TelemetryFrame")

    g = p.add_argument_group("Serial Pico")
    g.add_argument("--serial-port", default=_env("SERIAL_PORT", DEFAULT_SERIAL_PORT),
                   help="Dispositivo USB da Raspberry Pi Pico")
    g.add_argument("--serial-baud", type=int, default=_env_int("SERIAL_BAUD", DEFAULT_SERIAL_BAUD),
                   help="Velocidade da ligacao ASCII")
    g.add_argument("--serial-poll-hz", type=float,
                   default=_env_float("SERIAL_POLL_HZ", DEFAULT_SERIAL_POLL_HZ),
                   help="Frequencia de interogacao STATUS")

    g = p.add_argument_group("Simulacao (explicita; nada e simulado sem flag)")
    g.add_argument("--mock", action="store_true", default=_env_flag("MOCK", False),
                   help="Ativa TODOS os modos simulados de uma vez")
    g.add_argument("--mock-pico", action="store_true", default=_env_flag("MOCK_PICO", False),
                   help="Simula a Pico (servos + IMU)")
    g.add_argument("--mock-sar", action="store_true", default=_env_flag("MOCK_SAR", False),
                   help="Simula a aquisicao SAR em vez de executar o comando real")
    g.add_argument("--mock-gnss", action="store_true", default=_env_flag("MOCK_GNSS", False),
                   help="Simula a posicao GNSS em vez de ler as fontes reais")
    g.add_argument("--auto-mock", action="store_true", default=_env_flag("AUTO_MOCK", False),
                   help=(
                       "Se o hardware estiver ausente, cair automaticamente para mock. "
                       "Desligue isto em voo (--require-hardware) para nunca haver "
                       "telemetria fabricada."
                   ))
    g.add_argument("--require-hardware", action="store_true",
                   default=_env_flag("REQUIRE_HARDWARE", False),
                   help=(
                       "Proibe qualquer fallback para mock. Sem hardware, o servidor "
                       "arranca degradado (online=False / fix_ok=False) ou falha."
                   ))

    g = p.add_argument_group("GNSS")
    g.add_argument("--gnss-json-port", type=int,
                   default=_env_int("GNSS_JSON_PORT", DEFAULT_GNSS_JSON_PORT),
                   help="Porta UDP do feed JSON do receptor GNSS")
    g.add_argument("--no-gnss-json", action="store_true",
                   default=_env_flag("NO_GNSS_JSON", False),
                   help="Desliga a fonte JSON")
    g.add_argument("--readub-ports", default=_env("READUB_PORTS",
                   " ".join(str(p) for p in DEFAULT_READUB_PORTS)),
                   help="Portas UDP NavData do Read_uB (2000 = estado fundido)")
    g.add_argument("--readub-primary-port", type=int,
                   default=_env_int("READUB_PRIMARY_PORT", DEFAULT_READUB_PRIMARY_PORT),
                   help="Porta NavData autoritativa para o TelemetryFrame")
    g.add_argument("--no-readub", action="store_true",
                   default=_env_flag("NO_READUB", False),
                   help="Desliga a fonte binaria Read_uB")
    g.add_argument("--gnss-stale-s", type=float,
                   default=_env_float("GNSS_STALE_S", DEFAULT_GNSS_STALE_S),
                   help="Segundos sem novidade antes de fix_ok=False")

    g = p.add_argument_group("Banda (tc)")
    g.add_argument("--no-tc", action="store_true", default=_env_flag("NO_TC", False),
                   help="Desliga completamente a gestao de banda")
    g.add_argument("--tc-iface", default=_env("TC_IFACE", DEFAULT_TC_IFACE),
                   help="Interface de rede a limitar")
    g.add_argument("--sudo-prefix", default=_env("SUDO_PREFIX", "sudo"),
                   help="Prefixo de privilegio. Ex.: 'sudo', 'doas tc', '' se ja root")
    g.add_argument("--tc-burst", default=_env("TC_BURST", DEFAULT_TC_BURST))
    g.add_argument("--tc-latency", default=_env("TC_LATENCY", DEFAULT_TC_LATENCY))

    g = p.add_argument_group("Dados")
    g.add_argument("--data-dir", default=None,
                   help=f"Diretorio servido por HTTP (default {DEFAULT_DATA_DIR})")

    g = p.add_argument_group("Comandos externos")
    g.add_argument("--sar-command", default=_env("SAR_COMMAND", DEFAULT_SAR_COMMAND),
                   help="Comando de aquisicao SAR. '{out}' opcional para o destino.")
    g.add_argument("--camera-command", default=_env("CAMERA_COMMAND", DEFAULT_CAMERA_COMMAND),
                   help="Comando de fotografia. '{out}' = caminho do ficheiro.")

    g = p.add_argument_group("Diagnostico")
    g.add_argument("--log-level", default=_env("LOG_LEVEL", "INFO"),
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])

    return p


def _parse_ports(raw: str) -> tuple[int, ...]:
    ports = _split_ports(raw)
    return ports or DEFAULT_READUB_PORTS


def config_from_args(args: argparse.Namespace) -> Config:
    """Constroi a :class:`Config` a partir de argumentos ja parseados."""
    mock = bool(args.mock)
    mock_pico = mock or bool(args.mock_pico)
    mock_sar = mock or bool(args.mock_sar)
    mock_gnss = mock or bool(args.mock_gnss)

    if args.require_hardware and (mock_pico or mock_sar or mock_gnss):
        raise SystemExit(
            "Conflito: --require-hardware proibe mocks, mas foi pedido "
            "--mock/--mock-pico/--mock-sar/--mock-gnss. Escolha um dos dois."
        )

    readub_ports = _parse_ports(args.readub_ports)
    if args.readub_primary_port not in readub_ports:
        LOG.warning(
            "--readub-primary-port %d nao esta em --readub-ports %s; "
            "o GNSS ficara sem fix valido.",
            args.readub_primary_port,
            list(readub_ports),
        )

    tc_enabled = not args.no_tc
    prefix: tuple[str, ...] = (
        tuple(shlex.split(args.sudo_prefix)) if tc_enabled else ()
    )

    return Config(
        cmd_bind=args.cmd_bind,
        pub_bind=args.pub_bind,
        http_host=args.http_host,
        http_port=args.http_port,
        telemetry_hz=args.telemetry_hz,
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
        serial_poll_hz=args.serial_poll_hz,
        mock_pico=mock_pico,
        mock_sar=mock_sar,
        mock_gnss=mock_gnss,
        auto_mock=bool(args.auto_mock),
        require_hardware=bool(args.require_hardware),
        gnss_json_port=args.gnss_json_port,
        gnss_json_enabled=not args.no_gnss_json,
        readub_ports=readub_ports,
        readub_primary_port=args.readub_primary_port,
        readub_enabled=not args.no_readub,
        gnss_stale_s=args.gnss_stale_s,
        tc_enabled=tc_enabled,
        tc_iface=args.tc_iface,
        sudo_prefix=prefix,
        tc_burst=args.tc_burst,
        tc_latency=args.tc_latency,
        data_dir=resolve_data_dir(Path(args.data_dir) if args.data_dir else None),
        sar_command=tuple(shlex.split(args.sar_command)),
        camera_command=tuple(shlex.split(args.camera_command)),
        log_level=args.log_level,
    )


def load_config(argv: list[str] | None = None) -> Config:
    """Ponto de entrada: parse de argv e devolve a configuracao final."""
    args = build_parser().parse_args(argv)
    cfg = config_from_args(args)

    if cfg.telemetry_hz <= 0:
        raise SystemExit("--telemetry-hz tem de ser > 0")
    if cfg.serial_poll_hz <= 0:
        raise SystemExit("--serial-poll-hz tem de ser > 0")
    if not cfg.sar_command:
        raise SystemExit("--sar-command nao pode ser vazio")
    return cfg


def with_data_dir(cfg: Config, path: Path) -> Config:
    """Copia da config com outro diretorio de dados (usado nos testes)."""
    return replace(cfg, data_dir=path)
