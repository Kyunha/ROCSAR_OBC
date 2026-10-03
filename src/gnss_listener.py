"""Ingestao de dados GNSS.

Suporta duas fontes, ambas em UDP local:

**1. Feed JSON** (formato da spec ROCSAR) na porta 9000::

    {"lat": 41.1579, "lon": -8.6291, "alt": 120.5}

**2. Feed binario ``Read_uB``** (``struct NavData``) nas portas 2000-2004.

O ``Read_uB`` e um servidor UDP *push* com registo: ``UDP_Comm_Server`` so faz
``sendto`` para clientes previamente registados, e cada registo expira. Por isso
esta fonte precisa de enviar um registo de 2 bytes (**little-endian**) e de o
renovar periodicamente. Sem isso, nao chega zero datagramas.

Formato ``NavData`` (420 bytes, ``#pragma pack(1)``, sem header, sem checksum)::

    offset   tipo        campo
    ------   ----------  --------------------------------------------
       0     char[8]     label   ("tty00", "Straplex", ...)
       8     int64       NAV_time.tv_sec
      16     int64       NAV_time.tv_nsec
      24     double      Latitude   (graus, WGS-84)
      32     double      Longitude  (graus, WGS-84)
      40     double      Altitude   (metros, altura ELIPSOIDAL, nao MSL)
      ...
     120     uint16      flags      (bit 0 = LLH_VALID)
     122     uint16      stage
    ------
     420 bytes

Nao existe magic number nem tipo no datagrama: a distincao faz-se pela **porta
de origem** e pelo campo ``label``. A porta **2000** transporta o estado
*fundido* (o ``state`` ja escolheu o melhor GNSS e aplicou fallback de
staleness) e e a fonte autoritativa para o ``TelemetryFrame``.

Modos
-----
Real por omissao. ``--mock-gnss`` gera uma trajetoria sintetica. Sem nenhuma das
duas, o GNSS e publicado com ``fix_ok=False``.
"""

from __future__ import annotations

import json
import logging
import math
import os
import socket
import struct
import threading
import time
from dataclasses import dataclass

LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Layout do NavData (Read_uB/nav_data.h:52, #pragma pack(1))
# ---------------------------------------------------------------------------
NAVDATA_SIZE = 420

OFF_LABEL = 0        # char[8]
OFF_NAV_SEC = 8      # int64
OFF_NAV_NSEC = 16    # int64
OFF_LATITUDE = 24    # double
OFF_LONGITUDE = 32   # double
OFF_ALTITUDE = 40    # double
OFF_FLAGS = 120      # uint16
OFF_STAGE = 122      # uint16

#: ``nav_data.h:25`` -- coordenadas geograficas validas.
LLH_VALID = 0x0001

#: Intervalo de registo pedido ao servidor ``UDP_Comm_Server``.
REGISTER_EXPIRY_S = 125

#: Renovar a metastade do registo periodicamente (ver ``udp_comm.cpp:173``).
RENEW_INTERVAL_S = 30.0

DEFAULT_HOST = "127.0.0.1"


@dataclass(frozen=True)
class GnssFix:
    """Uma posicao GNSS, ja normalizada (independente da fonte)."""

    latitude: float
    longitude: float
    altitude: float
    fix_ok: bool
    source: str
    received_at: float

    def age_s(self, now: float | None = None) -> float:
        return (now if now is not None else time.time()) - self.received_at


@dataclass(frozen=True)
class NavData:
    """Cabecalho do ``NavData`` que interessa ao OBC."""

    label: str
    latitude: float
    longitude: float
    altitude: float
    flags: int
    stage: int

    @property
    def llh_valid(self) -> bool:
        return bool(self.flags & LLH_VALID)


class GnssError(RuntimeError):
    """Erro de GNSS (transmitido a CommandResponse, quando aplicavel)."""


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def parse_navdata(payload: bytes) -> NavData | None:
    """Faz o parse de um datagrama ``NavData``. Devolve ``None`` se invalido.

    O formato nao tem magic, versao nem prefixo de tamanho, por isso o unico
    criterio de versao e o **tamanho exacto**. Datagramas maiores ou menores
    que 420 bytes sao recusados.
    """
    if len(payload) != NAVDATA_SIZE:
        LOG.debug("NavData com tamanho inesperado: %d (esperado %d)",
                  len(payload), NAVDATA_SIZE)
        return None

    try:
        raw_label = payload[OFF_LABEL:OFF_LABEL + 8]
        # label vem com padding de zeros e pode nao ser terminada (ex: "Straplex").
        label = raw_label.split(b"\x00", 1)[0].decode("ascii", errors="replace")
        latitude, longitude, altitude = struct.unpack_from(
            "<ddd", payload, OFF_LATITUDE
        )
        flags, stage = struct.unpack_from("<HH", payload, OFF_FLAGS)
    except struct.error as exc:  # pragma: no cover - tamanho ja validado
        LOG.debug("NavData malformado: %s", exc)
        return None

    # Sanidade fisica: endianness errado ou payload corrompido produz valores
    # absurdos. Rejeitamos em vez de publicar posicoes inventadas.
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
        LOG.debug("NavData com valor nao finito em lat/lon/alt")
        return None
    if not (-90.0 <= latitude <= 90.0):
        LOG.debug("NavData latitude fora de range: %r", latitude)
        return None
    if not (-180.0 <= longitude <= 180.0):
        LOG.debug("NavData longitude fora de range: %r", longitude)
        return None

    return NavData(
        label=label,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        flags=flags,
        stage=stage,
    )


def _first_present(payload: dict[str, object], *names: str) -> object | None:
    """Primeiro valor presente entre varios nomes alternativos."""
    for nome in names:
        if nome in payload:
            return payload[nome]
    return None


def _as_float(valor: object) -> float | None:
    """Converte um valor de JSON em ``float``, ou ``None`` se nao der.

    O JSON pode trazer ``null``, uma string, ou qualquer coisa em que nao
    devemos confiar. Rejeitamos em vez de adivinhar.
    """
    if valor is None or isinstance(valor, bool):
        return None
    try:
        return float(valor)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


def parse_gnss_json(raw: bytes | str) -> GnssFix | None:
    """Faz o parse de um datagrama JSON. Devolve ``None`` se nao for GNSS."""
    if isinstance(raw, bytes):
        try:
            texto = raw.decode("utf-8")
        except UnicodeDecodeError:
            LOG.debug("Datagrama JSON nao e UTF-8")
            return None
    else:
        texto = raw

    texto = texto.strip()
    if not texto:
        return None

    try:
        doc = json.loads(texto)
    except json.JSONDecodeError as exc:
        LOG.debug("JSON invalido (%s): %r", exc, texto[:120])
        return None

    # Aceita um objecto, uma lista, ou um envelope tipo {"data": {...}}.
    if isinstance(doc, list):
        doc = next((d for d in reversed(doc) if isinstance(d, dict)), None)
    if not isinstance(doc, dict):
        return None

    for chave in ("data", "nav", "position", "pos"):
        if chave in doc and isinstance(doc[chave], dict):
            doc = doc[chave]
            break

    lat = _first_present(doc, "lat", "latitude")
    lon = _first_present(doc, "lon", "long", "lng", "longitude")
    latitude = _as_float(lat)
    longitude = _as_float(lon)
    if latitude is None or longitude is None:
        LOG.debug("lat/lon ausentes ou nao numericos: %r / %r", lat, lon)
        return None

    altitude = _as_float(
        _first_present(doc, "alt", "altitude", "hgt", "height")
    )
    if altitude is None:
        altitude = 0.0

    if not (-90.0 <= latitude <= 90.0) or not (-180.0 <= longitude <= 180.0):
        LOG.debug("JSON lat/lon fora de range: %r / %r", latitude, longitude)
        return None

    bruto_fix = _first_present(doc, "fix_ok", "fix", "valid", "llh_valid")
    if bruto_fix is None:
        fix_ok = True
    elif isinstance(bruto_fix, bool):
        fix_ok = bruto_fix
    elif isinstance(bruto_fix, (int, float)):
        fix_ok = bool(bruto_fix)
    else:
        fix_ok = str(bruto_fix).strip().lower() in {"1", "true", "yes", "ok"}

    return GnssFix(
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        fix_ok=fix_ok,
        source="json",
        received_at=time.time(),
    )


# ---------------------------------------------------------------------------
# Armazenamento
# ---------------------------------------------------------------------------
class GnssStore:
    """Ultimo fix conhecido, protected por lock, com deteccao de staleness.

    So a fonte primaria alimenta este store; as restantes servem para
    diagnostico (logs).
    """

    def __init__(self, *, source: str, stale_s: float) -> None:
        self._lock = threading.Lock()
        self._fix: GnssFix | None = None
        self._source = source
        self._stale_s = stale_s
        self._updates = 0

    def update(self, fix: GnssFix) -> None:
        with self._lock:
            self._fix = fix
            self._updates += 1

    def snapshot(self) -> GnssFix | None:
        """Fix actual, ou ``None`` se nunca chegou nada ou ja expirou."""
        with self._lock:
            fix = self._fix
            stale_s = self._stale_s
        if fix is None:
            return None
        if fix.age_s() > stale_s:
            return None
        return fix

    @property
    def update_count(self) -> int:
        with self._lock:
            return self._updates


# ---------------------------------------------------------------------------
# Fontes
# ---------------------------------------------------------------------------
class _BaseSource:
    """Funcionalidade comum: thread, arranque e paragem limpos."""

    def __init__(self, store: GnssStore | None, name: str, period_s: float) -> None:
        self._store = store
        self._name = name
        self._period_s = period_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def name(self) -> str:
        return self._name

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name=f"gnss-{self._name}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._cleanup()

    def _cleanup(self) -> None:
        """Liberta recursos especificos da fonte."""

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._pump()
            except OSError as exc:
                LOG.warning("GNSS %s: erro de socket: %s", self._name, exc)
                self._stop.wait(self._period_s)
            except Exception as exc:  # pragma: no cover - rede de seguranca
                LOG.exception("GNSS %s: erro inesperado: %s", self._name, exc)
                self._stop.wait(self._period_s)

    def _pump(self) -> None:  # pragma: no cover - abstrato
        raise NotImplementedError

    def _publish(self, fix: GnssFix) -> None:
        if self._store is None:
            LOG.debug("GNSS %s: fix %s (diagnostico)", self._name, fix)
        else:
            self._store.update(fix)
        LOG.debug("GNSS %s: %s", self._name, fix)


class JsonGnssSource(_BaseSource):
    """Escuta o feed JSON num socket UDP ligado a ``host:port``.

    Faz ``bind()`` -- o OBC e o *servidor* desta fonte: e o receptor GNSS que
    envia para o OBC. O socket fica sem ``connect``, por isso aceita datagramas
    de qualquer emissor (tipicamente ``127.0.0.1``).
    """

    def __init__(self, store: GnssStore | None, *, host: str = DEFAULT_HOST,
                 port: int = 9000, period_s: float = 1.0) -> None:
        super().__init__(store, f"json:{port}", period_s)
        self._addr = (host, port)
        self._sock: socket.socket | None = None

    def _open(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR evita "address already in use" quando o OBC reinicia.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(self._addr)
        sock.settimeout(self._period_s)
        LOG.info("GNSS JSON: a escutar em %s:%d", self._addr[0], self._addr[1])
        return sock

    def _pump(self) -> None:
        if self._sock is None:
            try:
                self._sock = self._open()
            except OSError as exc:
                LOG.warning(
                    "GNSS JSON: nao foi possivel escutar em %s:%d (%s)",
                    self._addr[0], self._addr[1], exc,
                )
                self._stop.wait(self._period_s)
                return

        assert self._sock is not None
        try:
            raw, _ = self._sock.recvfrom(4096)
        except TimeoutError:
            return
        except OSError:
            self._sock.close()
            self._sock = None
            raise

        fix = parse_gnss_json(raw)
        if fix is None:
            return
        fix = GnssFix(
            latitude=fix.latitude,
            longitude=fix.longitude,
            altitude=fix.altitude,
            fix_ok=fix.fix_ok,
            source=f"json:{self._addr[1]}",
            received_at=fix.received_at,
        )
        self._publish(fix)

    def _cleanup(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None


class ReadUbGnssSource(_BaseSource):
    """Cliente registado do servidor ``UDP_Comm_Server`` do ``Read_uB``.

    O protocolo e "push com registo":

    1. enviar ``struct.pack('<H', 125)`` para o servidor (2 bytes, **little-endian**
       -- ver ``udp_comm.cpp:176-177``);
    2. receber datagramas de 420 bytes;
    3. **renovar** o registo antes de expirar, ou o servidor poda-nos.

    O socket e deixado propositadamente sem ``bind``: o servidor responde para a
    porta efemera que o kernel atribuir.
    """

    def __init__(
        self,
        store: GnssStore | None,
        *,
        port: int,
        host: str = DEFAULT_HOST,
        period_s: float = 1.0,
        renew_interval_s: float = RENEW_INTERVAL_S,
    ) -> None:
        super().__init__(store, f"readub:{port}", period_s)
        self._addr = (host, port)
        self._renew_interval_s = renew_interval_s
        self._sock: socket.socket | None = None
        self._registered = False
        self._next_renew = 0.0
        self._last_data_at: float | None = None
        self._packets = 0

    @property
    def packets(self) -> int:
        return self._packets

    def _registration_packet(self) -> bytes:
        # little-endian: byte baixo primeiro (udp_comm.cpp:176-177).
        return struct.pack("<H", REGISTER_EXPIRY_S)

    def _register(self) -> bool:
        """(Re)regista este socket no servidor. ``False`` se falhou."""
        assert self._sock is not None
        try:
            self._sock.sendto(self._registration_packet(), self._addr)
        except OSError as exc:
            LOG.warning("GNSS %s: registo falhou (%s)", self._name, exc)
            self._registered = False
            return False
        self._registered = True
        self._next_renew = time.monotonic() + self._renew_interval_s
        LOG.info("GNSS %s: registado em %s:%d", self._name, self._addr[0], self._addr[1])
        return True

    def _pump(self) -> None:
        if self._sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(self._period_s)
            # Sem bind(): a porta efemera do kernel e' a identidade do registo.
            self._sock = sock
            self._register()

        assert self._sock is not None

        # Re-registo: renovamos periodicamente porque o servidor poda registos
        # expirados, e voltamos a registar se ficar suspiciously tempo sem
        # dados (o Read_uB pode ter reiniciado e perdido a nossa inscricao).
        agora = time.monotonic()
        precisa_renovar = self._registered and agora >= self._next_renew
        precisa_resgistro = not self._registered and (
            self._last_data_at is None
            or (agora - self._last_data_at) > self._renew_interval_s
        )
        if precisa_renovar or precisa_resgistro:
            self._register()

        try:
            raw, _ = self._sock.recvfrom(2048)
        except TimeoutError:
            # Timeout NAO significa registo perdido: o simply nao houve
            # datagrama neste periodo. Nao tocar em _registered aqui.
            return
        except OSError:
            self._sock.close()
            self._sock = None
            self._registered = False
            raise

        self._last_data_at = time.monotonic()
        nav = parse_navdata(raw)
        if nav is None:
            return
        self._packets += 1

        fix = GnssFix(
            latitude=nav.latitude,
            longitude=nav.longitude,
            altitude=nav.altitude,
            fix_ok=nav.llh_valid,
            source=f"readub:{self._addr[1]}:{nav.label or '?'}",
            received_at=time.time(),
        )
        self._publish(fix)

    def _cleanup(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self._registered = False


class MockGnssSource(_BaseSource):
    """Gera uma trajetoria sintetica (apenas com ``--mock-gnss``).

    Percorre uma orbita lenta sobre a regiao do Porto, para que a GS veja
    movimento coerente nos testes de interface.
    """

    def __init__(self, store: GnssStore | None, period_s: float = 0.5) -> None:
        super().__init__(store, "mock", period_s)
        self._t0 = time.monotonic()

    def _pump(self) -> None:
        t = time.monotonic() - self._t0
        # Orbita de ~5 km de raio centrada no Porto,periodo 300 s.
        self._publish(
            GnssFix(
                latitude=41.1579 + 0.045 * math.sin(2 * math.pi * t / 300.0),
                longitude=-8.6291 + 0.060 * math.cos(2 * math.pi * t / 300.0),
                altitude=120.5 + 8.0 * math.sin(2 * math.pi * t / 150.0),
                fix_ok=True,
                source="mock",
                received_at=time.time(),
            )
        )
        self._stop.wait(self._period_s)


# ---------------------------------------------------------------------------
# Orquestrador
# ---------------------------------------------------------------------------
class GnssListener:
    """Arranca as fontes habilitadas e mantem o fix autoritativo.

    A fonte primaria (por omissao a porta 2000, o estado fundido do ``state``)
    alimenta o :class:`GnssStore`. As restantes sao fontes de diagnostico: os
    fixes sao registados mas nao publicam.
    """

    def __init__(
        self,
        *,
        json_port: int = 9000,
        json_enabled: bool = True,
        readub_ports: tuple[int, ...] = (2000,),
        readub_primary_port: int = 2000,
        readub_enabled: bool = True,
        stale_s: float = 5.0,
        mock: bool = False,
        host: str = DEFAULT_HOST,
    ) -> None:
        self.mock = mock
        self._stale_s = stale_s
        self._sources: list[_BaseSource] = []
        self._stopped = False
        # Atribuido logo abaixo em todos os caminhos; declarado aqui para que o
        # tipo nao dependa do primeiro `=` que o mypy encontrar.
        self._store: GnssStore

        primary_label = f"readub:{readub_primary_port}"
        if mock:
            self._store = GnssStore(source="mock", stale_s=stale_s)
            self._sources.append(MockGnssSource(self._store))
            LOG.warning("GNSS: MODO MOCK ativo (--mock-gnss). Posicao ARTIFICIAL.")
            return

        self._store = GnssStore(source=primary_label, stale_s=stale_s)

        if readub_enabled:
            for porta in readub_ports:
                store_here = self._store if porta == readub_primary_port else None
                self._sources.append(
                    ReadUbGnssSource(store_here, port=porta, host=host)
                )
            LOG.info(
                "GNSS: fontes Read_uB %s (primaria %s)",
                list(readub_ports),
                readub_primary_port,
            )
        if json_enabled:
            # A fonte JSON e primaria apenas se nao houver Read_uB configurado.
            store_json = None if readub_enabled else self._store
            self._sources.append(
                JsonGnssSource(store_json, host=host, port=json_port)
            )

        if not self._sources:
            LOG.warning(
                "GNSS: nenhuma fonte habilitada; o TelemetryFrame sera publicado "
                "com fix_ok=False."
            )

    @property
    def store(self) -> GnssStore:
        return self._store

    def start(self) -> None:
        for fonte in self._sources:
            fonte.start()

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        for fonte in self._sources:
            try:
                fonte.stop()
            except Exception as exc:  # pragma: no cover
                LOG.warning("GNSS: erro a parar %s: %s", fonte.name, exc)

    def snapshot(self) -> GnssFix | None:
        """Fix autoritativo actual, ou ``None`` se indisponivel/expirado."""
        return self._store.snapshot()


def bind_gnss_probe(port: int, *, host: str = "0.0.0.0") -> socket.socket:
    """Cria um socket UDP de escuta para uso de diagnostico.

    Util para confirmar que o feed JSON esta a emitir antes de arrancar o
    servidor.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    return sock


def env_port(name: str, default: int) -> int:
    """Le uma porta do ambiente (util em scripts de teste)."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        return default
