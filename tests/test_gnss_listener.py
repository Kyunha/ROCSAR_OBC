"""Testes da ingestao GNSS: parsing, registo no servidor Read_uB e staleness.

O parser binario e validado contra bytes construidos a mao com os offsets
reais do ``struct NavData`` (``Read_uB/nav_data.h:52``). O registo e validado
contra um servidor UDP falso que reproduz o comportamento de
``UDP_Comm_Server``: so envia para quem se registou, e poda registos expirados.
"""

from __future__ import annotations

import contextlib
import json
import socket
import struct
import threading
import time

import pytest
from conftest import build_navdata, free_udp_port, wait_until

from src.gnss_listener import (
    LLH_VALID,
    NAVDATA_SIZE,
    GnssListener,
    GnssStore,
    JsonGnssSource,
    MockGnssSource,
    ReadUbGnssSource,
    parse_gnss_json,
    parse_navdata,
)


# ===========================================================================
# Parsing do NavData binario
# ===========================================================================
class TestParseNavData:
    def test_campos_nos_offsets_correctos(self) -> None:
        """Confere que o le os bytes nos offsets exactos do struct."""
        nav = parse_navdata(build_navdata(
            latitude=41.1579, longitude=-8.6291, altitude=120.5,
            flags=0x0001, stage=2, label=b"Straplex",
        ))
        assert nav is not None
        assert nav.label == "Straplex"
        assert nav.latitude == pytest.approx(41.1579)
        assert nav.longitude == pytest.approx(-8.6291)
        assert nav.altitude == pytest.approx(120.5)
        assert nav.flags == 0x0001
        assert nav.stage == 2
        assert nav.llh_valid is True

    def test_llh_valid_bit(self) -> None:
        assert parse_navdata(build_navdata(flags=LLH_VALID)).llh_valid is True
        assert parse_navdata(build_navdata(flags=0x0000)).llh_valid is False
        # Bits vizinhos a LLH_VALID nao podem fazer o bit 0 virar.
        assert parse_navdata(build_navdata(flags=0x0002)).llh_valid is False
        assert parse_navdata(build_navdata(flags=0x0006)).llh_valid is False

    def test_label_com_padding_nulo(self) -> None:
        assert parse_navdata(build_navdata(label=b"tty00")).label == "tty00"

    def test_label_exatamente_8_chars_sem_nulo(self) -> None:
        # "Straplex" tem 8 chars e nao e terminado por NUL (state.cpp:384).
        nav = parse_navdata(build_navdata(label=b"Straplex"))
        assert nav is not None and nav.label == "Straplex"

    def test_tamanho_incorrecto_rejeitado(self) -> None:
        bruto = build_navdata()
        assert len(bruto) == NAVDATA_SIZE
        assert parse_navdata(bruto[:NAVDATA_SIZE - 1]) is None
        assert parse_navdata(bruto[:10]) is None
        assert parse_navdata(b"") is None
        # Mais bytes que o struct: nao e um NavData nosso.
        assert parse_navdata(bruto + b"\x00" * 4) is None

    @pytest.mark.parametrize("lat", [90.1, -90.1, 999.0, -1e9])
    def test_latitude_fora_de_range(self, lat: float) -> None:
        assert parse_navdata(build_navdata(latitude=lat)) is None

    @pytest.mark.parametrize("lon", [180.1, -180.1, 1e9])
    def test_longitude_fora_de_range(self, lon: float) -> None:
        assert parse_navdata(build_navdata(longitude=lon)) is None

    def test_limites_geometricos_aceites(self) -> None:
        assert parse_navdata(build_navdata(latitude=90.0, longitude=180.0)) is not None
        assert parse_navdata(build_navdata(latitude=-90.0, longitude=-180.0)) is not None

    def test_nan_rejeitado(self) -> None:
        assert parse_navdata(build_navdata(latitude=float("nan"))) is None
        assert parse_navdata(build_navdata(altitude=float("nan"))) is None
        assert parse_navdata(build_navdata(altitude=float("inf"))) is None

    def test_altitude_negativa_aceite(self) -> None:
        # Abaixo do nivel do mar e valido; so recusamos valores absurdos.
        assert parse_navdata(build_navdata(altitude=-120.0)) is not None

    def test_endianness_invertido_rejeitado(self) -> None:
        """Big-endian produz numeros absurdos -> tem de ser recusado."""
        buf = bytearray(NAVDATA_SIZE)
        buf[0:8] = b"Straplex"
        struct.pack_into(">ddd", buf, 24, 41.1579, -8.6291, 120.5)
        struct.pack_into(">HH", buf, 120, 1, 2)
        assert parse_navdata(bytes(buf)) is None, "payload big-endian nao pode passar"


# ===========================================================================
# Parsing do feed JSON
# ===========================================================================
class TestParseGnssJson:
    def test_formato_da_spec(self) -> None:
        fix = parse_gnss_json(b'{"lat": 41.1579, "lon": -8.6291, "alt": 120.5}')
        assert fix is not None
        assert fix.latitude == pytest.approx(41.1579)
        assert fix.longitude == pytest.approx(-8.6291)
        assert fix.altitude == pytest.approx(120.5)
        assert fix.fix_ok is True

    def test_nomes_alternativos(self) -> None:
        fix = parse_gnss_json('{"latitude": 1.0, "longitude": 2.0, "altitude": 3.0}')
        assert fix is not None
        assert (fix.latitude, fix.longitude, fix.altitude) == (1.0, 2.0, 3.0)

    def test_altura_ausente_vezes_zero(self) -> None:
        fix = parse_gnss_json('{"lat": 1.0, "lon": 2.0}')
        assert fix is not None and fix.altitude == 0.0

    def test_envelope_aninhado(self) -> None:
        fix = parse_gnss_json('{"type":"NAV","data":{"lat":41.0,"lon":-8.0,"alt":900}}')
        assert fix is not None and fix.altitude == pytest.approx(900.0)

    def test_lista_toma_o_ultimo_objeto(self) -> None:
        fix = parse_gnss_json('[{"x":1},{"lat":41.0,"lon":-8.0}]')
        assert fix is not None and fix.latitude == pytest.approx(41.0)

    @pytest.mark.parametrize("valor,esperado", [
        (True, True), (False, False),
        (1, True), (0, False),
        ("true", True), ("false", False),
        ("1", True), ("0", False),
    ])
    def test_flags_de_fix(self, valor: object, esperado: bool) -> None:
        fix = parse_gnss_json(json.dumps({"lat": 1.0, "lon": 2.0, "fix_ok": valor}))
        assert fix is not None and fix.fix_ok is esperado

    @pytest.mark.parametrize("corpo", [
        "", "   ", "nao e json", "{", "[]", "[1,2]", "null", "42",
        '{"x": 1}',                       # sem lat/lon
        '{"lat": "abc", "lon": 1}',      # nao numerico
        '{"lat": 91, "lon": 0}',         # fora de range
        '{"lat": 0, "lon": 181}',        # fora de range
    ])
    def test_rejeitados(self, corpo: str) -> None:
        assert parse_gnss_json(corpo) is None

    def test_bytes_nao_utf8(self) -> None:
        assert parse_gnss_json(b"\xff\xfe\x00nao-utf8") is None


# ===========================================================================
# Servidor UDP falso que imita o UDP_Comm_Server
# ===========================================================================
class FakeReadUbServer:
    """Reproduz ``UDP_Comm_Server``: registo + push so para clientes registados.

    Como o C++ real, so entrega datagramas a quem enviou o pacote de registo de
    2 bytes little-endian.
    """

    def __init__(self, *, expire_s: float = 125.0, require_registration: bool = True) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]
        self.require_registration = require_registration
        self._clients: dict[tuple[str, int], float] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.sent = 0
        self.registrations = 0
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                data, src = self.sock.recvfrom(2048)
            except TimeoutError:
                self._prune()
                continue
            except OSError:
                break
            with self._lock:
                # b'' -> default; caso contrario, 2 bytes little-endian.
                expiry = (data[0] + 256 * data[1]) if len(data) >= 2 else self.expire_s
                self._clients[src] = time.monotonic() + expiry
                self.registrations += 1
                self._deliver_locked()
        self.sock.close()

    def _prune_locked(self) -> None:
        """Podar expirados. Exige a lock ja tomada (Lock nao e' reentrante)."""
        agora = time.monotonic()
        for addr, exp in list(self._clients.items()):
            if exp < agora:
                del self._clients[addr]

    def _prune(self) -> None:
        with self._lock:
            self._prune_locked()

    def _send_locked(self, payload: bytes) -> None:
        """Envia a todos os clientes registados. Exige a lock ja tomada."""
        self._prune_locked()
        for addr in list(self._clients):
            with contextlib.suppress(OSError):
                self.sock.sendto(payload, addr)

    def _deliver_locked(self) -> None:
        self._prune_locked()
        if not self.require_registration or self._clients:
            self._send_locked(build_navdata())
            self.sent += 1

    def push(self, payload: bytes) -> None:
        with self._lock:
            self._send_locked(payload)

    def client_count(self) -> int:
        with self._lock:
            self._prune_locked()
            return len(self._clients)

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


# ===========================================================================
# Fonte Read_uB: registo e recepcao
# ===========================================================================
class TestReadUbSource:
    def test_regista_e_recebe(self) -> None:
        server = FakeReadUbServer()
        try:
            store = GnssStore(source="readub", stale_s=5.0)
            src = ReadUbGnssSource(store, port=server.port, period_s=0.1)
            src.start()
            try:
                assert wait_until(lambda: store.update_count > 0, timeout_s=5.0), \
                    "nenhum NavData recebido"
                fix = store.snapshot()
                assert fix is not None
                assert fix.latitude == pytest.approx(41.1579)
                assert fix.longitude == pytest.approx(-8.6291)
                assert fix.altitude == pytest.approx(120.5)
                assert fix.fix_ok is True
                assert "readub" in fix.source
            finally:
                src.stop()
        finally:
            server.stop()

    def test_registro_e_little_endian(self) -> None:
        """O pacote de registo tem de ser 2 bytes little-endian.

        Se fosse big-endian, 125 seria 0x7D00 = 32000 s e o servidor C++
        interpretaria outro valor.
        """
        server = FakeReadUbServer()
        try:
            store = GnssStore(source="readub", stale_s=5.0)
            src = ReadUbGnssSource(store, port=server.port, period_s=0.1)
            assert src._registration_packet() == struct.pack("<H", 125)
            assert src._registration_packet() == b"\x7d\x00"
            src.start()
            try:
                assert wait_until(lambda: server.registrations >= 1, timeout_s=3.0)
            finally:
                src.stop()
        finally:
            server.stop()

    def test_sem_registro_nao_chega_nada(self) -> None:
        """Confirma que o registo e mesmo obrigatorio."""
        server = FakeReadUbServer(require_registration=True)
        try:
            # Socket cru, sem registo: nao deve receber nada.
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.6)
            try:
                server.push(build_navdata())
                with pytest.raises((socket.timeout, TimeoutError)):
                    s.recvfrom(2048)
            finally:
                s.close()
            assert server.client_count() == 0
        finally:
            server.stop()

    def test_renova_registro(self) -> None:
        """A renovacao mantem o registo vivo e volta a contar."""
        server = FakeReadUbServer()
        try:
            store = GnssStore(source="readub", stale_s=5.0)
            src = ReadUbGnssSource(store, port=server.port, period_s=0.05,
                                  renew_interval_s=0.2)
            src.start()
            try:
                assert wait_until(lambda: store.update_count > 3, timeout_s=5.0)
                assert server.client_count() >= 1, "o cliente devia manter-se registado"
                assert server.registrations >= 2, "devia ter renovado"
            finally:
                src.stop()
        finally:
            server.stop()

    def test_ignora_payloads_de_tamanho_errado(self) -> None:
        server = FakeReadUbServer()
        try:
            store = GnssStore(source="readub", stale_s=5.0)
            src = ReadUbGnssSource(store, port=server.port, period_s=0.1)
            src.start()
            try:
                # O servidor tambem empurra um NavData valido no registo, por
                # isso medimos o delta e nao o total.
                assert wait_until(lambda: store.update_count >= 1, timeout_s=3.0)
                base = store.update_count
                for payload in (b"lixo", b"", b"\x00" * 100,
                                b"\x00" * (NAVDATA_SIZE - 1),
                                b"\x00" * (NAVDATA_SIZE + 1)):
                    server.push(payload)
                    time.sleep(0.05)
                assert store.update_count == base, (
                    "payloads de tamanho invalido nao podem publicar"
                )
            finally:
                src.stop()
        finally:
            server.stop()


# ===========================================================================
# Fonte JSON
# ===========================================================================
class TestJsonSource:
    def test_recebe_feed_json(self) -> None:
        porta = free_udp_port()
        store = GnssStore(source="json", stale_s=5.0)
        src = JsonGnssSource(store, port=porta, period_s=0.1)
        src.start()
        try:
            assert wait_until(lambda: store.update_count >= 0)
            emissor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                # O emissor envia para a porta que o OBC escuta.
                for _ in range(5):
                    emissor.sendto(
                        json.dumps({"lat": 41.1579, "lon": -8.6291, "alt": 120.5}).encode(),
                        ("127.0.0.1", porta),
                    )
                    if store.update_count:
                        break
                    time.sleep(0.1)
            finally:
                emissor.close()
            fix = store.snapshot()
            assert fix is not None
            assert fix.latitude == pytest.approx(41.1579)
            assert "json" in fix.source
        finally:
            src.stop()


# ===========================================================================
# GnssStore: staleness
# ===========================================================================
class TestGnssStore:
    def test_sem_dados_devolve_none(self) -> None:
        store = GnssStore(source="x", stale_s=5.0)
        assert store.snapshot() is None

    def test_fix_expirado_devolve_none(self) -> None:
        from src.gnss_listener import GnssFix

        store = GnssStore(source="x", stale_s=0.2)
        store.update(GnssFix(1.0, 2.0, 3.0, True, "x", received_at=time.time() - 5))
        assert store.snapshot() is None, "fix de 5 s atras esta expirado com stale_s=0.2"

    def test_fix_recem_devolve_valor(self) -> None:
        from src.gnss_listener import GnssFix

        store = GnssStore(source="x", stale_s=5.0)
        store.update(GnssFix(1.0, 2.0, 3.0, True, "x", received_at=time.time()))
        assert store.snapshot() is not None


# ===========================================================================
# Mock e degradação
# ===========================================================================
class TestMockEdegradado:
    def test_mock_gera_posicao(self) -> None:
        store = GnssStore(source="mock", stale_s=5.0)
        src = MockGnssSource(store, period_s=0.05)
        src.start()
        try:
            assert wait_until(lambda: store.update_count > 0)
            fix = store.snapshot()
            assert fix is not None
            assert -90 <= fix.latitude <= 90
            assert -180 <= fix.longitude <= 180
            assert fix.fix_ok is True
            assert fix.source == "mock"
        finally:
            src.stop()

    def test_sem_fontes_nao_ha_fix(self) -> None:
        """Desligar todas as fontes tem de dar fix_ok=False, nunca invencao."""
        listener = GnssListener(json_enabled=False, readub_enabled=False)
        listener.start()
        try:
            time.sleep(0.2)
            assert listener.snapshot() is None
        finally:
            listener.stop()


# ===========================================================================
# Orquestrador
# ===========================================================================
class TestGnssListener:
    def test_porta_primaria_alimenta_o_store(self) -> None:
        server = FakeReadUbServer()
        try:
            listener = GnssListener(
                json_enabled=False,
                readub_ports=(server.port,),
                readub_primary_port=server.port,
                stale_s=5.0,
            )
            listener.start()
            try:
                assert wait_until(lambda: listener.snapshot() is not None, timeout_s=5.0)
            finally:
                listener.stop()
        finally:
            server.stop()

    def test_mock_nao_precisa_de_fontes(self) -> None:
        listener = GnssListener(readub_enabled=False, json_enabled=False, mock=True)
        listener.start()
        try:
            assert wait_until(lambda: listener.snapshot() is not None, timeout_s=3.0)
            assert listener.snapshot().source == "mock"
        finally:
            listener.stop()

    def test_stop_e_idempotente(self) -> None:
        listener = GnssListener(readub_enabled=False, json_enabled=False, mock=True)
        listener.start()
        listener.stop()
        listener.stop()  # nao deve levantar
