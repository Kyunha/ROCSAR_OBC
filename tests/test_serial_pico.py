"""Testes do parser e dos modos de operacao da interface com a Pico.

Cobre os dois formatos de wire ``TELEM`` (v2 completo e v1 legado), a
rejeicao de linhas corrompidas e os tres modos de operacao: real, degradado e
mock.
"""

from __future__ import annotations

import threading

import pytest
from conftest import build_team_v2, wait_until

from src.serial_pico import (
    MAX_TICK,
    MIN_TICK,
    NUM_SERVOS,
    PicoController,
    PicoError,
    TelemetrySnapshot,
    parse_telem,
)


# ---------------------------------------------------------------------------
# TELEM v2 (formato actual, dois servos completos)
# ---------------------------------------------------------------------------
class TestTelemV2:
    def test_parse_dois_servos_completos(self) -> None:
        linha = build_team_v2()
        r = parse_telem(linha)
        assert r is not None, f"linha rejeitada: {linha!r}"
        heading, target, imu_ok, servos = r

        assert heading == pytest.approx(123.45)
        assert target == pytest.approx(180.0)
        assert imu_ok is True
        assert len(servos) == NUM_SERVOS

        s1, s2 = servos
        assert (s1.id, s1.current_tick, s1.current_speed, s1.current_load) == (1, 2048, -15, 42)
        assert s1.voltage_v == pytest.approx(12.1)
        assert s1.temperature_c == 34
        assert s1.online is True
        assert (s2.id, s2.current_tick, s2.current_speed, s2.current_load) == (2, 2051, 12, 40)
        assert s2.online is True

    def test_parse_um_servo(self) -> None:
        r = parse_telem(build_team_v2(servos=((1, 100, 5, 6, 12.5, 30, 1),)))
        assert r is not None
        assert len(r[3]) == 1
        assert r[3][0].current_tick == 100

    def test_zero_servos_e_valido(self) -> None:
        r = parse_telem(build_team_v2(servos=()))
        assert r is not None
        assert r[3] == ()
        assert r[0] == pytest.approx(123.45)

    def test_contagem_incoerente_rejeitada(self) -> None:
        # Diz 2 servos mas so traz os campos de 1.
        r = parse_telem("TELEM,1.0,2.0,1,2,1,2048,0,0,12.0,30,1")
        assert r is None

    def test_n_servos_fora_de_intervalo_rejeitada(self) -> None:
        assert parse_telem("TELEM,1.0,2.0,1,99,1,2048,0,0,12.0,30,1") is None

    def test_campos_de_servo_nao_numericos_rejeitada(self) -> None:
        assert parse_telem("TELEM,1.0,2.0,1,1,1,abc,0,0,12.0,30,1") is None

    def test_imu_offline(self) -> None:
        r = parse_telem(build_team_v2(imu_ok=0))
        assert r is not None
        assert r[2] is False

    def test_servo_offline_marcado(self) -> None:
        r = parse_telem(build_team_v2(servos=((1, 2048, 0, 0, 0.0, 0, 0),)))
        assert r is not None
        assert r[3][0].online is False

    def test_heading_negativo(self) -> None:
        r = parse_telem(build_team_v2(heading=-5.5, target=720.25))
        assert r is not None
        assert r[0] == pytest.approx(-5.5)
        assert r[1] == pytest.approx(720.25)


# ---------------------------------------------------------------------------
# TELEM v1 legado (formato da spec original)
# ---------------------------------------------------------------------------
class TestTelemV1Legado:
    LINHA = "TELEM,45.50,90.00,1,2048,12.1,34,12.0,35"

    def test_parse_legado(self) -> None:
        r = parse_telem(self.LINHA)
        assert r is not None, f"formato v1 rejeitado: {self.LINHA!r}"
        heading, target, imu_ok, servos = r

        assert heading == pytest.approx(45.50)
        assert target == pytest.approx(90.00)
        assert imu_ok is True
        assert len(servos) == 2

    def test_servo1_com_tick_conhecido(self) -> None:
        s1 = parse_telem(self.LINHA)[3][0]
        assert s1.current_tick == 2048
        assert s1.voltage_v == pytest.approx(12.1)
        assert s1.temperature_c == 34
        assert s1.online is True

    def test_servo2_sem_tick_por_desenho(self) -> None:
        # A Pico antiga so emitia tensao/temperatura do servo 2.
        s2 = parse_telem(self.LINHA)[3][1]
        assert s2.id == 2
        assert s2.voltage_v == pytest.approx(12.0)
        assert s2.temperature_c == 35
        assert s2.current_tick == 0, "v1 nao transporta tick2; tem de ficar a 0"
        assert s2.current_speed == 0
        assert s2.current_load == 0

    def test_ambos_os_formatos(self) -> None:
        assert parse_telem(self.LINHA) is not None
        assert parse_telem(build_team_v2()) is not None


# ---------------------------------------------------------------------------
# Rejeicoes
# ---------------------------------------------------------------------------
class TestParseRejeicoes:
    @pytest.mark.parametrize(
        "linha",
        [
            "",
            "   ",
            "OK: TARGET_SET",
            "OK: HEAT1",
            "OK: JOG",
            "ERR: UNKNOWN_CMD",
            "TELEM",
            "TELEM,",
            "TELEM,x,y,1",
            "TELEM,1,2",
            "TELEM,1,2,1",
            "TELEM,1,2,1,2,1,2048",          # payload incompleto
            "telem,1,2,1,2048,12.1,34,12.0",  # 8 tokens: nem v1 nem v2
            "algarismo,1,2,1,2048,12.1,34,12.0,35",
        ],
    )
    def test_devolve_none(self, linha: str) -> None:
        assert parse_telem(linha) is None, f"deveria rejeitar: {linha!r}"

    def test_acks_nao_sao_telem(self) -> None:
        for ack in ("OK: TARGET_SET", "OK: HEAT1", "OK: HEAT2", "OK: JOG"):
            assert parse_telem(ack) is None


# ---------------------------------------------------------------------------
# Modos de operacao
# ---------------------------------------------------------------------------
class TestModos:
    def test_mock_nao_fabrica_nada_sem_flag(self, tmp_path) -> None:
        """Sem --mock-pico e sem device: degradado, nao telemetria inventada."""
        c = PicoController(port="/dev/ttyACM_INEXISTENTE", baud=115200, poll_hz=20)
        assert c.mock is False, "mock so com flag explicita"

        c.start()
        try:
            snap = c.snapshot()
            assert snap.link_up is False
            assert snap.imu_online is False
            assert all(not s.online for s in snap.servos)
            assert snap.gondola_heading == 0.0, "degradado nao inventa heading"
        finally:
            c.stop()

    def test_mock_com_flag(self) -> None:
        c = PicoController(port="/dev/ttyACM_INEXISTENTE", baud=115200,
                           poll_hz=20, mock=True)
        assert c.mock is True
        c.start()
        try:
            c.set_target(90.0)
            # Da tempo ao poller para produzir pelo menos um snapshot.
            deadline = threading.Event()
            deadline.wait(0.4)
            snap = c.snapshot()
            assert snap.link_up is True
            assert snap.imu_online is True
            assert snap.target_heading == pytest.approx(90.0)
            assert len(snap.servos) == NUM_SERVOS
            assert all(s.online for s in snap.servos)
            assert 0.0 <= snap.gondola_heading < 360.0
        finally:
            c.stop()

    def test_snapshot_offline(self) -> None:
        snap = TelemetrySnapshot.offline()
        assert snap.link_up is False
        assert snap.imu_online is False
        assert len(snap.servos) == NUM_SERVOS
        assert all(not s.online for s in snap.servos)


# ---------------------------------------------------------------------------
# Validacao de comandos
# ---------------------------------------------------------------------------
class TestValidacaoComandos:
    @pytest.fixture
    def ctl(self):
        c = PicoController(port="/dev/ttyACM_INEXISTENTE", baud=115200, mock=True)
        c.start()
        yield c
        c.stop()

    def test_target_normalizado(self, ctl) -> None:
        ctl.set_target(370.0)
        assert wait_until(lambda: ctl.snapshot().target_heading == pytest.approx(10.0))
        ctl.set_target(-90.0)
        assert wait_until(lambda: ctl.snapshot().target_heading == pytest.approx(270.0))

    @pytest.mark.parametrize("heater", [0, 3, 99])
    def test_heater_id_invalido(self, ctl, heater: int) -> None:
        with pytest.raises(PicoError, match="heater_id invalido"):
            ctl.set_heater(heater, True)

    @pytest.mark.parametrize("servo", [0, 3])
    def test_servo_id_invalido(self, ctl, servo: int) -> None:
        with pytest.raises(PicoError, match="servo_id invalido"):
            ctl.jog_servo(servo, 100)

    @pytest.mark.parametrize("tick", [MIN_TICK - 1, MAX_TICK + 1, 100000])
    def test_tick_fora_de_intervalo(self, ctl, tick: int) -> None:
        with pytest.raises(PicoError, match="target_tick invalido"):
            ctl.jog_servo(1, tick)

    @pytest.mark.parametrize("tick", [MIN_TICK, 0, 2048, MAX_TICK])
    def test_tick_valido(self, ctl, tick: int) -> None:
        ctl.jog_servo(1, tick)  # nao deve levantar

    def test_heaters_refletidos_no_snapshot(self, ctl) -> None:
        ctl.set_heater(1, True)
        ctl.set_heater(2, True)
        assert wait_until(lambda: ctl.snapshot().heater1_active is True)
        snap = ctl.snapshot()
        assert snap.heater1_active is True
        assert snap.heater2_active is True

    def test_jog_mantem_tick_no_mock(self, ctl) -> None:
        ctl.jog_servo(2, 3000)
        assert wait_until(
            lambda: next(s for s in ctl.snapshot().servos if s.id == 2).current_tick == 3000
        ), "modo manual tem de persistir o tick pedido"
        snap = ctl.snapshot()
        s2 = next(s for s in snap.servos if s.id == 2)
        assert s2.current_tick == 3000, "modo manual tem de persistir o tick pedido"

    def test_comando_com_pico_ausente_falha(self) -> None:
        c = PicoController(port="/dev/ttyACM_INEXISTENTE", baud=115200)
        c.start()
        try:
            with pytest.raises(PicoError):
                c.set_target(45.0)
        finally:
            c.stop()


# ---------------------------------------------------------------------------
# Proto
# ---------------------------------------------------------------------------
class TestServoStatusProto:
    def test_to_proto_preenche_todos_os_campos(self) -> None:
        from src.serial_pico import ServoStatus

        s = ServoStatus(id=2, current_tick=3000, current_speed=-7,
                        current_load=91, voltage_v=11.8, temperature_c=41, online=True)
        p = s.to_proto()
        assert (p.id, p.current_tick, p.current_speed) == (2, 3000, -7)
        assert (p.current_load, p.temperature_c) == (91, 41)
        # voltage_v e' float32 no proto: 11.8 nao e exacto em IEEE-754.
        assert p.voltage_v == pytest.approx(11.8, abs=1e-5)
        assert p.online is True
