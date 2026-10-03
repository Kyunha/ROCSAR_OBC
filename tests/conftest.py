"""Fixtures partilhadas e construtores de datagramas de teste.

O conftest expoe :func:`build_navdata`, que monta um ``struct NavData`` de 420
bytes *exatamente* como o ``Read_uB`` o emite (ver ``Read_uB/nav_data.h:52`` e o
DWARF de ``Read_uB.o``). Os testes usam-no para validar o parser contra bytes
reais em vez de contra mocks.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src import rocsar_messages_pb2 as pb  # noqa: E402
from src.gnss_listener import (  # noqa: E402
    NAVDATA_SIZE,
    OFF_FLAGS,
    OFF_LATITUDE,
    OFF_NAV_NSEC,
    OFF_NAV_SEC,
)


# ---------------------------------------------------------------------------
# Construtores de NavData
# ---------------------------------------------------------------------------
def build_navdata(
    *,
    latitude: float = 41.1579,
    longitude: float = -8.6291,
    altitude: float = 120.5,
    flags: int = 0x0001,
    stage: int = 2,
    label: bytes = b"Straplex",
    size: int = NAVDATA_SIZE,
) -> bytes:
    """Monta um datagrama ``NavData`` de ``size`` bytes.

    Reproduz ``#pragma pack(1)``, little-endian, com os offsets reais do
    ``struct NavData``: label@0, NAV_time@8/16, lat@24, lon@32, alt@40,
    flags@120, stage@122.
    """
    buf = bytearray(size)
    buf[0:min(len(label), 8)] = label[:8]
    struct.pack_into("<q", buf, OFF_NAV_SEC, 1_750_000_000)
    struct.pack_into("<q", buf, OFF_NAV_NSEC, 123_456_789)
    struct.pack_into("<ddd", buf, OFF_LATITUDE, latitude, longitude, altitude)
    struct.pack_into("<HH", buf, OFF_FLAGS, flags, stage)
    return bytes(buf)


def build_team_v2(
    *,
    heading: float = 123.45,
    target: float = 180.0,
    imu_ok: int = 1,
    servos: tuple[tuple[int, int, int, int, float, int, int], ...] | None = None,
) -> str:
    """Constroi uma linha ``TELEM`` v2 tal como a Pico a emite.

    Cada servo e ``(id, tick, speed, load, volt, temp, online)``.
    """
    if servos is None:
        servos = (
            (1, 2048, -15, 42, 12.1, 34, 1),
            (2, 2051, 12, 40, 12.0, 35, 1),
        )
    partes = [f"TELEM,{heading:.2f},{target:.2f},{imu_ok},{len(servos)}"]
    for sid, tick, spd, load, volt, temp, on in servos:
        partes.append(f"{sid},{tick},{spd},{load},{volt:.1f},{temp},{on}")
    return ",".join(partes)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture
def tmp_data_dir(tmp_path: Path) -> Path:
    """Directorio de dados isolado por teste."""
    d = tmp_path / "rocsar_data"
    d.mkdir()
    return d


@pytest.fixture
def proto():
    return pb


@pytest.fixture
def navdata_builder():
    return build_navdata


@pytest.fixture
def telem_builder():
    return build_team_v2


def free_port() -> int:
    """Reserva uma porta TCP/UDP livre e devolve o numero."""
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def free_udp_port() -> int:
    """Reserva uma porta UDP livre e devolve o numero."""
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def wait_until(predicate, timeout_s: float = 2.0, period_s: float = 0.01) -> bool:
    """Espera ate ``predicate()`` ser verdade. Devolve True se tiver sucesso.

    Necessario porque os comandos sao assincronos em relacao ao snapshot: o
    snapshot de telemetria e derivado do que o hardware reportou no ultimo
    ``STATUS``, ou seja, no maximo um ciclo de poll depois do comando.
    """
    import time

    limite = time.monotonic() + timeout_s
    while time.monotonic() < limite:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()
