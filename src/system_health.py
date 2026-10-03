"""Saude do sistema OBC: temperatura da CPU, uptime e ocupacao do disco.

Todos os valores sao *best effort*: numa Raspberry Pi real `/sys/class/thermal`
normalmente existe, mas em contentores, macOS ou maquinas de desenvolvimento nao.
Por isso cada leitor degrada para ``None`` em vez de falhar, e quem chama decide
o que fazer.
"""

from __future__ import annotations

import logging
import shutil
import threading
import time
from pathlib import Path

from . import rocsar_messages_pb2 as pb

LOG = logging.getLogger(__name__)

#: Leitor de temperatura mais comum em Raspberry Pi OS.
_THERMAL_ZONE = Path("/sys/class/thermal/thermal_zone0/temp")

#: Segundos que se espera razoavelmente entre leituras de saude (a 5 Hz nao ha
#: ganho em ler o disco 10x por segundo).
_INTERVAL_S = 2.0


def read_cpu_temp_c() -> float | None:
    """Temperatura da CPU em graus Celsius, ou ``None`` se indisponivel."""
    try:
        bruto = _THERMAL_ZONE.read_text().strip()
        # O sysfs expoe milikelvin: 48000 == 48.0 C
        return float(bruto) / 1000.0
    except (OSError, ValueError):
        LOG.debug("Temperatura da CPU indisponivel em %s", _THERMAL_ZONE)
        return None


def read_uptime_seconds() -> int:
    """Uptime do sistema em segundos.

    Tenta ``/proc/uptime``; em alternativa usa o uptime do proprio processo,
    que e sempre valido.
    """
    try:
        bruto = Path("/proc/uptime").read_text().split()[0]
        return int(float(bruto))
    except (OSError, ValueError, IndexError):
        return int(time.monotonic())


def read_disk_used_percent(path: Path) -> float:
    """Percentagem de ocupacao do filesystem onde vive ``path``.

    Se o path nao existir devolve 0.0 -- nao vale a pena abortar o loop de
    telemetria por causa de um disco em modo de leitura.
    """
    try:
        usage = shutil.disk_usage(path)
    except OSError as exc:
        LOG.debug("disk_usage(%s) falhou: %s", path, exc)
        return 0.0
    if usage.total <= 0:
        return 0.0
    return 100.0 * usage.used / usage.total


class SystemHealthMonitor:
    """Amostra a saude do sistema com cache, para nao martelar o filesystem.

    A telemetria corre a 5 Hz mas o disco e a temperatura nao precisam dessa
    resolucao; o monitor limita-se a reamostrar de ``_INTERVAL_S`` em
    ``_INTERVAL_S``.
    """

    def __init__(self, data_dir: Path, *, interval_s: float = _INTERVAL_S) -> None:
        self._data_dir = data_dir
        self._interval_s = interval_s
        self._lock = threading.Lock()
        self._next_sample = 0.0
        self._cpu_temp: float | None = None
        self._disk_used = 0.0
        self._uptime = 0

    def _resample_if_due(self) -> None:
        agora = time.monotonic()
        with self._lock:
            if agora < self._next_sample:
                return
            self._next_sample = agora + self._interval_s

        # Fora da lock: a leitura pode bloquear milissegundos.
        cpu = read_cpu_temp_c()
        disco = read_disk_used_percent(self._data_dir)
        uptime = read_uptime_seconds()

        with self._lock:
            if cpu is not None:
                self._cpu_temp = cpu
            self._disk_used = disco
            self._uptime = uptime

    def fill(self, health: pb.SystemHealth, *, limit_active: bool) -> pb.SystemHealth:
        """Preenche ``health`` in-place com a leitura mais recente."""
        self._resample_if_due()
        with self._lock:
            # proto3 nao distingue 0 de ausente; se nao ha sensor de
            # temperatura, reportamos 0.0 e a GS sabe que e desconhecido.
            health.cpu_temp_c = self._cpu_temp if self._cpu_temp is not None else 0.0
            health.uptime_seconds = self._uptime
            health.limit_active = limit_active
            health.disk_used_percent = self._disk_used
        return health
