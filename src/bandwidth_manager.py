"""Gestao dinamica de largura de banda da ligacao eterea (Linux ``tc``).

A limitacao e feita com um filtro TBF (*Token Bucket Filter*) na interface de
saida. O comando exige privilegio de root, que numa OBC obtemos via ``sudo``::

    sudo tc qdisc del dev eth0 root            # limpa a regra anterior
    sudo tc qdisc add dev eth0 root tbf rate 512kbit burst 32kbit latency 400ms

Filosofia
---------
Este modulo **nunca levanta excecao**. ``tc`` pode falhar por dezenas de razoes
nao relatedas com a logica -- ``sudo`` sem password, ``tc`` ausente, interface
errada, kernel sem suporte -- e nenhuma delas justifica derrubar o servidor de
telemetria. Todas devolvem ``(False, motivo)`` e o chamador traduz isso para um
``CommandResponse`` com ``success=False`` e a mensagem de erro real.
"""

from __future__ import annotations

import logging
import shutil
import subprocess

LOG = logging.getLogger(__name__)

#: Timeout por comando. ``tc`` responde em milissegundos; 5 s e folgado e evita
#: que um ``sudo`` a pedir password fique bloqueado para sempre.
COMMAND_TIMEOUT_S = 5.0


class BandwidthManager:
    """Aplica e remove limites de banda com ``tc``.

    Parametros
    ----------
    iface:
        Interface de rede a limitar (por omissao ``eth0``).
    sudo_prefix:
        Prefixo de privilegio, ex. ``("sudo",)``, ``("doas", "tc")`` ou ``()``
        se o processo ja corre como root. Vazio desliga a gestao.
    burst / latency:
        Parametros do bucket TBF, tal como especificado na arquitectura.
    """

    def __init__(
        self,
        *,
        iface: str = "eth0",
        sudo_prefix: tuple[str, ...] = ("sudo",),
        burst: str = "32kbit",
        latency: str = "400ms",
        timeout_s: float = COMMAND_TIMEOUT_S,
        enabled: bool = True,
    ) -> None:
        self.iface = iface
        self.sudo_prefix = tuple(sudo_prefix)
        self.burst = burst
        self.latency = latency
        self.timeout_s = timeout_s
        self.enabled = enabled
        self._limit_active = False
        self._current_rate_kbps = 0

    # ------------------------------------------------------------------
    @property
    def limit_active(self) -> bool:
        """True se ha (pelo menos teoricamente) um limite TBF aplicado."""
        return self._limit_active

    @property
    def current_rate_kbps(self) -> int:
        """Ultimo limite pedido com sucesso (0 = sem limite)."""
        return self._current_rate_kbps

    # ------------------------------------------------------------------
    def _tc_binary(self) -> str | None:
        return shutil.which("tc")

    def _run(self, args: list[str]) -> tuple[bool, str]:
        """Corre ``<sudo_prefix> tc <args>``. Devolve (ok, mensagem)."""
        comando = [*self.sudo_prefix, "tc", *args]
        LOG.info("tc: %s", " ".join(comando))
        try:
            proc = subprocess.run(  # noqa: S603 - argumentos construidos, nao shell
                comando,
                capture_output=True,
                text=True,
                timeout=self.timeout_s,
                check=False,
            )
        except FileNotFoundError as exc:
            return False, f"executavel nao encontrado: {exc}"
        except subprocess.TimeoutExpired:
            return False, (
                f"'{' '.join(comando)}' excedeu {self.timeout_s:.0f}s "
                "(o sudo esta a pedir password?)"
            )
        except PermissionError as exc:
            return False, f"sem permissao para executar {comando[0]}: {exc}"
        except OSError as exc:
            return False, f"falha a executar {' '.join(comando)}: {exc}"

        if proc.returncode == 0:
            return True, (proc.stderr or proc.stdout).strip()

        detalhe = (proc.stderr or proc.stdout).strip() or f"exit {proc.returncode}"
        return False, detalhe

    # ------------------------------------------------------------------
    def clear(self) -> tuple[bool, str]:
        """Remove a regra TBF. Ausencia de regra nao e erro."""
        ok, msg = self._run(["qdisc", "del", "dev", self.iface, "root"])
        if ok:
            self._limit_active = False
            self._current_rate_kbps = 0
            return True, f"limite removido de {self.iface}"
        # `tc qdisc del` sem regra previa devolve erro, mas nao e falha para o
        # OBC. O padrao tem de ser ancorado: a mensagem de "executavel nao
        # encontrado" tambem contem "No such file or directory" e seria
        # confundida com "ja nao havia qdisc", reportando sucesso indevidamente.
        already_gone = (
            "RTNETLINK answers: No such file or directory",
            "Cannot delete qdisc",
        )
        if any(p in msg for p in already_gone):
            self._limit_active = False
            self._current_rate_kbps = 0
            LOG.debug("tc: nao havia qdisc root em %s", self.iface)
            return True, f"nenhum limite ativo em {self.iface}"
        return False, msg

    def set_limit(self, rate_kbps: int) -> tuple[bool, str]:
        """Define o limite de banda.

        ``rate_kbps <= 0`` remove o limite (largura de banda ilimitada).

        Devolve ``(ok, mensagem)``. Nunca levanta excecao.
        """
        if not self.enabled:
            self._limit_active = False
            self._current_rate_kbps = 0
            return True, "gestao de banda desativada (--no-tc)"

        if self._tc_binary() is None and "tc" not in self.sudo_prefix:
            LOG.warning(
                "tc: executavel 'tc' nao encontrado no PATH; a gestao de banda "
                "ficara inoperante."
            )

        # limpa sempre a regra anterior: `add` falha se ja existir uma root qdisc
        ok, msg = self.clear()
        if not ok:
            self._limit_active = False
            LOG.warning("tc: nao foi possivel limpar a regra anterior: %s", msg)
            return False, f"tc del falhou: {msg}"

        if rate_kbps <= 0:
            LOG.info("tc: limite removido (pedido de %d kbps)", rate_kbps)
            return True, f"limite removido (pedido de {rate_kbps} kbps)"

        ok, msg = self._run(
            [
                "qdisc", "add", "dev", self.iface, "root",
                "tbf", "rate", f"{rate_kbps}kbit",
                "burst", self.burst,
                "latency", self.latency,
            ]
        )
        if not ok:
            self._limit_active = False
            LOG.warning("tc: falha ao aplicar %d kbps: %s", rate_kbps, msg)
            return False, f"tc add falhou: {msg}"

        self._limit_active = True
        self._current_rate_kbps = rate_kbps
        LOG.info("tc: limite de %d kbps aplicado a %s", rate_kbps, self.iface)
        return True, f"limite de {rate_kbps} kbps aplicado a {self.iface}"

    def describe(self) -> str:
        """Estado actual, para logs e telemetria."""
        if not self.enabled:
            return "desativado"
        if not self._limit_active:
            return f"sem limite em {self.iface}"
        return f"{self._current_rate_kbps} kbps em {self.iface}"
