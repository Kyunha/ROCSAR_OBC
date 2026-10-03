"""Testes de :mod:`src.bandwidth_manager`.

O contracto central e a politica *fail-soft*: ``tc`` pode falhar por dezenas de
razoes (sem root, sem ``tc``, interface errada, kernel sem suporte) e **nenhuma**
pode derrubar o servidor. Todos os testes aqui injectam um ``_run`` falso, para
nao tocar em ``tc`` nem em ``sudo`` reais.
"""

from __future__ import annotations

import subprocess

import pytest

from src.bandwidth_manager import BandwidthManager


class FakeTc:
    """Substitui ``_run`` e responde como o ``tc``/`sudo`` responding."""

    def __init__(self, *, on_del: tuple[bool, str] = (True, ""),
                 on_add: tuple[bool, str] = (True, "")) -> None:
        self.on_del = on_del
        self.on_add = on_add
        self.calls: list[list[str]] = []

    def __call__(self, args: list[str]) -> tuple[bool, str]:
        self.calls.append(list(args))
        if args[:2] == ["qdisc", "del"]:
            return self.on_del
        if args[:2] == ["qdisc", "add"]:
            return self.on_add
        return True, ""

    @property
    def n_del(self) -> int:
        return sum(1 for c in self.calls if c[:2] == ["qdisc", "del"])

    @property
    def n_add(self) -> int:
        return sum(1 for c in self.calls if c[:2] == ["qdisc", "add"])


def make(tc: FakeTc | None = None, **kwargs) -> tuple[BandwidthManager, FakeTc]:
    mgr = BandwidthManager(**kwargs)
    fake = tc if tc is not None else FakeTc()
    mgr._run = fake  # type: ignore[method-assign]
    return mgr, fake


# ===========================================================================
class TestSetLimitHappyPath:
    def test_aplica_limite(self) -> None:
        mgr, tc = make()
        ok, msg = mgr.set_limit(512)
        assert ok, msg
        assert mgr.limit_active is True
        assert mgr.current_rate_kbps == 512
        assert tc.n_del == 1, "tem de limpar a regra anterior antes de add"
        assert tc.n_add == 1

    def test_comando_tem_interface_e_rate(self) -> None:
        mgr, tc = make(iface="eth1", burst="64kbit", latency="800ms")
        mgr.set_limit(256)
        add = next(c for c in tc.calls if c[:2] == ["qdisc", "add"])
        assert add[:5] == ["qdisc", "add", "dev", "eth1", "root"]
        assert "tbf" in add
        assert "256kbit" in add
        assert "64kbit" in add
        assert "800ms" in add

    def test_del_usa_a_interface_certa(self) -> None:
        mgr, tc = make(iface="wlan0")
        mgr.set_limit(128)
        dele = next(c for c in tc.calls if c[:2] == ["qdisc", "del"])
        assert dele == ["qdisc", "del", "dev", "wlan0", "root"]

    def test_reaplicar_substitui(self) -> None:
        mgr, tc = make()
        mgr.set_limit(512)
        mgr.set_limit(1024)
        assert tc.n_del == 2
        assert tc.n_add == 2
        assert mgr.current_rate_kbps == 1024


# ===========================================================================
class TestRemoveLimit:
    @pytest.mark.parametrize("rate", [0, -1])
    def test_rate_nao_positivo_remove(self, rate: int) -> None:
        mgr, tc = make()
        mgr.set_limit(512)
        ok, msg = mgr.set_limit(rate)
        assert ok, msg
        assert mgr.limit_active is False
        assert mgr.current_rate_kbps == 0
        assert tc.n_add == 1, "nao deve re-aplicar um limite para 0"

    def test_clear_quando_nao_ha_regra_e_sucesso(self) -> None:
        """`tc qdisc del` sem regra devolve erro -- nao e falha para o OBC."""
        mgr, _ = make(tc=FakeTc(
            on_del=(False, "RTNETLINK answers: No such file or directory")))
        ok, msg = mgr.clear()
        assert ok, msg
        assert mgr.limit_active is False


# ===========================================================================
class TestFailSoft:
    """Nenhum destes casos pode levantar excecao nem marcar limite."""

    def test_add_falha(self) -> None:
        mgr, _ = make(tc=FakeTc(on_add=(False, "RTNETLINK answers: No such device")))
        ok, msg = mgr.set_limit(512)
        assert ok is False
        assert "No such device" in msg
        assert mgr.limit_active is False

    def test_del_falha_impede_o_add(self) -> None:
        mgr, tc = make(tc=FakeTc(on_del=(False, "Operation not permitted")))
        ok, msg = mgr.set_limit(512)
        assert ok is False
        assert "del falhou" in msg
        assert "Operation not permitted" in msg
        assert tc.n_add == 0, "nao faz sentido aplicar se nao limpou"
        assert mgr.limit_active is False

    def test_add_falha_depois_de_del_bem_sucedido(self) -> None:
        mgr, _ = make(tc=FakeTc(on_del=(True, ""), on_add=(False, "No buffer")))
        ok, msg = mgr.set_limit(512)
        assert ok is False
        assert "add falhou" in msg
        assert mgr.limit_active is False

    def test_timeout_do_sudo(self, monkeypatch: pytest.MonkeyPatch) -> None:
        def boom(*a, **k):
            raise subprocess.TimeoutExpired(cmd=["sudo", "tc"], timeout=5.0)
        monkeypatch.setattr(subprocess, "run", boom)
        mgr = BandwidthManager()
        ok, msg = mgr.clear()
        assert ok is False
        assert "password" in msg or "excedeu" in msg

    def test_tc_ausente(self, monkeypatch: pytest.MonkeyPatch) -> None:
        def boom(*a, **k):
            raise FileNotFoundError(2, "No such file or directory", "tc")
        monkeypatch.setattr(subprocess, "run", boom)
        mgr = BandwidthManager()
        ok, msg = mgr.clear()
        assert ok is False
        assert "nao encontrado" in msg or "not found" in msg.lower()

    def test_permission_error(self, monkeypatch: pytest.MonkeyPatch) -> None:
        def boom(*a, **k):
            raise PermissionError(13, "Permission denied")
        monkeypatch.setattr(subprocess, "run", boom)
        mgr = BandwidthManager()
        ok, msg = mgr.clear()
        assert ok is False
        assert "permissao" in msg.lower() or "permission" in msg.lower()

    def test_os_error_generico(self, monkeypatch: pytest.MonkeyPatch) -> None:
        def boom(*a, **k):
            raise OSError("dispositivo ocupado")
        monkeypatch.setattr(subprocess, "run", boom)
        mgr = BandwidthManager()
        ok, msg = mgr.clear()
        assert ok is False
        assert "ocupado" in msg


# ===========================================================================
class TestPrefixo:
    def test_sudo_prefixo_vazio_e_legitimo(self) -> None:
        """Ja a correr como root, `sudo` e desnecessario."""
        mgr, _ = make(sudo_prefix=())
        ok, msg = mgr.set_limit(64)
        assert ok, msg
        assert mgr.limit_active is True

    def test_doas_como_prefixo(self) -> None:
        mgr, _ = make(sudo_prefix=("doas", "tc"))
        ok, msg = mgr.set_limit(64)
        assert ok, msg


# ===========================================================================
class TestDisabled:
    def test_desativado_nao_chama_tc(self) -> None:
        mgr, tc = make(enabled=False)
        ok, msg = mgr.set_limit(512)
        assert ok, msg
        assert "desativada" in msg
        assert tc.calls == [], "nao deve executar nada"
        assert mgr.limit_active is False

    def test_desativado_descreve(self) -> None:
        mgr, _ = make(enabled=False)
        assert mgr.describe() == "desativado"


# ===========================================================================
class TestDescribe:
    def test_describe_sequencia(self) -> None:
        mgr, _ = make(iface="eth0")
        assert mgr.describe() == "sem limite em eth0"
        mgr.set_limit(512)
        assert mgr.describe() == "512 kbps em eth0"
        mgr.set_limit(0)
        assert mgr.describe() == "sem limite em eth0"

    def test_estado_inicial(self) -> None:
        mgr, _ = make()
        assert mgr.limit_active is False
        assert mgr.current_rate_kbps == 0
