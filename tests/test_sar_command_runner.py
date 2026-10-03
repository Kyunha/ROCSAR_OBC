"""Testes de :mod:`src.sar_runner` e :mod:`src.command_runner`.

O ponto critico do dispatcher SAR e a **auto-recursao**: o OBC chama
``python3 -m sar_runner``, que tem de delegar na aquisicao real. Se o delegado
voltar a ser o proprio dispatcher, obtem-se um ``fork`` infinito.

O :class:`CommandRunner` e diferente do ``sar_runner``: lanca processos de longa
duracao e nao espera por eles. Por isso os testes usam ``/bin/sh`` com um
ficheiro de flag em vez de dormir.
"""

from __future__ import annotations

import shutil
import subprocess
import time
from pathlib import Path

import pytest

from src import sar_runner
from src.command_runner import CommandRunner
from src.config import DEFAULT_SAR_DELEGATE

#: Este ambiente (Nix) nao tem /bin/true, /bin/false nem /bin/sleep, por isso
#: os testes usam `sh -c`, que existe em qualquer sistema POSIX.
SH = shutil.which("sh") or "/bin/sh"
EXIT_OK = (SH, "-c", "exit 0")
EXIT_FAIL = (SH, "-c", "exit 1")
SLEEP_30 = (SH, "-c", "sleep 30")


def wait_until(pred, timeout_s: float = 5.0, poll_s: float = 0.02) -> bool:
    """Espera activa ate ``pred()`` ser verdadeiro."""
    fim = time.monotonic() + timeout_s
    while time.monotonic() < fim:
        if pred():
            return True
        time.sleep(poll_s)
    return pred()


# ===========================================================================
class TestSelfRecursionGuard:
    """O dispatcher nunca pode invocar-se a si proprio."""

    @pytest.mark.parametrize("comando", [
        "python3 -m sar_runner",
        "python3 -m src.sar_runner",
        "python3 sar_runner",
        "./sar_runner",
        "/usr/local/bin/sar_runner",
    ])
    def test_recursao_recusada(self, comando: str) -> None:
        assert sar_runner.main(["--sar-command", comando]) == 2, (
            f"recursao nao recusada para {comando!r}"
        )

    @pytest.mark.parametrize("comando", [
        "./sdr-ettus-b200mini/run.sh",
        "/usr/local/bin/sar_capture --out {out}",
        "python3 -m src.sar_acquisition",
        "./tools/sar_runner_v2",
        "sar",
        "true",
    ])
    def test_comandos_validos_passao(self, comando: str) -> None:
        rc = sar_runner.main(["--sar-command", comando, "--dry-run"])
        assert rc == 0, f"comando valido recusado: {comando!r}"

    def test_comando_vazio(self) -> None:
        assert sar_runner.main(["--sar-command", ""]) == 2

    def test_default_delega_no_sdr(self) -> None:
        """O default tem de ser a aquisicao real, nunca o dispatcher."""
        assert "sar_runner" not in DEFAULT_SAR_DELEGATE
        assert not sar_runner._invokes_self(DEFAULT_SAR_DELEGATE.split())


# ===========================================================================
class TestDryRun:
    def test_nao_executa_nada(self, tmp_path: Path,
                             monkeypatch: pytest.MonkeyPatch) -> None:
        calls: list = []
        monkeypatch.setattr(subprocess, "call", lambda *a, **k: calls.append(a) or 0)
        rc = sar_runner.main(["--dry-run", "--data-dir", str(tmp_path)])
        assert rc == 0
        assert calls == [], "--dry-run nao pode executar nada"

    def test_substitui_out(self, tmp_path: Path,
                           capsys: pytest.CaptureFixture) -> None:
        sar_runner.main(["--dry-run", "--data-dir", str(tmp_path),
                         "--sar-command", "cap --file {out}"])
        saida = capsys.readouterr().out
        assert "{out}" not in saida, "o marcador {out} tem de ser substituido"
        assert "sar-" in saida and ".bin" in saida


# ===========================================================================
class TestSarRunnerExecution:
    def test_propaga_exit_code(self, tmp_path: Path,
                              monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(subprocess, "call", lambda *a, **k: 3)
        rc = sar_runner.main(["--data-dir", str(tmp_path),
                              "--sar-command", "cap"])
        assert rc == 3

    def test_executavel_ausente_da_127(self, tmp_path: Path) -> None:
        rc = sar_runner.main(["--data-dir", str(tmp_path),
                              "--sar-command", "nao-existe-xyz-123"])
        assert rc == 127

    def test_executa_aquisicao_real(self, tmp_path: Path) -> None:
        """O comando delegado corre mesmo, no diretorio certo, e propaga o exit."""
        rc = sar_runner.main(["--data-dir", str(tmp_path),
                              "--sar-command", f"{SH} -c 'echo 7 > {{out}}'"])
        assert rc == 0
        # sar_runner escolhe o proprio nome: sar-<timestamp>.bin
        ficheiros = list(tmp_path.glob("sar-*.bin"))
        assert ficheiros, f"nenhum ficheiro de saida em {tmp_path}"
        assert ficheiros[0].read_text().strip() == "7"


# ===========================================================================
class TestCommandRunnerMock:
    def test_mock_nao_lanca_nada(self, tmp_path: Path) -> None:
        r = CommandRunner("sar", EXIT_FAIL, mock=True, work_dir=tmp_path)
        ok, msg = r.start()
        assert ok, msg
        assert "MOCK" in msg
        assert r.is_running is False, "mock nao pode ter processo"
        assert r.last_exit_code is None

    def test_mock_nao_cria_ficheiros(self, tmp_path: Path) -> None:
        CommandRunner("sar", EXIT_FAIL, mock=True,
                      work_dir=tmp_path / "nao-existe").start()
        assert not (tmp_path / "nao-existe").exists()


class TestCommandRunnerReal:
    def test_sucesso(self, tmp_path: Path) -> None:
        r = CommandRunner("t", EXIT_OK, mock=False, work_dir=tmp_path)
        ok, msg = r.start()
        assert ok, msg
        assert wait_until(lambda: r.reap() is not None)
        assert r.last_exit_code == 0
        r.stop()

    def test_falha_propaga_exit_code(self, tmp_path: Path) -> None:
        r = CommandRunner("t", EXIT_FAIL, mock=False, work_dir=tmp_path)
        ok, msg = r.start()
        # Python 3.14 so aceita `assert <test>, <msg>` -- no maximo dois
        # elementos separados por virgula.
        assert ok, f"o lancamento tem de ser bem sucedido: {msg}"
        assert wait_until(lambda: r.reap() is not None)
        assert r.last_exit_code == 1
        r.stop()

    def test_comando_inexistente(self, tmp_path: Path) -> None:
        r = CommandRunner("t", ("nao-existe-xyz-123",), mock=False,
                          work_dir=tmp_path)
        ok, msg = r.start()
        assert not ok, "tem de falhar de forma soft"
        assert "nao encontrado" in msg

    def test_out_e_substituido(self, tmp_path: Path) -> None:
        """{out} tem de chegar ao processo como caminho real."""
        r = CommandRunner("cam", (SH, "-c", 'printf ok > "{out}"'),
                          mock=False, work_dir=tmp_path)
        ok, msg = r.start(suffix="txt")
        assert ok, msg
        assert wait_until(lambda: r.reap() is not None)
        ficheiros = list(tmp_path.glob("cam-*.txt"))
        assert ficheiros, "o ficheiro de destino nao foi criado"
        assert ficheiros[0].read_text() == "ok"
        r.stop()

    def test_log_captura_stdout(self, tmp_path: Path) -> None:
        r = CommandRunner("t", (SH, "-c", "echo ola-mundo"), mock=False,
                          work_dir=tmp_path)
        r.start()
        assert r.log_path is not None
        assert wait_until(lambda: r.reap() is not None)
        assert wait_until(lambda: "ola-mundo" in r.log_path.read_text())
        r.stop()

    def test_nao_lanca_duas_vezes(self, tmp_path: Path) -> None:
        r = CommandRunner("t", SLEEP_30, mock=False, work_dir=tmp_path)
        ok1, _ = r.start()
        assert ok1
        ok2, msg = r.start()
        assert not ok2, "segundo disparo tem de ser recusado"
        assert "ja esta a correr" in msg
        r.stop()

    def test_stop_termina(self, tmp_path: Path) -> None:
        r = CommandRunner("t", SLEEP_30, mock=False, work_dir=tmp_path)
        assert r.start()[0]
        assert r.is_running
        ok, msg = r.stop(timeout_s=3.0)
        assert ok, msg
        assert not r.is_running

    def test_stop_sem_processo(self, tmp_path: Path) -> None:
        r = CommandRunner("t", EXIT_OK, mock=False, work_dir=tmp_path)
        ok, msg = r.stop()
        assert ok
        assert "nada a parar" in msg

    def test_status_antes_e_depois(self, tmp_path: Path) -> None:
        r = CommandRunner("t", EXIT_OK, mock=False, work_dir=tmp_path)
        assert "nunca executado" in r.status()
        r.start()
        assert wait_until(lambda: r.reap() is not None)
        assert "terminou" in r.status()
        r.stop()

    def test_work_dir_inacessivel(self, tmp_path: Path) -> None:
        alvo = tmp_path / "sub"
        alvo.mkdir()
        alvo.chmod(0o500)  # leitura + execucao, sem escrita
        try:
            r = CommandRunner("t", EXIT_OK, mock=False, work_dir=alvo)
            ok, msg = r.start()
            assert not ok, "tem de falhar de forma soft"
            assert "log" in msg
        finally:
            alvo.chmod(0o700)
