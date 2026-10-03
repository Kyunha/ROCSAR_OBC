"""Execucao de comandos externos de longa duracao (SAR, fotografia).

O comando a executar vem da configuracao, nunca do payload da mensagem
protobuf: a GS comanda *quando* disparar, mas nao *o que* executar. Um payload
que indique um nome de executavel seria execucao remota de codigo arbitrario.

O comando e lancado com :class:`subprocess.Popen` e nao bloqueia a thread de
comandos. A saida e escrita num ficheiro de log em vez de herdar o terminal,
para que o log do servidor nao seja inundado pelas mensagens de instrumentos.
"""

from __future__ import annotations

import logging
import shlex
import subprocess
import threading
import time
from pathlib import Path
from typing import IO

LOG = logging.getLogger(__name__)

#: Substituicao aceite no comando configurado. Se presente, e trocada pelo
#: caminho do ficheiro de saida.
OUT_PLACEHOLDER = "{out}"

DEFAULT_STOP_TIMEOUT_S = 5.0


class CommandRunner:
    """Lanca e acompanha um processo externo de longa duracao.

    Parametros
    ----------
    name:
        Nome logico ("sar", "camera"), usado nos logs.
    argv:
        Comando a executar, ja dividido em tokens. Se contiver ``{out}``, e
        substituido pelo caminho de saida.
    work_dir:
        Directorio de trabalho do processo e onde ficam os logs.
    mock:
        Quando True, :meth:`start` nao lanca nada e reporta sucesso. Este modo
        so e ativado por flag explicita (``--mock-sar``).
    """

    def __init__(
        self,
        name: str,
        argv: tuple[str, ...],
        *,
        work_dir: Path,
        mock: bool = False,
        stop_timeout_s: float = DEFAULT_STOP_TIMEOUT_S,
    ) -> None:
        self.name = name
        self.argv = tuple(argv)
        self.work_dir = work_dir
        self.mock = mock
        self.stop_timeout_s = stop_timeout_s

        self._proc: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()
        self._log_handle: IO[str] | None = None
        self._log_path: Path | None = None
        self._last_exit: int | None = None
        self._started_at: float | None = None

    # ------------------------------------------------------------------
    @property
    def is_running(self) -> bool:
        with self._lock:
            if self._proc is None:
                return False
            return self._proc.poll() is None

    @property
    def last_exit_code(self) -> int | None:
        with self._lock:
            return self._last_exit

    @property
    def log_path(self) -> Path | None:
        return self._log_path

    # ------------------------------------------------------------------
    def _build_argv(self, out_path: Path) -> list[str]:
        return [
            arg.replace(OUT_PLACEHOLDER, str(out_path)) if OUT_PLACEHOLDER in arg else arg
            for arg in self.argv
        ]

    def _new_output_path(self, suffix: str) -> Path:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        return self.work_dir / f"{self.name}-{stamp}.{suffix}"

    # ------------------------------------------------------------------
    def start(self, *, suffix: str = "bin", extra_args: tuple[str, ...] = ()) -> tuple[bool, str]:
        """Lanca o processo se ja nao estiver a correr.

        Devolve ``(ok, mensagem)``. Nunca levanta excecao.
        """
        if self.mock:
            with self._lock:
                self._started_at = time.time()
            LOG.warning(
                "%s: MODO MOCK -- NAO foi executado '%s'. Comando real: %s",
                self.name,
                shlex.join(self.argv),
                shlex.join(self.argv),
            )
            return True, (
                f"{self.name}: MOCK (nenhum processo lancado; "
                f"comando real seria '{shlex.join(self.argv)}')"
            )

        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                pid = self._proc.pid
                return False, (
                    f"{self.name} ja esta a correr (pid {pid}). "
                    "Aguarde o fim ou envie um novo comando de disparo."
                )

        out_path = self._new_output_path(suffix)
        argv = [*self._build_argv(out_path), *extra_args]

        try:
            self.work_dir.mkdir(parents=True, exist_ok=True)
            log_path = self.work_dir / f"{self.name}-{out_path.stem}.log"
            handle = log_path.open("w", encoding="utf-8")
        except OSError as exc:
            LOG.error("%s: nao foi possivel abrir o log: %s", self.name, exc)
            return False, f"{self.name}: nao foi possivel abrir o log: {exc}"

        LOG.info("%s: a executar %s", self.name, shlex.join(argv))
        try:
            proc = subprocess.Popen(  # noqa: S603 - comando vem da configuracao
                argv,
                cwd=str(self.work_dir),
                stdout=handle,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                text=True,
            )
        except FileNotFoundError:
            handle.close()
            LOG.error("%s: executavel '%s' nao encontrado", self.name, argv[0])
            return False, f"{self.name}: executavel '{argv[0]}' nao encontrado"
        except PermissionError as exc:
            handle.close()
            LOG.error("%s: sem permissao para '%s'", self.name, argv[0])
            return False, f"{self.name}: sem permissao para executar '{argv[0]}': {exc}"
        except OSError as exc:
            handle.close()
            LOG.error("%s: falha ao iniciar: %s", self.name, exc)
            return False, f"{self.name}: falha ao lancsar o processo: {exc}"

        with self._lock:
            self._proc = proc
            self._log_handle = handle
            self._log_path = log_path
            self._started_at = time.time()
            self._last_exit = None

        return True, f"{self.name}: iniciado (pid {proc.pid}), log em {log_path.name}"

    # ------------------------------------------------------------------
    def reap(self) -> int | None:
        """Recolhe o exit code se o processo ja tiver terminado."""
        with self._lock:
            proc = self._proc
            if proc is None:
                return None
            code = proc.poll()
            if code is None:
                return None
            if self._last_exit != code:
                self._last_exit = code
                LOG.info("%s: terminou com exit %d", self.name, code)
                handle = self._log_handle
                if handle is not None:
                    handle.close()
                    self._log_handle = None
            return code

    def stop(self, *, timeout_s: float | None = None) -> tuple[bool, str]:
        """Termina o processo se estiver a correr (SIGTERM, depois SIGKILL)."""
        with self._lock:
            proc = self._proc
        if proc is None:
            return True, f"{self.name}: nada a parar"

        if proc.poll() is not None:
            self.reap()
            return True, f"{self.name}: ja estava terminado (exit {proc.returncode})"

        limite = self.stop_timeout_s if timeout_s is None else timeout_s
        LOG.info("%s: a terminar pid %d", self.name, proc.pid)
        try:
            proc.terminate()
            proc.wait(timeout=limite)
        except subprocess.TimeoutExpired:
            LOG.warning("%s: pid %d ignorou SIGTERM; a usar SIGKILL", self.name, proc.pid)
            try:
                proc.kill()
                proc.wait(timeout=limite)
            except (subprocess.TimeoutExpired, OSError) as exc:
                return False, f"{self.name}: nao foi possivel terminar pid {proc.pid}: {exc}"
        except OSError as exc:
            return False, f"{self.name}: erro ao terminar: {exc}"

        self.reap()
        return True, f"{self.name}: terminado (exit {proc.returncode})"

    # ------------------------------------------------------------------
    def status(self) -> str:
        with self._lock:
            if self._proc is None:
                return f"{self.name}: nunca executado"
            pid = self._proc.pid
            code = self._last_exit
            inicio = self._started_at
        if self.is_running:
            duracao = (time.time() - inicio) if inicio else 0.0
            return f"{self.name}: a correr (pid {pid}, {duracao:.0f}s)"
        return f"{self.name}: terminou (exit {code})"
