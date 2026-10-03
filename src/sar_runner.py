"""Wrapper de linha de comando para a aquisicao SAR.

Este modulo existe para que o comando por omissao da configuracao
(``python3 -m sar_runner``) seja executavel, e para ser o ponto unico onde se
liga o comando SAR real ao OBC.

Por omissao este wrapper **nao faz nada sozinho**: delega no comando
``--sar-command`` (ou ``$ROCSAR_SAR_COMMAND``) e propaga o exit code. Assim o
comando SAR real da missao substitui este ficheiro sem alterar nada no OBC --
ou entao aponta-se ``--sar-command`` directamente para o executavel real e este
modulo deixa de ser usado.

Exemplos::

    # usar o binario C++ do SDR presente no repositorio
    python3 -m src.sar_runner --sar-command './sdr-ettus-b200mini/run.sh'

    # ver o que seria executado, sem executar
    python3 -m src.sar_runner --dry-run
"""

from __future__ import annotations

import argparse
import logging
import shlex
import subprocess
import sys
import time
from pathlib import Path, PurePath

from .config import DEFAULT_SAR_DELEGATE, resolve_data_dir

LOG = logging.getLogger("rocsar.sar")

#: Tokens que identificam este proprio modulo, para detectar recursao.
_SELF_TOKENS = {"sar_runner", "src.sar_runner"}


def _invokes_self(argv_cmd: list[str]) -> bool:
    """O comando pediria ao dispatcher que se invoque a si proprio?

    O OBC chama ``python3 -m sar_runner`` e este modulo delega na aquisicao
    real. Se o delegado voltar a ser o dispatcher -- por configuracao errada, ou
    porque ``--sar-command`` herdou o valor do proprio ``DEFAULT_SAR_COMMAND`` --
    obtem-se um ``fork`` infinito que enche a memoria e o disco.

    Compara o basename de cada token, por isso apanha tambem ``./sar_runner``,
    ``/usr/local/bin/sar_runner`` e a forma ``python3 -m sar_runner`` (onde
    ``sar_runner`` e, ele proprio, um token).

    Limitacao conhecida: nao descentranha codigo de shell, logo
    ``sh -c "python3 -m sar_runner"`` nao e detectado. E uma configuracao
    patologica; se a aktivares, o operador ve o processo a crescer sem parar.
    """
    return any(PurePath(tok).name in _SELF_TOKENS for tok in argv_cmd)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="sar_runner",
        description="Dispara a aquisicao SAR usando o comando configurado.",
    )
    p.add_argument(
        "--sar-command",
        default=DEFAULT_SAR_DELEGATE,
        help=(
            "Comando de aquisicao real a executar. {out} = caminho de saida. "
            f"(default: {DEFAULT_SAR_DELEGATE})"
        ),
    )
    p.add_argument(
        "--data-dir",
        default=None,
        help="Directorio de trabalho e de logs (default: o do OBC).",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Mostra o comando que seria executado e sai.",
    )
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    argv_cmd = shlex.split(args.sar_command)
    if not argv_cmd:
        LOG.error("--sar-command esta vazio")
        return 2

    if _invokes_self(argv_cmd):
        LOG.error("--sar-command aponta para o proprio sar_runner: %s", args.sar_command)
        LOG.error("Isto criaria recursao infinita. Aponte para a aquisicao real.")
        return 2

    data_dir = resolve_data_dir(Path(args.data_dir) if args.data_dir else None)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_path = data_dir / f"sar-{stamp}.bin"
    final = [a.replace("{out}", str(out_path)) for a in argv_cmd]

    if args.dry_run:
        print(f"[sar_runner] data-dir : {data_dir}")
        print(f"[sar_runner] saida    : {out_path}")
        print(f"[sar_runner] comando  : {shlex.join(final)}")
        return 0

    LOG.info("[sar_runner] directorio de trabalho: %s", data_dir)
    LOG.info("[sar_runner] a executar: %s", shlex.join(final))

    try:
        return subprocess.call(final, cwd=str(data_dir))  # noqa: S603
    except FileNotFoundError:
        LOG.error("[sar_runner] executavel '%s' nao encontrado", final[0])
        LOG.error(
            "[sar_runner] Aplique o firmware/ferramenta SAR ou ajuste "
            "--sar-command / $ROCSAR_SAR_COMMAND."
        )
        return 127
    except KeyboardInterrupt:
        LOG.warning("[sar_runner] interrompido pelo operador")
        return 130


if __name__ == "__main__":
    sys.exit(main())
