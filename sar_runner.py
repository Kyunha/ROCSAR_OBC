"""Entrada de nivel superior do disparador SAR.

Existe para que o comando por omissao da configuracao
(``python3 -m sar_runner``) seja executavel a partir da raiz do repositorio,
como pede a documentacao de operacao. A implementacao vive em
``src/sar_runner.py``; este modulo e apenas uma ponte.

Equivalente: ``python3 -m src.sar_runner``.
"""

from __future__ import annotations

from src.sar_runner import main

if __name__ == "__main__":
    raise SystemExit(main())
