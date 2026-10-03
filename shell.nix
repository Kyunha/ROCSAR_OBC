{ pkgs ? import <nixpkgs> {} }:

let
  python = pkgs.python3.withPackages (ps: with ps; [
    # Runtime do OBC -- tem de bater certo com as `dependencies` de
    # pyproject.toml, para o que se instala no Pi e o que se develope aqui
    # serem o mesmo software.
    pyserial
    pyzmq
    protobuf

    # Build dos stubs protobuf (scripts/build_proto.sh) e empacotamento.
    setuptools
    pip

    # Testes e verificacao estatica.
    pytest
    pytest-cov
    mypy
    ruff

    # O OBC chama `tc` para limitar a banda e `tc` precisa de iproute2.
    
  ]);
in
pkgs.mkShell {
  packages = [
    python
    pkgs.gcc
    pkgs.protobuf
    pkgs.curl
    pkgs.util-linux
    pkgs.git
  ];

  shellHook = ''
    echo "ROCSAR OBC -- shell de desenvolvimento"
    python --version
    gcc --version | head -1
    echo
    echo "  testes     : pytest -q"
    echo "  lint       : ruff check ."
    echo "  tipos      : mypy src"
    echo "  proto      : scripts/build_proto.sh"
    echo "  servidor   : python3 -m src.obc_server"
    echo "  simulador  : python3 tests/test_client_sim.py"
  '';
}
