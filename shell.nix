{ pkgs ? import <nixpkgs> {} }:

let
  python = pkgs.python3.withPackages (ps: with ps; [
    pip
    setuptools
    pyserial
    pyzmq
    protobuf

    # P5's ground-station GUI (groundstation/qt_app.py). This is a development
    # dependency only: it is in pyproject's `gui` optional extra and is NOT in
    # the declared runtime `dependencies`, because the GUI runs on a ground
    # station and must never enter the Pi's installed footprint. Carrying it
    # here means the widget layer is actually runnable and testable in the
    # supported environment rather than written blind and verified somewhere
    # else. Qt widgets do not need the display here -- the tests use the
    # offscreen platform (see tests/conftest.py) -- but they do need the
    # runtime libraries PySide6 pulls in, which nixpkgs resolves.
    pyside6
  ]);
in
pkgs.mkShell {
  packages = [
    python
    pkgs.mypy
    pkgs.ruff
    pkgs.gcc
    pkgs.curl
    pkgs.util-linux
    pkgs.protobuf
  ];

  shellHook = ''
    echo "Python development shell"
    python --version
    gcc --version | head -1
  '';
}
