#!/usr/bin/env bash
#
# Arranca o servidor OBC do ROCSAR.
#
#   ./scripts/run_server.sh                 # hardware real (configuracao de voo)
#   ./scripts/run_server.sh --mock          # desenvolvimento, sem hardware
#
# Todos os argumentos sao passados tal e qual ao servidor, por isso
# `./scripts/run_server.sh --mock --log-level DEBUG` funciona.
#
# Para carregar a interface usa o simulador da GS:
#   python3 tests/test_client_sim.py --host 127.0.0.1
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

PYTHON_BIN="${PYTHON_BIN:-python3}"

# ---------------------------------------------------------------------------
# Pre-condicoes
# ---------------------------------------------------------------------------
if [[ ! -f "src/rocsar_messages_pb2.py" ]]; then
    echo "[run_server] Falta src/rocsar_messages_pb2.py -- a gerar..." >&2
    "${SCRIPT_DIR}/build_proto.sh"
    echo
fi

# Detecta se foi pedido mock parauityo um aviso bem visivel.
MOCK_REQUESTED=0
for arg in "$@"; do
    case "${arg}" in
        --mock|--mock-pico|--mock-sar|--mock-gnss|--auto-mock) MOCK_REQUESTED=1 ;;
    esac
done

if [[ "${MOCK_REQUESTED}" -eq 1 ]]; then
    cat >&2 <<'EOF'
+----------------------------------------------------------------------+
| AVISO: MODO DE DESENVOLVIMENTO                                       |
|                                                                      |
| Foi pedido um modo simulado. A telemetria e ARTIFICIAL.               |
| Nao use esta configuracao em voo -- retire as flags --mock* antes      |
| de armar a gondola.                                                  |
+----------------------------------------------------------------------+
EOF
fi

# ---------------------------------------------------------------------------
# Permissao para tc (opcional). Num OBC de voo o utilizador de servico deve
# poder mudar a banda sem password, ex.:
#   echo "%sudo ALL=(root) NOPASSWD: /sbin/tc" | sudo tee /etc/sudoers.d/rocsar
# ---------------------------------------------------------------------------
if [[ "${ROCSAR_SUDO_PREFIX+x}" != "x" ]]; then
    export ROCSAR_SUDO_PREFIX="${ROCSAR_SUDO_PREFIX:-sudo}"
fi

echo "[run_server] a arrancar: ${PYTHON_BIN} -m src.obc_server $*"
exec "${PYTHON_BIN}" -m src.obc_server "$@"