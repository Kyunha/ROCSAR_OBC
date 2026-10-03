#!/usr/bin/env bash
#
# Compila proto/rocsar_messages.proto para Python.
#
# Saida: src/rocsar_messages_pb2.py
#
# Uso:  ./scripts/build_proto.sh
#
# Requer o protoc instalado. Em nix, use `nix-shell` e depois este script.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

PROTO_DIR="${ROOT_DIR}/proto"
OUT_DIR="${ROOT_DIR}/src"
PROTO_FILE="${PROTO_DIR}/rocsar_messages.proto"

# ---------------------------------------------------------------------------
# Pre-condicoes
# ---------------------------------------------------------------------------
if ! command -v protoc >/dev/null 2>&1; then
    echo "[build_proto] ERRO: 'protoc' nao encontrado no PATH." >&2
    echo "[build_proto] Instale com:  apt install protobuf-compiler" >&2
    echo "[build_proto] Ou entre no ambiente de desenvolvimento:  nix-shell" >&2
    exit 1
fi

if [[ ! -f "${PROTO_FILE}" ]]; then
    echo "[build_proto] ERRO: ficheiro .proto nao encontrado: ${PROTO_FILE}" >&2
    exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "[build_proto] ERRO: interpretador '${PYTHON_BIN}' nao encontrado." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Geracao
# ---------------------------------------------------------------------------
echo "[build_proto] protoc: $(protoc --version)"
echo "[build_proto] Python: $(${PYTHON_BIN} --version 2>&1)"

mkdir -p "${OUT_DIR}"
rm -f "${OUT_DIR}/rocsar_messages_pb2.py" "${OUT_DIR}/rocsar_messages_pb2.pyi"

# --python_out gera <nome_sem_ext>.py a partir de <nome_sem_ext>.proto
# --pyi_out gera o stub de tipos correspondente. Sem ele o mypy trata o
# modulo gerado como dinamico e reporta dezenas de 'Module has no attribute'.
protoc \
    --proto_path="${PROTO_DIR}" \
    --python_out="${OUT_DIR}" \
    --pyi_out="${OUT_DIR}" \
    "${PROTO_FILE}"

GENERATED="${OUT_DIR}/rocsar_messages_pb2.py"
GENERATED_STUB="${OUT_DIR}/rocsar_messages_pb2.pyi"
if [[ ! -f "${GENERATED}" ]]; then
    echo "[build_proto] ERRO: protoc nao gerou ${GENERATED}" >&2
    exit 1
fi
if [[ ! -f "${GENERATED_STUB}" ]]; then
    echo "[build_proto] AVISO: protoc nao gerou ${GENERATED_STUB}; o mypy vai" >&2
    echo "[build_proto]         tratar o modulo gerado como dinamico." >&2
fi

echo "[build_proto] Gerado: ${GENERATED} ($(wc -c <"${GENERATED}") bytes)"

# ---------------------------------------------------------------------------
# Smoke test: importar e construir cada mensagem do contrato
# ---------------------------------------------------------------------------
if ! "${PYTHON_BIN}" -c "
import sys
sys.path.insert(0, '${ROOT_DIR}')
from src import rocsar_messages_pb2 as pb

assert pb.DESCRIPTOR.package == 'rocsar', pb.DESCRIPTOR.package

esperadas = [
    'SetTargetPayload', 'SetBandwidthPayload', 'ControlHeaterPayload',
    'TriggerCameraPayload', 'JogServoPayload', 'CommandRequest',
    'CommandResponse', 'ServoData', 'GnssData', 'SystemHealth',
    'TelemetryFrame',
]
faltam = [m for m in esperadas if m not in pb.DESCRIPTOR.message_types_by_name]
assert not faltam, f'mensagens em falta no contrato: {faltam}'

# Round-trip minimo do caminho critico GS -> OBC -> GS
req = pb.CommandRequest()
req.command_id = 'smoke'
req.type = pb.CMD_SET_TARGET
req.set_target.target_heading = 180.0
dec = pb.CommandRequest()
dec.ParseFromString(req.SerializeToString())
assert dec.set_target.target_heading == 180.0

frame = pb.TelemetryFrame()
frame.gnss.latitude = 41.1579
frame.servos.add(id=1, current_tick=2048)
back = pb.TelemetryFrame()
back.ParseFromString(frame.SerializeToString())
assert len(back.servos) == 1 and back.gnss.latitude == 41.1579

print(f'[build_proto] Contrato validado: {len(esperadas)} mensagens, '
      f'{len(pb.DESCRIPTOR.enum_types_by_name)} enum, round-trip OK')
"; then
    echo "[build_proto] ERRO: o codigo gerado nao importa ou nao corresponde ao contrato." >&2
    exit 1
fi

echo "[build_proto] OK"