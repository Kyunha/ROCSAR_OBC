import rocsar_messages_pb2 as pb
import time

# 1. CRIAR UM COMANDO NA GROUND STATION
cmd = pb.CommandRequest()
cmd.command_id = f"cmd_{int(time.time())}"
cmd.type = pb.CMD_SET_TARGET
cmd.set_target.target_heading = 180.5  # Apontar para 180.5 graus

# Codificar a mensagem para bytes (pronto para enviar via ZeroMQ)
binary_payload = cmd.SerializeToString()
print(f"Tamanho do pacote binário codificado: {len(binary_payload)} bytes")


# 2. DECODIFICAR O COMANDO NO OBC (RASPBERRY PI 4B)
received_cmd = pb.CommandRequest()
received_cmd.ParseFromString(binary_payload)

print(f"\nComando Recebido ID: {received_cmd.command_id}")
if received_cmd.type == pb.CMD_SET_TARGET:
    target_angle = received_cmd.set_target.target_heading
    print(f"Ação: Definir Target Heading para {target_angle}°")
