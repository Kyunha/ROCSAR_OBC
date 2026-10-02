#!/usr/bin/env python3
"""
Módulo: Cliente ZMQ da Ground Station (GS) - Projeto ROCSAR
Descrição: Subscreve a telemetria do OBC e permite enviar comandos em Protobuf.
"""

import time
import threading
import zmq
import rocsar_messages_pb2 as pb

# ============================================================================
# CONFIGURAÇÃO DE LIGAÇÃO (Alterar IP para o IP do Pi4B em voo)
# ============================================================================
OBC_IP = "127.0.0.1"  # Usar "127.0.0.1" para testes locais ou IP da rede Ethernet
ZMQ_CMD_PORT = 5555
ZMQ_PUB_PORT = 5556

# ============================================================================
# THREAD SUBSCRITORA DE TELEMETRIA (SUB)
# ============================================================================
def telemetry_receiver_thread(context):
    """Escuta e descodifica as mensagens TelemetryFrame vindas do OBC."""
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(f"tcp://{OBC_IP}:{ZMQ_PUB_PORT}")
    sub_socket.setsockopt(zmq.SUBSCRIBE, b"TELEMETRY")

    print(f"[GS TELEMETRY] Conectado ao canal de telemetria em {OBC_IP}:{ZMQ_PUB_PORT}")

    while True:
        try:
            topic, binary_payload = sub_socket.recv_multipart()
            
            # Descodifica os bytes Protobuf para o objeto TelemetryFrame
            telemetry = pb.TelemetryFrame()
            telemetry.ParseFromString(binary_payload)

            # Imprime os dados formatados na consola
            print(f"\n--- [TELEMETRY UPDATE] ---")
            print(f" Orientação Gôndola: {telemetry.gondola_heading:.1f}° | Alvo: {telemetry.target_heading:.1f}°")
            print(f" Posição GNSS: Lat {telemetry.gnss.latitude:.4f}, Lon {telemetry.gnss.longitude:.4f}, Alt {telemetry.gnss.altitude}m")
            print(f" CPU Temp: {telemetry.health.cpu_temp_c}°C | Aquecedores: H1={telemetry.heater1_active}, H2={telemetry.heater2_active}")
            print(f" Servos Online: S1={telemetry.servos[0].online}, S2={telemetry.servos[1].online}")
            print("-" * 30)

        except Exception as e:
            print(f"[ERRO TELEMETRIA] {e}")
            break

# ============================================================================
# FUNÇÃO PARA ENVIAR COMANDOS (DEALER)
# ============================================================================
def send_command(cmd_socket, command_req):
    """Serializa o pacote CommandRequest e aguarda pela CommandResponse."""
    print(f"\n[GS ENVIANDO COMANDO] ID: {command_req.command_id}")
    
    # Serializa para formato binário
    binary_data = command_req.SerializeToString()
    
    # O padrão DEALER envia: [Vazio, Mensagem Binária]
    cmd_socket.send_multipart([b"", binary_data])

    # Aguarda a resposta com timeout de 2 segundos
    if cmd_socket.poll(2000, zmq.POLLIN):
        empty, response_bytes = cmd_socket.recv_multipart()
        response = pb.CommandResponse()
        response.ParseFromString(response_bytes)
        
        print(f"[GS RESPOSTA RECEBIDA] Sucesso: {response.success} | Mensagem: {response.message}")
        return response
    else:
        print("[GS ERRO] Timeout: Nenhuma resposta do OBC.")
        return None

# ============================================================================
# EXECUÇÃO E TESTE DE COMANDOS
# ============================================================================
def main():
    context = zmq.Context()

    # Inicia a escuta da telemetria em segundo plano
    threading.Thread(target=telemetry_receiver_thread, args=(context,), daemon=True).start()

    # Configura o Socket DEALER para envio de comandos
    cmd_socket = context.socket(zmq.DEALER)
    cmd_socket.connect(f"tcp://{OBC_IP}:{ZMQ_CMD_PORT}")
    time.sleep(1)  # Aguarda estabilização da ligação ZMQ

    # ------------------------------------------------------------------------
    # EXEMPLO 1: Definir novo alvo de orientação
    # ------------------------------------------------------------------------
    req1 = pb.CommandRequest()
    req1.command_id = "cmd_001"
    req1.type = pb.CMD_SET_TARGET
    req1.set_target.target_heading = 180.0
    send_command(cmd_socket, req1)
    time.sleep(2)

    # ------------------------------------------------------------------------
    # EXEMPLO 2: Ligar o Aquecedor 1
    # ------------------------------------------------------------------------
    req2 = pb.CommandRequest()
    req2.command_id = "cmd_002"
    req2.type = pb.CMD_CONTROL_HEATER
    req2.control_heater.heater_id = 1
    req2.control_heater.enable = True
    send_command(cmd_socket, req2)
    time.sleep(2)

    # ------------------------------------------------------------------------
    # EXEMPLO 3: Disparar Aquisição do SAR
    # ------------------------------------------------------------------------
    req3 = pb.CommandRequest()
    req3.command_id = "cmd_003"
    req3.type = pb.CMD_TRIGGER_SAR
    send_command(cmd_socket, req3)

    # Mantém o script a correr para acompanhar a telemetria
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nCliente encerrado.")

if __name__ == "__main__":
    main()