#!/usr/bin/env python3
"""
Módulo: Servidor ZMQ do On-Board Computer (OBC) - Projeto ROCSAR
Descrição: Processa comandos Protobuf via ZeroMQ ROUTER e publica telemetria via PUB.
"""

import time
import threading
import zmq
import rocsar_messages_pb2 as pb

# ============================================================================
# CONFIGURAÇÕES DE REDE E PORTAS
# ============================================================================
ZMQ_CMD_PORT = 5555  # Porta para receção de comandos (ROUTER)
ZMQ_PUB_PORT = 5556  # Porta para emissão de telemetria (PUB)

# Estado global simulado da telemetria
state_lock = threading.Lock()
g_gondola_heading = 45.0
g_target_heading = 0.0
g_heater1_on = False
g_heater2_on = False
g_sar_running = False

# ============================================================================
# THREAD DE EMISSÃO DE TELEMETRIA (5 Hz)
# ============================================================================
def telemetry_publisher_thread(context):
    """Monta e envia o pacote Protobuf TelemetryFrame a cada 200 ms."""
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(f"tcp://*:{ZMQ_PUB_PORT}")
    print(f"[OBC TELEMETRY] Publicador iniciado na porta {ZMQ_PUB_PORT}")

    start_time = time.time()

    while True:
        frame = pb.TelemetryFrame()
        frame.timestamp = time.time()

        # Copia os dados do estado do sistema de forma segura
        with state_lock:
            frame.gondola_heading = g_gondola_heading
            frame.target_heading = g_target_heading
            frame.imu_online = True
            frame.heater1_active = g_heater1_on
            frame.heater2_active = g_heater2_on

        # Telemetria simulada do Servo 1
        s1 = frame.servos.add()
        s1.id = 1
        s1.current_tick = 2048
        s1.voltage_v = 12.1
        s1.temperature_c = 34
        s1.online = True

        # Telemetria simulada do Servo 2
        s2 = frame.servos.add()
        s2.id = 2
        s2.current_tick = 2050
        s2.voltage_v = 12.0
        s2.temperature_c = 35
        s2.online = True

        # Dados simulados do GNSS
        frame.gnss.latitude = 41.1579
        frame.gnss.longitude = -8.6291
        frame.gnss.altitude = 120.5
        frame.gnss.fix_ok = True

        # Saúde do sistema
        frame.health.cpu_temp_c = 48.5
        frame.health.uptime_seconds = int(time.time() - start_time)
        frame.health.disk_used_percent = 23.4

        # Codifica o pacote Protobuf em bytes
        serialized_payload = frame.SerializeToString()

        # Publica no tópico "TELEMETRY"
        pub_socket.send_multipart([b"TELEMETRY", serialized_payload])
        time.sleep(0.2)  # 200 ms = 5 Hz

# ============================================================================
# SERVIDOR PRINCIPAL DE COMANDOS (ROUTER)
# ============================================================================
def main():
    global g_target_heading, g_heater1_on, g_heater2_on, g_sar_running

    context = zmq.Context()

    # Inicia a thread de telemetria
    threading.Thread(target=telemetry_publisher_thread, args=(context,), daemon=True).start()

    # Configura o Socket ROUTER para receber comandos de múltiplos clientes
    cmd_socket = context.socket(zmq.ROUTER)
    cmd_socket.bind(f"tcp://*:{ZMQ_CMD_PORT}")
    print(f"[OBC COMMANDS] Servidor de comandos ativo na porta {ZMQ_CMD_PORT}")

    while True:
        try:
            # O protocolo ROUTER do ZMQ recebe: [Identidade do Cliente, Vazio, Mensagem]
            identity, empty, binary_msg = cmd_socket.recv_multipart()

            # Descodifica a mensagem Protobuf recebida
            request = pb.CommandRequest()
            request.ParseFromString(binary_msg)

            # Prepara a estrutura de resposta
            response = pb.CommandResponse()
            response.command_id = request.command_id
            response.success = True

            # Processa o tipo de comando recebido
            if request.type == pb.CMD_SET_TARGET:
                new_angle = request.set_target.target_heading
                with state_lock:
                    g_target_heading = new_angle
                response.message = f"Novo alvo de orientação definido para {new_angle}°"
                print(f"[CMD RECEBIDO] Set Target: {new_angle}°")

            elif request.type == pb.CMD_CONTROL_HEATER:
                h_id = request.control_heater.heater_id
                state = request.control_heater.enable
                with state_lock:
                    if h_id == 1:
                        g_heater1_on = state
                    elif h_id == 2:
                        g_heater2_on = state
                response.message = f"Aquecedor {h_id} alterado para {'LIGADO' if state else 'DESLIGADO'}"
                print(f"[CMD RECEBIDO] Aquecedor {h_id} -> {state}")

            elif request.type == pb.CMD_TRIGGER_SAR:
                with state_lock:
                    g_sar_running = True
                response.message = "Aquisição do SAR iniciada com sucesso"
                print("[CMD RECEBIDO] Disparo da aquisição do SAR")

            elif request.type == pb.CMD_SET_BANDWIDTH:
                rate = request.set_bandwidth.rate_kbps
                response.message = f"Limite de banda atualizado para {rate} kbps"
                print(f"[CMD RECEBIDO] Limite de Banda -> {rate} kbps")

            else:
                response.success = False
                response.message = "Comando desconhecido ou não implementado"

            # Responde ao cliente que enviou o comando
            cmd_socket.send_multipart([identity, b"", response.SerializeToString()])

        except Exception as e:
            print(f"[ERRO] Erro ao processar comando: {e}")

if __name__ == "__main__":
    main()