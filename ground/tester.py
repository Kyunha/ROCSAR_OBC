import zmq
import json

ctx = zmq.Context()
sock = ctx.socket(zmq.DEALER)
sock.connect("tcp://localhost:5555")

# Enviar comando para ajustar direção do alvo
sock.send_multipart([b"", json.dumps({"action": "SET_TARGET", "angle": 180.0}).encode()])
print(sock.recv_multipart())
