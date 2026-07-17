"""
Simula o envio de telemetria da Jetson, sem precisar do carro nem do ROS2.
Útil para desenvolver/testar o dashboard fora da oficina.

Uso (com o telemetry_server.py já rodando em outro terminal):
    python simulate_telemetry.py
"""
import json
import math
import socket
import time

# Se estiver rodando na MESMA máquina que o telemetry_server.py, deixe 127.0.0.1.
# Se for de outra máquina, troque pelo IP do notebook que roda o backend.
UDP_TARGET = ('127.0.0.1', 5006)  # precisa bater com UDP_LISTEN_PORT do telemetry_server.py

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f'Enviando telemetria simulada para {UDP_TARGET} (Ctrl+C para parar)')

t0 = time.time()

try:
    while True:
        t = time.time() - t0
        payload = {
            'timestamp': time.time(),
            'motor_on': True,
            'killed': False,
            'state': 1,
            'power_pct': 25,
            'steering_deg': 90 + 20 * math.sin(t),
            'x_wp': 2.0 + 0.3 * math.sin(t / 2),
            'y_wp': 0.5 * math.cos(t / 2),
            'dist_to_target': 2.0,
            'serial_connected': True,
        }
        sock.sendto(json.dumps(payload).encode('utf-8'), UDP_TARGET)
        time.sleep(0.2)
except KeyboardInterrupt:
    print('\nParando simulação.')