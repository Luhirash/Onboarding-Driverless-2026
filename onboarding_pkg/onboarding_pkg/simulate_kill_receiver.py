"""
Simula o servidor TCP de kill switch da Jetson (o que o telemetry_node.py
sobe de verdade), só para testar o botão do dashboard fora da oficina.

Uso:
    python simulate_kill_receiver.py
E no telemetry_server.py, aponte JETSON_IP para "127.0.0.1".
"""
import socket

TCP_PORT = 5007  # precisa bater com JETSON_KILL_TCP_PORT do telemetry_server.py

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', TCP_PORT))
server.listen(1)

print(f'Simulando o servidor de kill switch da Jetson em 0.0.0.0:{TCP_PORT}')
print('Ctrl+C para parar')

try:
    while True:
        conn, addr = server.accept()
        with conn:
            data = conn.recv(64).decode('utf-8').strip().upper()
            print(f'[{addr}] comando recebido: {data}')
            if data == 'KILL':
                conn.sendall(b'OK_KILLED')
            elif data == 'RESET':
                conn.sendall(b'OK_RESET')
            else:
                conn.sendall(b'ERR_UNKNOWN_CMD')
except KeyboardInterrupt:
    print('\nParando simulação.')
finally:
    server.close()