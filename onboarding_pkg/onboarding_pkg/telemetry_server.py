"""
Backend do Dashboard - Onboarding Driverless (Telemetria)
============================================================

Roda no NOTEBOOK (fora do container ROS2 -- esse script não usa ROS2).

Ponte entre a Jetson e o navegador:

    Jetson --(UDP, dados)--> [thread UDP]     --(WebSocket)--> navegador
    navegador --(WebSocket, kill switch)--> [conexão TCP]  --> Jetson

Ver DECISOES.md para o porquê de UDP para dados e TCP para o kill switch,
e o porquê de WebSocket (via Flask-SocketIO) em vez de polling HTTP.
"""

import json
import socket
import threading
import time

from flask import Flask, render_template
from flask_socketio import SocketIO

# ============================================================================
# Configuração -- AJUSTE O IP DA JETSON AQUI (ver README.md, seção
# "Configuração de rede", para instruções de como descobrir esse IP)
# ============================================================================
JETSON_IP = "192.168.1.50"        # <-- TROCAR pelo IP da Jetson na rede da oficina
JETSON_KILL_TCP_PORT = 5007       # precisa bater com o parâmetro 'kill_tcp_port' do telemetry_node

UDP_LISTEN_PORT = 5006            # precisa bater com o parâmetro 'dashboard_udp_port' do telemetry_node
WEB_PORT = 8080                   # porta em que o dashboard fica disponível no navegador

TELEMETRY_TIMEOUT_S = 2.0         # sem pacote novo nesse tempo -> dashboard mostra "sem sinal"
# ============================================================================

app = Flask(__name__)
app.config['SECRET_KEY'] = 'onboarding-driverless'
# async_mode='threading': evita depender do eventlet/gevent, mantendo o
# requirements.txt mínimo -- troca um pouco de performance em altíssima
# concorrência por simplicidade de instalação (ver DECISOES.md)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

_last_packet_time = 0.0


# =============================================================================
# Rota principal - serve o dashboard
# =============================================================================
@app.route('/')
def index():
    return render_template('index.html')


# =============================================================================
# Thread UDP: recebe telemetria da Jetson e repassa via WebSocket
# =============================================================================
def udp_listener():
    global _last_packet_time

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', UDP_LISTEN_PORT))
    print(f'[UDP] Escutando telemetria em 0.0.0.0:{UDP_LISTEN_PORT}')

    while True:
        try:
            data, addr = udp_socket.recvfrom(4096)
            payload = json.loads(data.decode('utf-8'))
            _last_packet_time = time.time()
            payload['_connected'] = True
            socketio.emit('telemetry', payload)
        except json.JSONDecodeError:
            print('[UDP] Pacote recebido não é JSON válido, ignorando')
        except Exception as e:
            print(f'[UDP] Erro: {e}')


# =============================================================================
# Watchdog: avisa o navegador quando o sinal da Jetson fica velho
# (cobre o caso do requisito "Estabilidade": se o carro travar/perder a
#  rede, o dashboard tem que deixar isso óbvio, não ficar "congelado" com
#  o último dado bom)
# =============================================================================
def signal_watchdog():
    global _last_packet_time
    while True:
        time.sleep(0.5)
        if _last_packet_time > 0 and (time.time() - _last_packet_time) > TELEMETRY_TIMEOUT_S:
            socketio.emit('connection_status', {'connected': False})


# =============================================================================
# WebSocket: recebe comando de kill switch do navegador e repassa via TCP
# =============================================================================
@socketio.on('kill_switch')
def handle_kill_switch(payload):
    action = payload.get('action', 'kill').upper()  # 'KILL' ou 'RESET'

    try:
        with socket.create_connection((JETSON_IP, JETSON_KILL_TCP_PORT), timeout=2.0) as sock:
            sock.sendall(action.encode('utf-8'))
            response = sock.recv(64).decode('utf-8')
            socketio.emit('kill_switch_ack', {'action': action, 'ok': True, 'response': response})
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        socketio.emit('kill_switch_ack', {
            'action': action,
            'ok': False,
            'response': f'Não consegui falar com a Jetson ({e})',
        })


if __name__ == '__main__':
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=signal_watchdog, daemon=True).start()

    print(f'[WEB] Dashboard disponível em http://0.0.0.0:{WEB_PORT}')
    print(f'[CONFIG] Esperando telemetria da Jetson em UDP:{UDP_LISTEN_PORT}')
    print(f'[CONFIG] Kill switch vai mandar TCP para {JETSON_IP}:{JETSON_KILL_TCP_PORT}')
    socketio.run(app, host='0.0.0.0', port=WEB_PORT)