"""
Nó de Telemetria - Onboarding Driverless
==========================================

Fica no lado da Jetson (dentro do container ROS2 Humble) e faz a ponte
entre o ROS2 e a rede Wi-Fi da oficina:

  1. Assina o tópico 'car_telemetry' (publicado pelo controle_node) e
     reenvia cada mensagem via UDP para o dashboard (no notebook). UDP foi
     escolhido aqui porque é aceitável perder um pacote de telemetria de
     vez em quando -- o próximo já chega logo depois, e latência baixa
     importa mais que garantia de entrega.

  2. Sobe um servidor TCP simples numa porta SEPARADA da telemetria, só
     para o comando de KILL SWITCH. TCP aqui é proposital: o kill switch
     não pode se perder no caminho como um pacote UDP pode, e usar uma
     porta dedicada garante que um pico de tráfego de telemetria não
     atrase esse comando crítico.
     Ao receber "KILL"/"RESET", publica no tópico 'kill_switch' (Bool),
     que o controle_node assina e repassa pro Arduino.

  Ver DECISOES.md para a justificativa completa de UDP (dados) vs TCP (kill)
  e da separação de canais.
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # ---------------- Parâmetros ----------------
        self.declare_parameter('dashboard_ip', '192.168.1.100')  # IP do notebook na rede da oficina
        self.declare_parameter('dashboard_udp_port', 5006)
        self.declare_parameter('kill_tcp_port', 5007)

        self.dashboard_ip = self.get_parameter('dashboard_ip').value
        self.dashboard_udp_port = self.get_parameter('dashboard_udp_port').value
        self.kill_tcp_port = self.get_parameter('kill_tcp_port').value

        # ---------------- Socket UDP (envio de telemetria) ----------------
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ---------------- Assina a telemetria já calculada pelo controle_node ----------------
        self.subscription = self.create_subscription(
            String, 'car_telemetry', self.telemetry_callback, 10)

        # ---------------- Publisher do kill switch (consumido pelo controle_node) ----------------
        self.kill_publisher = self.create_publisher(Bool, 'kill_switch', 10)

        # ---------------- Servidor TCP do kill switch, rodando em outra thread ----------------
        # (roda fora do executor do rclpy porque é um accept() bloqueante;
        #  usar uma thread daemon simples evita ter que integrar com o
        #  MultiThreadedExecutor só por causa disso)
        self._stop_event = threading.Event()
        self.tcp_thread = threading.Thread(target=self._tcp_server_loop, daemon=True)
        self.tcp_thread.start()

        self.get_logger().info(
            f'Telemetry node iniciado -> UDP dados para {self.dashboard_ip}:{self.dashboard_udp_port} '
            f'| TCP kill switch na porta {self.kill_tcp_port}'
        )

    # -------------------------------------------------------------------------
    # Reenvia cada mensagem de telemetria recebida via UDP para o dashboard
    # -------------------------------------------------------------------------
    def telemetry_callback(self, msg):
        try:
            self.udp_socket.sendto(
                msg.data.encode('utf-8'),
                (self.dashboard_ip, self.dashboard_udp_port),
            )
        except OSError as e:
            self.get_logger().warn(
                f'Falha ao enviar telemetria via UDP: {e}', throttle_duration_sec=5.0)

    # -------------------------------------------------------------------------
    # Servidor TCP dedicado do kill switch
    # -------------------------------------------------------------------------
    def _tcp_server_loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.kill_tcp_port))
        server.listen(1)
        server.settimeout(1.0)  # permite checar _stop_event periodicamente sem travar pra sempre

        while not self._stop_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with conn:
                try:
                    data = conn.recv(64).decode('utf-8').strip().upper()
                except Exception:
                    continue

                if data == 'KILL':
                    self.kill_publisher.publish(Bool(data=True))
                    self.get_logger().error(f'KILL SWITCH acionado remotamente por {addr}')
                    conn.sendall(b'OK_KILLED')
                elif data == 'RESET':
                    self.kill_publisher.publish(Bool(data=False))
                    self.get_logger().info(f'Kill switch resetado remotamente por {addr}')
                    conn.sendall(b'OK_RESET')
                else:
                    conn.sendall(b'ERR_UNKNOWN_CMD')

        server.close()

    def destroy_node(self):
        self._stop_event.set()
        self.tcp_thread.join(timeout=2.0)
        self.udp_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()