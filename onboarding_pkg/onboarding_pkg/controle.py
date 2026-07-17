"""
Nó de Controle - Onboarding Driverless
=======================================

Pipeline: percepcao -> mapeamento -> CONTROLE (paralelo: telemetria)

Esse nó:
  1. Assina o tópico 'waypoint' (Float32MultiArray: [x_wp, y_wp]), publicado
     pelo nó de mapping. x_wp e y_wp estão no referencial do carro
     (aproximado pelo referencial da ZED, que fica ~na origem do carro):
         x -> para frente
         y -> para a esquerda
  2. Modela o carro com o modelo bicicleta e calcula:
       - Controle lateral: Pure Pursuit -> ângulo de steering
       - Controle longitudinal: velocidade constante (mais simples para o
         onboarding) com PARADA automática ao chegar perto do alvo.
  3. Envia o comando por Serial (USB) para o Arduino, seguindo o protocolo:

         5 caracteres ASCII: [steering hex 2c][estado 1c][potência hex 2c]

     (ver v3.ino para detalhes do protocolo e o firmware que decodifica isso)

  4. Assina 'kill_switch' (Bool), publicado pelo telemetry_node quando o
     botão do dashboard é apertado. Enquanto killed=True, esse nó IGNORA a
     navegação normal e fica mandando o comando de corte pro Arduino.

  5. Publica 'car_telemetry' (String/JSON) periodicamente, com o status do
     motor e outros dados -- consumido pelo telemetry_node, que repassa
     pro dashboard via UDP. Importante: os dados de telemetria vêm do
     ÚLTIMO COMANDO que este próprio nó mandou pro Arduino (não é uma
     leitura de sensor real) -- decisão para não mexer no protocolo serial
     já testado. Ver DECISOES.md.

  Também funciona como "watchdog": se nenhum waypoint novo chegar dentro de
  um tempo limite, o carro para sozinho (segurança).
"""

import json
import math
import time

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String


class ControleNode(Node):
    def __init__(self):
        super().__init__('controle_node')

        # ---------------- Parâmetros (ajustáveis via ROS2 params) ----------------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Modelo bicicleta
        self.declare_parameter('wheelbase', 0.26)             # distância entre eixos [m]

        # Steering
        self.declare_parameter('steering_center_deg', 90.0)   # ângulo do servo "reto"
        self.declare_parameter('steering_max_delta_deg', 30.0)  # máx. esterçamento (+-)
        self.declare_parameter('steering_invert', False)      # inverte sinal se a direção
                                                                # ficar espelhada na prática

        # Controle longitudinal (velocidade constante para o onboarding)
        self.declare_parameter('cruise_power_pct', 25.0)      # potência 0-100%
        self.declare_parameter('arrival_radius', 0.30)        # [m] considera "chegou" e para

        # Segurança
        self.declare_parameter('waypoint_timeout', 1.0)       # [s] sem waypoint novo -> para
        self.declare_parameter('control_rate_hz', 20.0)       # frequência do loop de controle

        # Telemetria
        self.declare_parameter('telemetry_rate_hz', 10.0)     # frequência de publicação de telemetria

        # ---------------- Lendo os parâmetros ----------------
        self.wheelbase = self.get_parameter('wheelbase').value
        self.steering_center_deg = self.get_parameter('steering_center_deg').value
        self.steering_max_delta_deg = self.get_parameter('steering_max_delta_deg').value
        self.steering_invert = self.get_parameter('steering_invert').value
        self.cruise_power_pct = self.get_parameter('cruise_power_pct').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value

        control_rate_hz = self.get_parameter('control_rate_hz').value
        telemetry_rate_hz = self.get_parameter('telemetry_rate_hz').value

        # ---------------- Conexão serial com o Arduino ----------------
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)  # Arduino reinicia ao abrir a serial, precisa de um tempo
            self.get_logger().info(f'Conectado ao Arduino em {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(
                f'Não foi possível abrir a serial em {port}: {e}. '
                'O nó vai rodar sem enviar comandos até a conexão ser corrigida.'
            )

        # ---------------- Assinatura do waypoint (vem do mapping) ----------------
        self.subscription = self.create_subscription(
            Float32MultiArray, 'waypoint', self.waypoint_callback, 10)

        self.x_wp = None
        self.y_wp = None
        self.last_waypoint_time = None

        # ---------------- Kill switch (vem do telemetry_node) ----------------
        self.killed = False
        self.kill_subscription = self.create_subscription(
            Bool, 'kill_switch', self.kill_switch_callback, 10)

        # ---------------- Telemetria (vai para o telemetry_node) ----------------
        self.telemetry_publisher = self.create_publisher(String, 'car_telemetry', 10)

        # Último comando efetivamente mandado pro Arduino (fonte da telemetria)
        self.last_state = 0
        self.last_power_pct = 0
        self.last_steering_deg = int(round(self.steering_center_deg))

        # ---------------- Timers ----------------
        self.control_timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)
        self.telemetry_timer = self.create_timer(1.0 / telemetry_rate_hz, self.publish_telemetry)

        self.get_logger().info('Controle node iniciado')

    # -------------------------------------------------------------------------
    # Callback do tópico 'waypoint'
    # -------------------------------------------------------------------------
    def waypoint_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('Mensagem de waypoint incompleta, ignorando')
            return

        self.x_wp = msg.data[0]
        self.y_wp = msg.data[1]
        self.last_waypoint_time = self.get_clock().now()

    # -------------------------------------------------------------------------
    # Callback do tópico 'kill_switch'
    # -------------------------------------------------------------------------
    def kill_switch_callback(self, msg):
        was_killed = self.killed
        self.killed = bool(msg.data)

        if self.killed and not was_killed:
            self.get_logger().error('KILL SWITCH ACIONADO -- motor cortado e travado')
        elif not self.killed and was_killed:
            self.get_logger().info('Kill switch resetado -- destravando o Arduino')
            # Manda o frame de reset (estado 8) imediatamente pro Arduino destravar,
            # em vez de esperar o próximo tick do control_loop.
            self.send_command(self.steering_center_deg, state=8, power_pct=0)

    # -------------------------------------------------------------------------
    # Loop principal de controle
    # -------------------------------------------------------------------------
    def control_loop(self):
        if self.serial_conn is None:
            return

        # Kill switch tem prioridade absoluta sobre qualquer outra lógica de navegação
        if self.killed:
            self.send_command(self.steering_center_deg, state=9, power_pct=0)
            return

        # Nenhum waypoint recebido ainda
        if self.x_wp is None or self.last_waypoint_time is None:
            self.send_command(self.steering_center_deg, state=0, power_pct=0)
            return

        # Watchdog: sem waypoint novo há muito tempo -> parar por segurança
        elapsed = (self.get_clock().now() - self.last_waypoint_time).nanoseconds / 1e9
        if elapsed > self.waypoint_timeout:
            self.get_logger().warn(
                'Waypoint expirado, parando o carro por segurança',
                throttle_duration_sec=2.0,
            )
            self.send_command(self.steering_center_deg, state=0, power_pct=0)
            return

        x_wp, y_wp = self.x_wp, self.y_wp
        dist = math.hypot(x_wp, y_wp)

        # Chegou perto o suficiente do ponto médio entre as caixas -> parar
        if dist <= self.arrival_radius:
            self.get_logger().info('Alvo alcançado, parando', throttle_duration_sec=2.0)
            self.send_command(self.steering_center_deg, state=0, power_pct=0)
            return

        # Controle lateral (Pure Pursuit)
        steering_deg = self.pure_pursuit(x_wp, y_wp, dist)

        # Controle longitudinal (velocidade/potência constante)
        power_pct = self.cruise_power_pct

        self.send_command(steering_deg, state=1, power_pct=power_pct)

    # -------------------------------------------------------------------------
    # Pure Pursuit: calcula o ângulo de steering a partir do waypoint alvo
    # -------------------------------------------------------------------------
    def pure_pursuit(self, x_wp, y_wp, dist):
        """
        x_wp: distância à frente do carro até o alvo [m]
        y_wp: deslocamento lateral (esquerda positivo) até o alvo [m]
        dist: distância euclidiana até o alvo (== "Ld", lookahead distance)
        """
        # alpha: ângulo entre o eixo longitudinal do carro e a reta até o alvo
        alpha = math.atan2(y_wp, x_wp)

        # Fórmula clássica do Pure Pursuit (modelo bicicleta):
        #   delta = atan(2 * L * sin(alpha) / Ld)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), dist)
        delta_deg = math.degrees(delta)

        if self.steering_invert:
            delta_deg = -delta_deg

        # Satura no máximo esterçamento físico do carrinho
        delta_deg = max(-self.steering_max_delta_deg,
                        min(self.steering_max_delta_deg, delta_deg))

        steering_deg = self.steering_center_deg + delta_deg
        steering_deg = max(0.0, min(180.0, steering_deg))  # limite físico do servo

        return steering_deg

    # -------------------------------------------------------------------------
    # Monta e envia o comando serial no protocolo esperado pelo Arduino
    # -------------------------------------------------------------------------
    def send_command(self, steering_deg, state, power_pct):
        if self.serial_conn is None:
            return

        steering_byte = int(round(steering_deg))
        steering_byte = max(0, min(255, steering_byte))

        power_byte = int(round(power_pct))
        power_byte = max(0, min(100, power_byte))  # 0-100%, o Arduino faz o map p/ PWM

        # 5 caracteres: 2 (steering hex) + 1 (estado) + 2 (potência hex)
        msg = f'{steering_byte:02X}{int(state)}{power_byte:02X}'

        # Guarda o último comando enviado -- é essa informação que vira telemetria,
        # em vez de ler de volta algum sensor do Arduino (ver DECISOES.md)
        self.last_state = int(state)
        self.last_power_pct = power_byte
        self.last_steering_deg = steering_byte

        try:
            self.serial_conn.write(msg.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Erro ao escrever na serial: {e}')

    # -------------------------------------------------------------------------
    # Publica o estado atual do carro em 'car_telemetry' (consumido pelo
    # telemetry_node, que repassa pro dashboard via UDP)
    # -------------------------------------------------------------------------
    def publish_telemetry(self):
        motor_on = (self.last_state in (1, 2)) and (self.last_power_pct > 0) and not self.killed

        dist_to_target = None
        if self.x_wp is not None and self.y_wp is not None:
            dist_to_target = math.hypot(self.x_wp, self.y_wp)

        payload = {
            'timestamp': time.time(),
            'motor_on': motor_on,
            'killed': self.killed,
            'state': self.last_state,
            'power_pct': self.last_power_pct,
            'steering_deg': self.last_steering_deg,
            'x_wp': self.x_wp,
            'y_wp': self.y_wp,
            'dist_to_target': dist_to_target,
            'serial_connected': self.serial_conn is not None,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.telemetry_publisher.publish(msg)

    # -------------------------------------------------------------------------
    # Ao encerrar o nó, garante que o carro pare antes de fechar a serial
    # -------------------------------------------------------------------------
    def destroy_node(self):
        if self.serial_conn is not None:
            try:
                self.send_command(self.steering_center_deg, state=0, power_pct=0)
                time.sleep(0.05)
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControleNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()