import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class MappingNode(Node):
    def __init__ (self):
        # Inicializar a classe mãe
        super().__init__('mapping_node')

        # Criar subscriber que recebe o tópico "coordinates":
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'coordinates',
            self.coordinates_callback,
            10)
        
        # Criar um publisher que envia o tópico "waypoint" para Controle:
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'waypoint',
            10)

        # Incializando a janela do espaço geométrico enquanto roda o código:
        plt.ion()
        self.fig, self.ax = plt.subplots() # Cria a janela
        self.get_logger().info(" Mapping node iniciado ")
        
    def coordinates_callback(self, msg):
        """mensagens do tópico coordinates são direcionadas para essa função 
            em forma de msg.data"""
        data = msg.data # Essa mensagem chega como uma lista de coordenadas [x1, y1, x2, y2]
        x1, y1, x2, y2 = data
        
        x_wp, y_wp = (x1+x2)/2, (y1+y2)/2

        self.publish_waypont(x_wp, y_wp)

        self.update_plot(x1, y1, x2, y2, x_wp, y_wp)
    
    def publish_waypont(self, x_wp, y_wp):
        msg = Float32MultiArray() # Cria uma mensagem vazia no formato que ROS lê
        msg.data = [x_wp, y_wp]
        self.publisher.publish(msg) # Publicar mensagem com tópico "waypoint" para Controle
        self.get_logger().info(f"Waypoint publicado: X={x_wp:.2f}, Y={y_wp:.2f}")

    def update_plot(self, x1, y1, x2, y2, x_wp, y_wp):
        # Limpar gráfico
        self.ax.clear()

        # Plotar os pontos dos cones e do waypoint
        self.ax.scatter(x1, y1, label="Esquerda")

        self.ax.scatter(x2, y2, label="Direita")

        self.ax.scatter(x_wp, y_wp, label="Waypoint")

        # Plotar o carro:
        self.ax.scatter(0, 0, 0, marker="x", s=100, color="red", label="Carro") # s: size

        # Configurações e definições:
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_title("Mapa Local")
        self.ax.legend()
        self.ax.grid(True)
        self.ax.axis('equal')

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()

    try:
        rclpy.spin(node)

    except(KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()