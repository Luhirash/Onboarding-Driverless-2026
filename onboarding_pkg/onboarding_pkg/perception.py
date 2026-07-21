import sys
import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'coordinates', 10)

        self.model = YOLO("best.pt")
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        # Convenção estilo ROS (REP-103): X para frente, Y para esquerda, Z para cima.
        # Isso deixa point_cloud_value[0]=frente e [1]=esquerda, que é exatamente o
        # que o nó de mapping espera para montar (x1,y1) e (x2,y2).
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Deu ruim ao abrir a câmera ZED")
            sys.exit(1)

        self.image_zed = sl.Mat()
        self.point_cloud = sl.Mat()
        self.runtime_params = sl.RuntimeParameters()

        self.get_logger().info("Iniciando loop de detecção")
        self.timer = self.create_timer(1.0 / 30.0, self.detection_loop)

    def detection_loop(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

        frame_rgba = self.image_zed.get_data()
        frame_bgr = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2BGR)

        results = self.model(frame_bgr, verbose=False)

        caixas_detectadas = []  # cada item: (x_frente, y_esquerda)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                err, point_cloud_value = self.point_cloud.get_value(u, v)

                x_3d, y_3d = point_cloud_value[0], point_cloud_value[1]
                if np.all(np.isfinite([x_3d, y_3d])):
                    caixas_detectadas.append((x_3d, y_3d))

        # O nó de mapping espera exatamente 2 caixas ([x1,y1,x2,y2]).
        # Se detectar mais ou menos que 2, ignoramos o frame para não
        # quebrar o unpack do lado do mapping.
        if len(caixas_detectadas) != 2:
            self.get_logger().warn(
                f"Esperava 2 caixas, detectei {len(caixas_detectadas)}. Ignorando frame.",
                throttle_duration_sec=2.0,
            )
            return

        # Ordena por y (esquerda) para manter uma ordem consistente entre frames
        caixas_detectadas.sort(key=lambda c: c[1], reverse=True)

        msg = Float32MultiArray()
        (x1, y1), (x2, y2) = caixas_detectadas
        msg.data = [x1, y1, x2, y2]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Coordenadas detectadas: {msg.data}")

def destroy_node(self):
    if self.zed.is_opened():
        self.zed.close()
        self.get_logger().info("CÂMERA ZED FECHADA")
    super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
