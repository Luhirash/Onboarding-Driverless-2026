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

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Deu ruim ao abrir a câmera ZED")
            sys.exit(1)

        self.image_zed = sl.Mat()
        self.point_cloud = sl.Mat()
        self.runtime_params = sl.RuntimeParameters()

        self.get_logger().info("Iniciando loop de detecção")
        self.timer = self.create_timer(1.0 / 30.0, self.detection_loop)

    def detection_loop(self):
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

            frame_rgba = self.image_zed.get_data()
            frame_bgr = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2BGR)

            results = self.model(frame_bgr, verbose=False)

            msg = Float32MultiArray()
            coordenadas_detectadas = []

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    err, point_cloud_value = self.point_cloud.get_value(u, v)

                    if np.all(np.isfinite(point_cloud_value[:3])):
                        x_3d = point_cloud_value[0]
                        y_3d = point_cloud_value[1]
                        z_3d = point_cloud_value[2]
                        coordenadas_detectadas.extend([x_3d, y_3d, z_3d])

            if coordenadas_detectadas:
                msg.data = coordenadas_detectadas
                self.publisher_.publish(msg)
                self.get_logger().info(f"Coordenadas detectadas: {coordenadas_detectadas}")

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
