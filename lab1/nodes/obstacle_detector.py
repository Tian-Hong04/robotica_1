#!/usr/bin/env python3
# librerias a usar

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge


# primero definimos la clase como un nodo y creamos la variable limit, que representa la distancia maxima a la que estara el 
# robot como maximo
class Obstacle_Detector_Node(Node):

    def __init__(self):
        super().__init__("obstacle_detector")
        self.bridge = CvBridge()
        self.limit = 0.5 + 0.1775  # 0.5 representa la distancia y 0.1775 del centro del robot a su limite
        self.init_communications()

    def init_communications(self):
        self.publisher = self.create_publisher(Vector3, "/occupancy_state", 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self.read_image, 10)

    
    def read_image(self, data):
        # literalmente primero transformamos la informacion a un numpy estilo matriz
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # tomamos la franja de al medio, ya que abajo tenemos el suelo y arriba el cielo
        height = image.shape[0]
        band = image[int(height * 0.4):int(height * 0.65), :]

        # si nos tira nan significa que esta lo suficientemente cerca como para detenerse
        band = np.nan_to_num(band, nan=0.0)

        # dividimos en 3 franjas para asi saber en que direccion esta el obstaculo
        left, center, right = np.array_split(band, 3, axis=1)



        # buscamos lo mas cercano que se ve en cada franja
        min_left = left.min()
        min_center = center.min()
        min_right = right.min()

    
        # revisamos la izquierda
        if min_left <= self.limit:
            left_state = 1.0 # obstaculo
        else:
            left_state = 0.0 # libre

        # revisamos al medio
        if min_center <= self.limit:
            center_state = 1.0 # obstaculo
        else:
            center_state = 0.0 # libre

        # revisamos la derecha
        if min_right <= self.limit:
            right_state = 1.0 # obstaculo
        else:
            right_state = 0.0 #libre

        # creamos el vector y le ponemos la informacion
        occupancy = Vector3()
        occupancy.x = left_state
        occupancy.y = center_state
        occupancy.z = right_state
        self.publisher.publish(occupancy)



def main():
    rclpy.init()
    node = Obstacle_Detector_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()