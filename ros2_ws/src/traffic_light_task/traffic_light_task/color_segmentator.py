import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class SemaforoDetector(Node):
    def __init__(self):
        super().__init__('semaforo_detector')

        self.loop_rate = self.create_rate(10)

        self.subscription = self.create_subscription(
            Image,
            '/intellicar/ego_vehicle/rgb_front/traffic_light_image',
            self.image_callback,
            10)
        self.subscription 

        self.publisher_ = self.create_publisher(String, '/intellicar/ego_vehicle/traffic_light_type', 10)
        self.bridge = CvBridge()
        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.image is not None:    
            # Convertir la imagen a espacio de color HSV
            hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

            # Definir rangos de colores para el semáforo en HSV
            lower_green = np.array([45, 23, 82])
            upper_green = np.array([72, 245, 255])

            lower_yellow = np.array([25, 25, 80])
            upper_yellow = np.array([44, 255, 255])

            lower_red1 = np.array([0, 70, 110])
            upper_red1 = np.array([25, 255, 255])
            lower_red2 = np.array([165, 70, 110])
            upper_red2 = np.array([180, 255, 255])

            # Segmentar la imagen por cada color
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            # Aplicar operaciones morfológicas
            kernel = np.ones((1,5),np.uint8)
            binary_green = cv2.morphologyEx(mask_green, cv2.MORPH_ERODE, kernel)
            binary_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_ERODE, kernel)
            binary_red = cv2.morphologyEx(mask_red, cv2.MORPH_ERODE, kernel)

            kernel = np.ones((5,1),np.uint8)
            binary_green = cv2.morphologyEx(binary_green, cv2.MORPH_ERODE, kernel)
            binary_yellow = cv2.morphologyEx(binary_yellow, cv2.MORPH_ERODE, kernel)
            binary_red = cv2.morphologyEx(binary_red, cv2.MORPH_ERODE, kernel)

            kernel = np.ones((5,5),np.uint8)
            binary_green = cv2.morphologyEx(binary_green, cv2.MORPH_DILATE, kernel)
            binary_yellow = cv2.morphologyEx(binary_yellow, cv2.MORPH_DILATE, kernel)
            binary_red = cv2.morphologyEx(binary_red, cv2.MORPH_DILATE, kernel)

            # Determinar el tipo de semáforo detectado
            total_green = np.sum(binary_green)
            total_yellow = np.sum(binary_yellow)
            total_red = np.sum(binary_red)

            sem_type = String()
            if total_green > total_yellow and total_green > total_red:
                sem_type.data = "Green"
            elif total_yellow > total_green and total_yellow > total_red:
                sem_type.data = "Yellow"
            elif total_red > total_green and total_red > total_yellow:
                sem_type.data = "Red"
            else:
                sem_type.data = "No"

            # Publicar el tipo de semáforo
            self.publisher_.publish(sem_type)
        
def main(args=None):
    rclpy.init(args=args)
    semaforo_detector = SemaforoDetector()

    rclpy.spin(semaforo_detector)
    semaforo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
