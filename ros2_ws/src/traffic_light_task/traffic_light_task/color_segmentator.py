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
            lower_green = np.array([40, 23, 82])
            upper_green = np.array([72, 245, 255])

            lower_yellow = np.array([25, 25, 80])
            upper_yellow = np.array([40, 255, 255])

            lower_red1 = np.array([0, 90, 100])
            upper_red1 = np.array([15, 255, 255])
            lower_red2 = np.array([160, 90, 100])
            upper_red2 = np.array([180, 255, 255])

            # Segmentar la imagen por cada color
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            # Aplicar operaciones morfológicas

            # Cerrar poros
            kernel = np.ones((3,3),np.uint8)
            binary_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
            binary_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
            binary_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

            # Eliminar fondo del borde de la imagen
            kernel = np.ones((1,3),np.uint8)
            binary_green = cv2.morphologyEx(binary_green, cv2.MORPH_ERODE, kernel)
            binary_yellow = cv2.morphologyEx(binary_yellow, cv2.MORPH_ERODE, kernel)
            binary_red = cv2.morphologyEx(binary_red, cv2.MORPH_ERODE, kernel)

            kernel = np.ones((3,1),np.uint8)
            binary_green = cv2.morphologyEx(binary_green, cv2.MORPH_ERODE, kernel)
            binary_yellow = cv2.morphologyEx(binary_yellow, cv2.MORPH_ERODE, kernel)
            binary_red = cv2.morphologyEx(binary_red, cv2.MORPH_ERODE, kernel)

            # Recuperar la imagen
            kernel = np.ones((3,3),np.uint8)
            binary_green = cv2.morphologyEx(binary_green, cv2.MORPH_DILATE, kernel)
            binary_yellow = cv2.morphologyEx(binary_yellow, cv2.MORPH_DILATE, kernel)
            binary_red = cv2.morphologyEx(binary_red, cv2.MORPH_DILATE, kernel)

            kernel = np.ones((3,3),np.uint8)
            binary_green = cv2.morphologyEx(binary_green, cv2.MORPH_CLOSE, kernel)
            binary_yellow = cv2.morphologyEx(binary_yellow, cv2.MORPH_CLOSE, kernel)
            binary_red = cv2.morphologyEx(binary_red, cv2.MORPH_CLOSE, kernel)

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

            # Visualizar por pantalla (depuración)
            # mask_green_display = (mask_green * 255).astype(np.uint8)
            # mask_yellow_display = (mask_yellow * 255).astype(np.uint8)
            # mask_red_display = (mask_red * 255).astype(np.uint8)
            # cv2.imshow('Mask Green', mask_green)
            # cv2.imshow('Mask Yellow', mask_yellow)
            # cv2.imshow('Mask Red', mask_red)

            # binary_green_display = (binary_green * 255).astype(np.uint8)
            # binary_yellow_display = (binary_yellow * 255).astype(np.uint8)
            # binary_red_display = (binary_red * 255).astype(np.uint8)

            # cv2.imshow('Green', binary_green)
            # cv2.imshow('Yellow', binary_yellow)
            # cv2.imshow('Red', binary_red)
            # cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    semaforo_detector = SemaforoDetector()

    rclpy.spin(semaforo_detector)
    # cv2.destroyAllWindows()
    semaforo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
