import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from intellicar_interfaces.msg import BboxTrafficLight
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from math import asin, sin, degrees, radians, cos

class DistanceDetector(Node):
    def __init__(self):
        super().__init__('distance_detector')

        # Subscriptors
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/depth_front/image',
            self.depth_callback,
            10)
        self.subscription 

        self.subs_bbox = self.create_subscription(
            BboxTrafficLight,
            '/intellicar/ego_vehicle/bbox',
            self.bbox_callback,
            10)
        self.subs_bbox 

        # Publishers
        self.publisher_ = self.create_publisher(Float32, '/intellicar/ego_vehicle/traffic_light/dist_est', 10)

        self.bridge = CvBridge()

        # Params
        self.depht_image = None
        self.bbox = None

    def depth_callback(self, msg):
        self.depht_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        # gray_image = cv2.cvtColor(self.depht_image, cv2.COLOR_BGR2GRAY)
        # color_map = cv2.applyColorMap(gray_image, cv2.COLORMAP_JET)
        # cv2.imshow('Mapa de Calor', color_map)
        # cv2.waitKey(1)
    
    def bbox_callback(self, msg):
        self.bbox = msg

        if self.depht_image is not None:
            dist_pixel_central = self.depht_image[self.bbox.y,self.bbox.x]
            angles = angular_position((800, 600),self.bbox.x,self.bbox.y)
            yaw = angles[1]
            pitch = angles[0]
            print("yaw:",yaw)
            print("pitch:",pitch)
            dist_proy_est = (dist_pixel_central*cos(pitch*math.pi/180.0))*cos(yaw*math.pi/180)

            # Proyeccion de la distancia
            dist_msg = Float32()
            dist_msg.data = float(dist_proy_est)
            # Publicar distancia estimada
            self.publisher_.publish(dist_msg)
        
def main(args=None):
    rclpy.init(args=args)
    distance_detector = DistanceDetector()

    rclpy.spin(distance_detector)
    distance_detector.destroy_node()
    rclpy.shutdown()

def angular_position(shape,y,x,fov=(60,49.5)):
    x_c = x-(shape[1]-1)/2
    y_c = (shape[0]-1)/2-y
    w_c = (shape[1]+1)/2
    h_c = (shape[0]+1)/2
    theta_max = fov[0]/2
    phi_max = fov[1]/2
    theta = degrees(asin(sin(radians(theta_max)/w_c*x_c)))
    phi = degrees(asin(sin(radians(phi_max)/h_c*y_c)))
    return theta,phi

if __name__ == '__main__':
    main()
