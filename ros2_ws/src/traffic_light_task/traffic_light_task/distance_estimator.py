import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from intellicar_interfaces.msg import BboxTrafficLight
from cv_bridge import CvBridge
import cv2
import numpy as np

class DistanceDetector(Node):
    def __init__(self):
        super().__init__('distance_detector')

        self.loop_rate = self.create_rate(10)

        # Subscriptors
        self.subscription = self.create_subscription(
            Image,
            '/depth',
            self.depth_callback,
            10)
        self.subscription 

        self.subs_bbox = self.create_subscription(
            BboxTrafficLight,
            '/BboxTrafficLight',
            self.bbox_callback,
            10)
        self.subs_bbox 

        # Publishers
        self.publisher_ = self.create_publisher(Int32, '/dist_est', 10)

        self.bridge = CvBridge()

        # Params
        self.depht_image = None
        self.bbox = None

    def depth_callback(self, msg):
        self.depht_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    
    def bbox_callback(self, msg):
        self.bbox = msg

    def start(self):
        while rclpy.ok():
            if self.image is not None:    

                dist_msg = Int32()
                dist_msg.data = 0
                # Publicar distancia estimada
                self.publisher_.publish(dist_msg)
            self.loop_rate.sleep()
        
def main(args=None):
    rclpy.init(args=args)
    distance_detector = DistanceDetector()
    distance_detector.start()

    # rclpy.spin(distance_detector)
    # distance_detector.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
