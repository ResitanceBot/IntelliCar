#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import supervision as sv
import yolov7

class FrontImageProcessor(Node):
    def __init__(self):
        super().__init__('front_image_processor')

        # Params
        self.image = None
        self.br = CvBridge()

        # Node cycle rate (in Hz).
        self.loop_rate = self.create_rate(10)

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_front/image',
            self.image_callback,
            10
        )

        # Publishers
        self.pub = self.create_publisher(
            Image,
            '/intellicar/ego_vehicle/rgb_front/image_processed_detection',
            10
        )

        self.model = yolov7.load("/home/lion/intellicar/src/yolo_detection/model/best.pt", device='cuda')
        self.model.conf = 0.1  # NMS confidence threshold

        # customize the bounding box
        self.box_annotator = sv.BoxAnnotator(
            thickness=2,
            text_thickness=2,
            text_scale=1
        )

    def image_callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def run(self):
        while rclpy.ok():
            results = self.model(self.image)
            detections = sv.Detections.from_yolov5(results)

            image_processed = self.image
            if self.image is not None:
                image_processed = self.box_annotator.annotate(
                    scene=image_processed,
                    detections=detections
                )

                image_processed = cv2.cvtColor(image_processed, cv2.COLOR_RGB2BGR)
                self.pub.publish(self.br.cv2_to_imgmsg(image_processed, encoding='rgb8'))
            
            rclpy.spin_once(self)
            self.loop_rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    front_image_processor = FrontImageProcessor()
    front_image_processor.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
