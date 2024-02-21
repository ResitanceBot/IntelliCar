#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from intellicar_interfaces.msg import BboxTrafficLight
import cv2
import supervision as sv
from ultralytics import YOLO

class Nodo(Node):
    def __init__(self):
        super().__init__('prediction_node')

        # Params
        self.image = None
        self.br = CvBridge()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.run)

        # Subscribers
        self.subscription = self.create_subscription(Image, '/carla/ego_vehicle/rgb_front/image', self.callback, 10)

        # Publishers
        self.pub_image = self.create_publisher(Image, '/intellicar/ego_vehicle/rgb_front/image_processed_detection', 10)
        self.pub_bbox = self.create_publisher(BboxTrafficLight, '/intellicar/ego_vehicle/bbox', 10)
        self.pub_image_crop = self.create_publisher(Image, '/intellicar/ego_vehicle/rgb_front/traffic_light_image', 10)

        self.model = YOLO('yolov8n.pt')  # load an official model
        self.model = YOLO("/home/lion/intellicar/ros2_ws/src/yolo_detection/model/best_v8.pt")

        # customize the bounding box
        self.box_annotator = sv.BoxAnnotator(
            thickness=2,
            text_thickness=2,
            text_scale=1
        )

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def run(self):
        image_processed = self.image
        if self.image is not None:
            image_processed = cv2.cvtColor(image_processed, cv2.COLOR_RGBA2RGB)

            # Realiza predicciones en el cuadro
            results = self.model.predict(image_processed, device=0, verbose=False)[0]
            boxes = results.boxes.cpu().numpy().xywh
            classes = results.boxes.cpu().numpy().cls
            bbox_msg = BboxTrafficLight()          
            for n in range(0,len(boxes)):
                target_box = boxes[n]
                id = classes[n]
                # traffic_ligts = '3' and bigger bbox
                if(id==3 and (target_box[2]*target_box[3] > bbox_msg.width*bbox_msg.height)):
                    bbox_msg.x = int(target_box[0])
                    bbox_msg.y = int(target_box[1])
                    bbox_msg.width = int(target_box[2])
                    bbox_msg.height = int(target_box[3])
                

            # Dibuja cajas delimitadoras en el cuadro según las predicciones
            detections = sv.Detections.from_ultralytics(results)
            labels = [
                f"{self.model.model.names[class_id]} {confidence:0.2f}"
                for _, _, confidence, class_id, _
                in detections
            ]
            image_processed = self.box_annotator.annotate(
                scene=image_processed,
                detections=detections,
                labels=labels
            )

            image_processed = cv2.cvtColor(image_processed, cv2.COLOR_RGB2BGR)
            self.pub_image.publish(self.br.cv2_to_imgmsg(image_processed, encoding='rgb8'))
            if(bbox_msg.width*bbox_msg.height != 0):
                image_crop = self.image[(bbox_msg.y-int(bbox_msg.height/2)):(bbox_msg.y+int(bbox_msg.height/2)) \
                                             , (bbox_msg.x-int(bbox_msg.width/2)):(bbox_msg.x+int(bbox_msg.width/2))]
                image_crop = cv2.cvtColor(image_crop, cv2.COLOR_RGB2BGR)
                self.pub_bbox.publish(bbox_msg)
                self.pub_image_crop.publish(self.br.cv2_to_imgmsg(image_crop, encoding='rgb8'))

def main(args=None):
    rclpy.init(args=args)
    nodeObject = Nodo()

    rclpy.spin(nodeObject)
    nodeObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
