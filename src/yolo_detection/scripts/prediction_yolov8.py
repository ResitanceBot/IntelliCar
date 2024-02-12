#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import supervision as sv
from ultralytics import YOLO

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Subscribers
        rospy.Subscriber("/carla/ego_vehicle/rgb_front/image",Image,self.callback)

        # Publishers
        self.pub = rospy.Publisher('/intellicar/ego_vehicle/rgb_front/image_processed_detection', Image,queue_size=10)
        self.model = YOLO('yolov8n.pt')  # load an official model
        self.model = YOLO("/home/lion/intellicar/src/yolo_detection/model/best_v8.pt")

         # customize the bounding box
        self.box_annotator = sv.BoxAnnotator(
            thickness=2,
            text_thickness=2,
            text_scale=1
        )

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)


    def run(self):

        while not rospy.is_shutdown():

            image_processed = self.image
            if self.image is not None:
                image_processed = cv2.cvtColor(image_processed, cv2.COLOR_RGBA2RGB)
                
                # Realiza predicciones en el cuadro
                results = self.model.predict(image_processed, device=0)[0]
                detections = sv.Detections.from_yolov8(results)

                # Dibuja cajas delimitadoras en el cuadro según las predicciones
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
                self.pub.publish(self.br.cv2_to_imgmsg(image_processed, encoding='rgb8'))
            self.loop_rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node("front_image_processor", anonymous=True)
    my_node = Nodo()
    my_node.run()
