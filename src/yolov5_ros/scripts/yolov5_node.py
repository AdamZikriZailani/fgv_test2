#!/usr/bin/python3

import rospy
import torch
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys


class YOLOv5Node:
    def __init__(self, image_topic, detection_topic):
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.classes = [0]
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.image_pub = rospy.Publisher(detection_topic, Image, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        results = self.model(cv_image)
        detections = results.render()[0]

        try:
            detection_msg = self.bridge.cv2_to_imgmsg(detections, "bgr8")
            self.image_pub.publish(detection_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        rospy.logerr("Usage: yolov5_node.py <image_topic> <detection_topic>")
        sys.exit(1)

    image_topic = sys.argv[1]
    detection_topic = sys.argv[2]

    rospy.init_node('yolov5_node', anonymous=True)
    yolov5_node = YOLOv5Node(image_topic, detection_topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLOv5 node")



##--working but detects everything--#

# class YOLOv5Node:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
#         self.image_sub = rospy.Subscriber("/image_raw_1", Image, self.image_callback)
#         self.image_pub = rospy.Publisher("/yolov5/detections", Image, queue_size=10)

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")
#             return

#         results = self.model(cv_image)
#         detections = results.render()[0]

#         try:
#             detection_msg = self.bridge.cv2_to_imgmsg(detections, "bgr8")
#             self.image_pub.publish(detection_msg)
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")

# if __name__ == '__main__':
#     rospy.init_node('yolov5_node', anonymous=True)
#     yolov5_node = YOLOv5Node()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo("Shutting down YOLOv5 node")


#---working for detect human but highly ineffecient--#

# class YOLOv5Node:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
#         self.image_sub = rospy.Subscriber("/image_raw_1", Image, self.image_callback)
#         self.image_pub = rospy.Publisher("/yolov5/detections", Image, queue_size=10)
#         self.human_class_id = 0  # Class ID for 'person' in COCO dataset

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")
#             return

#         results = self.model(cv_image)
#         detections = results.xyxy[0]  # Get the detections in xyxy format

#         # Filter detections to keep only humans
#         human_detections = [d for d in detections if int(d[5]) == self.human_class_id]

#         # Draw bounding boxes for human detections
#         for det in human_detections:
#             x1, y1, x2, y2, conf, cls = det
#             cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
#             cv2.putText(cv_image, f'Human {conf:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

#         try:
#             detection_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
#             self.image_pub.publish(detection_msg)
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")

# if __name__ == '__main__':
#     rospy.init_node('yolov5_node', anonymous=True)
#     yolov5_node = YOLOv5Node()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo("Shutting down YOLOv5 node")