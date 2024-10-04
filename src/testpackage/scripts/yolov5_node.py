#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import torch
import cv2
import numpy as np
import sys

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)

# Define the region of interest (ROI) dimensions
ROI_WIDTH = 800
ROI_HEIGHT = 450

class YOLOv5Node:
    def __init__(self, image_topic, detection_topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.image_pub = rospy.Publisher(detection_topic, Image, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Get the dimensions of the camera feed
        camera_height, camera_width, _ = cv_image.shape

        # Calculate the ROI coordinates to be centered and touching the bottom of the camera feed
        x = (camera_width - ROI_WIDTH) // 2
        y = camera_height - ROI_HEIGHT
        w = ROI_WIDTH
        h = ROI_HEIGHT

        # Draw the ROI boundary as a trapezium on the original image
        roi_pts = np.array([
            [x + w // 4, y],          # Top left
            [x + 3 * w // 4, y],      # Top right
            [x + w, y + h],           # Bottom right
            [x, y + h]                # Bottom left
        ], np.int32)
        roi_pts = roi_pts.reshape((-1, 1, 2))
        cv2.polylines(cv_image, [roi_pts], isClosed=True, color=(255, 0, 0), thickness=2)  # Blue polygon

        # Crop the image to the ROI
        roi_image = cv_image[y:y+h, x:x+w]

        # Perform detection on the ROI
        results = model(roi_image)

        # Filter results to only include humans (class 0 in COCO dataset) with confidence > 0.70
        humans = results.xyxy[0][(results.xyxy[0][:, -1] == 0) & (results.xyxy[0][:, 4] > 0.70)]

        # Draw bounding boxes and labels on the original image
        for *box, conf, cls in humans:
            x1, y1, x2, y2 = map(int, box)
            # Define the vertices of the bounding box polygon
            pts = np.array([[x1 + x, y1 + y], [x2 + x, y1 + y], [x2 + x, y2 + y], [x1 + x, y2 + y]], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)  # Green polygon
            label = f"Person: {conf:.2f}"
            cv2.putText(cv_image, label, (x1 + x, y1 + y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the detections
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    image_topic = sys.argv[1]
    detection_topic = sys.argv[2]

    rospy.init_node('yolov5_node', anonymous=True)
    yolov5_node = YOLOv5Node(image_topic, detection_topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLOv5 node")


# import rospy
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge, CvBridgeError
# import torch
# import cv2
# import numpy as np
# import sys

# # Load the YOLOv5 model
# model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)

# # Define the region of interest (ROI)
# ROI = (200, 100, 800, 650)  # Example ROI (x, y, width, height)

# class YOLOv5Node:
#     def __init__(self, image_topic, detection_topic):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
#         self.image_pub = rospy.Publisher(detection_topic, Image, queue_size=10)

#     def callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
#             return

#         # Draw the ROI boundary as a trapezium on the original image
#         x, y, w, h = ROI
#         roi_pts = np.array([
#             [x + w // 4, y],          # Top left
#             [x + 3 * w // 4, y],      # Top right
#             [x + w, y + h],           # Bottom right
#             [x, y + h]                # Bottom left
#         ], np.int32)
#         roi_pts = roi_pts.reshape((-1, 1, 2))
#         cv2.polylines(cv_image, [roi_pts], isClosed=True, color=(255, 0, 0), thickness=2)  # Blue polygon

#         # Create a mask for the trapezium ROI
#         mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
#         cv2.fillPoly(mask, [roi_pts], 255)

#         # Crop the image to the ROI using the mask
#         roi_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
#         roi_image = roi_image[y:y+h, x:x+w]

#         # Perform detection on the ROI
#         results = model(roi_image)

#         # Filter results to only include humans (class 0 in COCO dataset) with confidence > 0.60
#         humans = results.xyxy[0][(results.xyxy[0][:, -1] == 0) & (results.xyxy[0][:, 4] > 0.50)]

#         # Draw bounding boxes and labels on the original image
#         for *box, conf, cls in humans:
#             x1, y1, x2, y2 = map(int, box)
#             cv2.rectangle(cv_image, (x1 + x, y1 + y), (x2 + x, y2 + y), (0, 255, 0), 2)  # Green bounding boxes
#             label = f"Person: {conf:.2f}"
#             cv2.putText(cv_image, label, (x1 + x, y1 + y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#         # Publish the detections
#         try:
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#         except CvBridgeError as e:
#             rospy.logerr(e)

# if __name__ == '__main__':
#     image_topic = sys.argv[1]
#     detection_topic = sys.argv[2]

#     rospy.init_node('yolov5_node', anonymous=True)
#     yolov5_node = YOLOv5Node(image_topic, detection_topic)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo("Shutting down YOLOv5 node")
