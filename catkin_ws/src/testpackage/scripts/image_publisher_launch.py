#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys

def publish_image(camera_name, topic_name):
    """Capture frames from a camera and publish it to the specified topic"""
    image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
    bridge = CvBridge()
    capture = cv2.VideoCapture(camera_name)

    while not rospy.is_shutdown():
        # Capture a frame
        ret, img = capture.read()
        if not ret:
            rospy.logerr("Could not grab a frame!")
            break

        # Publish the image to the specified topic
        try:
            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            
            image_pub.publish(img_msg)
            rospy.loginfo(f"Image published to {topic_name}")
        except CvBridgeError as error:
            rospy.logerr(f"Error converting image: {error}")

if __name__ == "__main__":
    rospy.init_node("image_publisher", anonymous=True)
    if len(sys.argv) < 3:
        rospy.logerr("Usage: rosrun testpackage image_publisher.py <camera_name> <topic_name>")
        sys.exit(1)
    camera_name = sys.argv[1]
    topic_name = sys.argv[2]
    rospy.loginfo(f"Publishing images from {camera_name} to {topic_name}")
    publish_image(camera_name, topic_name)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down!")