#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Display the image
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr(f"Error converting image: {e}")



if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun testpackage image_subscriber.py <topic_name>")
        sys.exit(1)
    topic_name = sys.argv[1]
    rospy.Subscriber(topic_name, Image, image_callback)
    rospy.loginfo(f"Subscribed to the topic {topic_name}")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down!")
        cv2.destroyAllWindows()