#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys

class ImageFlipper:
    def __init__(self, input_topic, output_topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(input_topic, Image, self.callback)
        self.image_pub = rospy.Publisher(output_topic, Image, queue_size=10)

    def callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Flip the image vertically
        flipped_image = cv2.flip(cv_image, -1)

        try:
            # Convert the OpenCV image back to ROS Image message
            flipped_image_msg = self.bridge.cv2_to_imgmsg(flipped_image, "bgr8")
            # Publish the flipped image
            self.image_pub.publish(flipped_image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('image_flipper', anonymous=True)

    # Get the input and output topics from the command line
    if len(sys.argv) < 3:
        rospy.logerr("Usage: rosrun testpackage image_flipper.py <input_topic> <output_topic>")
        sys.exit(1)
    input_topic = sys.argv[1]
    output_topic = sys.argv[2]
    
    ImageFlipper(input_topic, output_topic)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image_flipper")