#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_callback(ros_image):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    # Display the image
    cv2.imshow("Webcam Feed", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('webcam_display_node', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    
    # Keep the node running
    rospy.spin()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()