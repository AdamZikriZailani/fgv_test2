#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SingleCameraNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('single_camera_node', anonymous=True)
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        
        # Subscribe to the image topic published by the libuvc_camera node
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Create a resizable window named 'Camera'
        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
        
        rospy.loginfo("SingleCameraNode initialized")

    def image_callback(self, data):
        rospy.loginfo("Image received")
        try:
            # Convert the ROS Image message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Get the current window size
        window_width = cv2.getWindowImageRect('Camera')[2]
        window_height = cv2.getWindowImageRect('Camera')[3]

        # Ensure minimum size constraints
        window_width = max(window_width, 200)
        window_height = max(window_height, 200)

        # Resize the frame to fit the window
        resized_frame = cv2.resize(frame, (window_width, window_height))

        # Display the resized frame in the 'Camera' window
        cv2.imshow('Camera', resized_frame)
        
        # Check for 'q' key press to shut down the node
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User requested shutdown')

    def run(self):
        rospy.loginfo("SingleCameraNode running")
        # Keep the node running
        rospy.spin()
        
        # Destroy all OpenCV windows when the node shuts down
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # Create an instance of the SingleCameraNode class
        node = SingleCameraNode()
        
        # Run the node
        node.run()
    except rospy.ROSInterruptException:
        pass