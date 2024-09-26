#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node('camera_node', anonymous=True)
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)

    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)  # Allow window to be resized

    rate = rospy.Rate(60) # 10hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue

        try:
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(image_message)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Get the current window size
        window_width = cv2.getWindowImageRect('Camera')[2]
        window_height = cv2.getWindowImageRect('Camera')[3]

        # Ensure minimum size constraints
        window_width = max(window_width, 100)
        window_height = max(window_height, 100)

        # Resize the frame to fit the window
        resized_frame = cv2.resize(frame, (window_width, window_height))

        # Explicitly set the window size
        cv2.resizeWindow('Camera', window_width, window_height)

        cv2.imshow('Camera', resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    # def __init__(self):
    #     rospy.init_node('camera_node', anonymous=True)
    #     self.bridge = CvBridge()
    #     self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
    #     cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)  # Allow window to be resized

    # def image_callback(self, data):
    #     try:
    #         frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         rospy.logerr("CvBridge Error: {0}".format(e))
    #         return

    #     # Get the current window size
    #     window_width = cv2.getWindowImageRect('Camera')[2]
    #     window_height = cv2.getWindowImageRect('Camera')[3]

    #     # Ensure minimum size constraints
    #     window_width = max(window_width, 200)
    #     window_height = max(window_height, 200)

    #     # Resize the frame to fit the window
    #     resized_frame = cv2.resize(frame, (window_width, window_height))

    #     cv2.imshow('Camera', resized_frame)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         rospy.signal_shutdown('User requested shutdown')

    # def run(self):
    #     rospy.spin()
    #     cv2.destroyAllWindows()

    # if __name__ == '__main__':
    #     try:
    #         node = CameraNode()
    #         node.run()
    #     except rospy.ROSInterruptException:
    #         pass