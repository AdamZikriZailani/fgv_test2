#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys

def publish_image(camera_name, topic_name, frame_rate):
    """Capture frames from a camera and publish it to the specified topic"""
    image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
    bridge = CvBridge()
    capture = cv2.VideoCapture(camera_name)
    rate = rospy.Rate(frame_rate)

    while not rospy.is_shutdown():
        # Capture a frame
        ret, img = capture.read()
        if not ret:
            rospy.logerr("Could not grab a frame!")
            break

        # Determine the size of the square
        height, width, _ = img.shape
        size = min(height, width)

        # Crop the image to a square
        start_x = (width - size) // 2
        start_y = (height - size) // 2
        img = img[start_y:start_y + size, start_x:start_x + size]

        # Rotate the image 90 degrees to the left
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

        # Publish the image to the specified topic
        try:
            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            image_pub.publish(img_msg)
            rospy.loginfo(f"Image published to {topic_name}")
        except CvBridgeError as error:
            rospy.logerr(f"Error converting image: {error}")

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("image_publisher", anonymous=True)
    if len(sys.argv) < 3:
        rospy.logerr("Usage: rosrun testpackage image_publisher.py <camera_name> <topic_name>")
        sys.exit(1)
    camera_name = sys.argv[1]
    topic_name = sys.argv[2]
    frame_rate = rospy.get_param("~frame_rate", 10)  # Default frame rate is 10 FPS
    rospy.loginfo(f"Publishing images from {camera_name} to {topic_name} at {frame_rate} FPS")
    publish_image(camera_name, topic_name, frame_rate)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down!")