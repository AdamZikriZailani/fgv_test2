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

    # Set the camera resolution to 1080 by 1920
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280.0)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720.0)

    # Verify if the resolution is set correctly
    actual_width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    rospy.loginfo(f"Requested resolution: 1920x1080, Actual resolution: {actual_width}x{actual_height}")

    rate = rospy.Rate(frame_rate)



    while not rospy.is_shutdown():
        # Capture a frame
        ret, img = capture.read()
        if not ret:
            rospy.logerr("Could not grab a frame!")
            break

        # Capture the full frame of the camera
        # height, width, _ = img.shape
        # rospy.loginfo(f"Captured frame of size: {width}x{height}")

        # Rotate the image 180 degrees
        img = cv2.rotate(img, cv2.ROTATE_180)

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
    frame_rate = int(sys.argv [3])
    #frame_rate = rospy.get_param("~frame_rate", 10)  # Default frame rate is 10 FPS
    rospy.loginfo(f"Publishing images from {camera_name} to {topic_name} at {frame_rate} FPS")
    publish_image(camera_name, topic_name, frame_rate)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down!")