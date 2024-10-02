#!/usr/bin/python3

import roslaunch
import cv2

#To detect how many cameras are connected to the system automatically
def detect_cameras():
    index = 0
    arr = []
    while True:
        cap = cv2.VideoCapture(index)
        if not cap.read()[0]:
            break
        else:
            arr.append(index)
        cap.release()
        index += 1
    return arr

def launch_cameras():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    launch_files = []
    launch_args = []

    for i, camera_index in enumerate(detect_cameras()):
        namespace = f"camera_{i+1}_group"
        camera_name = f"camera_{i+1}"
        subscriber_name = f"image_subscriber_{i+1}"
        detection_name = f"yolov5_node_{i+1}"

        
        # Define the arguments to pass to the launch file
        launch_args.extend([
            f"namespace:={namespace}",
            f"camera_name:={camera_name}",
            f"camera_ID:={i}",
            f"subscriber_name:={subscriber_name}",
            f"detection_name:={detection_name}",
            f"image_topic:={camera_name}/image_raw_{i+1}",
            f"detection_topic:={camera_name}/detections_{i+1}",

        ])
        
        launch_files.append("/home/adam/GitRepositories/fgv_test2/src/testpackage/launch/cam4.launch")
    
    # Create the launch configuration
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    
    # Set the parameters
    for arg in launch_args:
        key, value = arg.split(":=")
        roslaunch.core.Param(key, value)
    
    launch.start()
    launch.spin()

if __name__ == '__main__':    
    launch_cameras()