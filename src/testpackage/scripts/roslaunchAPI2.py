#!/usr/bin/python3

import subprocess
import roslaunch

def count_connected_cameras():
    result = subprocess.run(['ls', '/dev'], stdout=subprocess.PIPE)
    devices = result.stdout.decode().split()
    camera_devices = [device for device in devices if device.startswith('video') and device[-1].isdigit()]
    return len(camera_devices) // 2  # Assuming each camera has two entries

def launch_cameras(camera_count):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    for i in range(camera_count):
        namespace = f"camera_{i+1}_group"
        camera_name = f"camera_{i+1}"
        subscriber_name = f"image_subscriber_{i+1}"
        detection_name = f"yolov5_node_{i+1}"
        camera_ID = i

        # Define the arguments to pass to the launch file
        launch_args = [
            f"namespace:={namespace}",
            f"camera_name:={camera_name}",
            f"camera_ID:={camera_ID}",
            f"subscriber_name:={subscriber_name}",
            f"detection_name:={detection_name}",
            f"image_topic:={camera_name}/image_raw_{i+1}",
            f"detection_topic:={camera_name}/detections_{i+1}",
        ]
        
        launch_file = roslaunch.rlutil.resolve_launch_arguments(['/home/adam/GitRepositories/fgv_test2/src/testpackage/launch/cam4.launch'])[0]
        
        # Create the launch configuration
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
        launch.start()
    
    # Keep the launch process running
    launch.spin()

if __name__ == "__main__":
    camera_count = count_connected_cameras()
    print(f"Number of connected cameras: {camera_count}")
    launch_cameras(camera_count)