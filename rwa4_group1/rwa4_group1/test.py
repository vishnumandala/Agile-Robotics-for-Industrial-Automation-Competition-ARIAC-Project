#!/usr/bin/env python3.8

# # Tutorial link: https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class camera_1(Node):

    def __init__(self):
        super().__init__("camera_1")
        self.left_tray_sub = self.create_subscription(Image, "/ariac/sensors/left_table_camera_rgb/rgb_image", self.callback_tb, 10)

    def callback_tb(self,data):
        # detecting aruco markers
        bridge = CvBridge()
        
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
        main_image = cv_image

        # Load the main image
        main_gray = cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)

        # List of template images
        templates = [
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_00.png', 'ID 0'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_01.png', 'ID 1'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_02.png', 'ID 2'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_03.png', 'ID 3'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_04.png', 'ID 4'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_05.png', 'ID 5'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_06.png', 'ID 6'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_07.png', 'ID 7'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_08.png', 'ID 8'),
            ('/home/attila/Desktop/UMD/Ariac/ariac_ws/src/ariac/ariac_gazebo/models/kit_tray/meshes/markers/id_09.png', 'ID 9')
        ]

        # Loop through the template images and their IDs
        for template_path, template_id in templates:
            template_image = cv2.imread(template_path, 0)  # Load template in grayscale
            w, h = template_image.shape[::-1]  # Get the width and height of the template

            # Perform template matching
            result = cv2.matchTemplate(main_gray, template_image, cv2.TM_CCOEFF_NORMED)

            # Define a threshold for what you consider a "match"
            threshold = 0.5  # Adjust this threshold based on your needs
            locations = np.where(result >= threshold)  # Get the locations of high matching scores
            locations = list(zip(*locations[::-1]))  # Switch x and y locations

            # Draw rectangles around detected markers
            for loc in locations:
                top_left = loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(main_image, top_left, bottom_right, (0, 255, 0), 2)
                cv2.putText(main_image, template_id, (top_left[0], top_left[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Save the result to an image file
        output_image_path = '/mnt/data/detected_templates.png'
        cv2.imwrite(output_image_path, main_image)

        # Optionally display the result
        cv2.imshow('Detected Templates', main_image)
        cv2.waitKey(0)
        

def main():
    rclpy.init()
    node = camera_1()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()



