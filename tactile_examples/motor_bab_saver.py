# This ROS2 node saves the raw motor babbling data to a specified folder
# Assumes 5 cameras only

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime
import sys

class MotorBabblingSaver(Node):
    def __init__(self, save_dir):
        super().__init__('motor_babbling_saver')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Create save directory with timestamp
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.index = 0
        self.save_dir = save_dir + f"/motor_babbling_data_{self.timestamp}"
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Initialize CSV file for joint angles
        self.csv_path = os.path.join(self.save_dir, 'joint_angles.csv')
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'index', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'])
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/franka/command/joint_states', #Changes this to the actual instead of commanded
            self.joint_callback,
            2
        )
        
        # Subscribe to camera topics
        self.camera_topics = ['/rgb/front', '/rgb/left', '/rgb/right', '/rgb/back', '/rgb/above']
        self.camera_indices = ['front', 'left', 'right', 'back', 'above']
        self.image_sub = {}
        for camera in self.camera_topics:
            self.image_sub[camera] = self.create_subscription(
                Image,
                camera,
                lambda msg, camera=camera: self.image_callback(msg, camera, False),
                2
            )

        # Create service that responds to capture requests to save the current state
        self.srv = self.create_service(Trigger, '/camera/capture', self.capture_callback)

        self.get_logger().info(f'Motor babbling saver initialized. Saving data to {self.save_dir}. Waiting for capture requests...')
        
    def joint_callback(self, msg):
        self.joint_angles = msg.position
    
    def joint_save(self):
        """Save joint angles to CSV file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S.%f')
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, self.index] + list(self.joint_angles))
        
    def image_callback(self, msg, camera_index, save_image):
        """Save camera image as PNG"""
        if save_image:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                image_path = os.path.join(self.save_dir, f'camera_{self.camera_indices[camera_index]}_{self.index}.png')
                
                # Save image
                cv2.imwrite(image_path, cv_image)
                
            except Exception as e:
                self.get_logger().error(f'Error saving image: {str(e)}')

    def capture_callback(self, request, response):
        try:
            self.joint_save()
            i = 0
            for camera in self.camera_indices:
                self.image_callback(i, camera, True)
                i += 1
            response.success = True
            response.message = "Image captured successfully"
            self.get_logger().info(f'Capture {self.index} saved')
        except Exception as e:
            self.get_logger().error(f'Error saving capture: {str(e)}')
            response.success = False
            response.message = "Error saving capture"
        finally:
            # Increment the index
            self.index += 1
            return response

def main(args=None):
    rclpy.init(args=args)
    
    # Get the save directory from command line argument
    if len(sys.argv) < 2:
        print("Please provide a save directory path as the first argument")
        return
        
    save_dir = sys.argv[1]
    node = MotorBabblingSaver(save_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()