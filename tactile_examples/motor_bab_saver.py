# This ROS2 node saves the raw motor babbling data to a specified folder
# Assumes 5 cameras only

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime
import sys

class MotorBabblingSaver(Node):
    def __init__(self, save_dir, num_captures):
        super().__init__('motor_babbling_saver')

        # Initialize variables
        self.num_captures = num_captures
        self.camera_topics = {'front': '/rgb/front', 'left': '/rgb/left', 'right': '/rgb/right', 'back': '/rgb/back', 'above': '/rgb/above'}
        self.camera_indices = ['front', 'left', 'right', 'back', 'above']
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Create save directory with timestamp
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.index = 0
        self.save_dir = save_dir + f"/motor_babbling_data_{self.timestamp}"
        self.save_dirs = {camera_index: os.path.join(self.save_dir, camera_index) for camera_index in self.camera_indices}
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Initialize CSV file for joint angles
        self.csv_path = os.path.join(self.save_dir, 'joint_angles.csv')
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'index', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'])
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states', #Changes this to the actual instead of commanded
            self.joint_callback,
            2
        )
        self.joint_angles = [0.0 for _ in range(7)]
        
        # Subscribe to camera topics
        self.save_images = {camera_index: True for camera_index in self.camera_indices}
        self.image_sub = {}
        for camera_index in self.camera_indices:
            self.image_sub[camera_index] = self.create_subscription(
                Image,
                self.camera_topics[camera_index],
                lambda msg, camera_index=camera_index: self.image_callback(msg, camera_index),
                2
            )

        # Create a publisher for the go_ahead topic
        self.go_ahead_pub = self.create_publisher(Bool, '/go_ahead', 1)

        # Create folders for each camera
        for camera_index in self.camera_indices:
            os.makedirs(os.path.join(self.save_dir, camera_index), exist_ok=True)

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

        self.get_logger().info(f'Saved joint angles {self.joint_angles} to {self.csv_path}')
        
    def image_callback(self, msg, camera_index):
        """Save camera image as PNG"""
        if self.save_images[camera_index]:
            try:
                
                # Convert ROS Image message to OpenCV image
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                
                image_path = os.path.join(self.save_dirs[camera_index], f'camera_{camera_index}_{self.index}.png')
                
                # Check if image is empty
                if cv_image.size == 0:
                    self.get_logger().error('Empty image received')
                    return
                
                # Save image
                success = cv2.imwrite(image_path, cv_image)
                
                if not success:
                    self.get_logger().error(f'cv2.imwrite returned False for {image_path}')
                else:
                    self.get_logger().info(f'Saved {camera_index} camera image to {image_path} (Size: {cv_image.size})')

            except Exception as e:
                self.get_logger().error(f'Error saving image: {str(e)}')
                # Print full exception traceback
                import traceback
                self.get_logger().error(traceback.format_exc())

            self.save_images[camera_index] = False

            if all(not self.save_images[camera] for camera in self.camera_indices):
                self.get_logger().info(f'All data saved for capture {self.index}\n\n')
                self.send_go_ahead(True)

    def send_go_ahead(self, status):
        """Send a go ahead message to the go_ahead topic"""
        msg = Bool()
        msg.data = status
        self.go_ahead_pub.publish(msg)

    def capture_callback(self, request, response):
        try:
            self.index += 1
            self.get_logger().info(f'Capturing {self.index}...')
            self.send_go_ahead(False)
            self.joint_save()
            self.save_images = {camera: True for camera in self.camera_topics}
            response.success = True
            response.message = "Saved capture"
        except Exception as e:
            self.get_logger().error(f'Error saving capture: {str(e)}')
            response.success = False
            response.message = "Error saving capture"
        finally:
            
            # Check if we've reached the desired number of captures
            if self.index >= self.num_captures:
                self.get_logger().info(f'Reached {self.num_captures} captures. Shutting down...')
                # Use a timer to shutdown after a brief delay to ensure last capture is saved
                self.create_timer(1.0, self.shutdown_callback)
            
            return response

    def shutdown_callback(self):
        """Callback to shutdown the node"""
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Get the save directory from command line argument
    if len(sys.argv) < 3:
        print("Please provide a save directory path as the first argument and number of captures as the second argument")
        return
        
    save_dir = sys.argv[1]
    num_captures = int(sys.argv[2])
    node = MotorBabblingSaver(save_dir, num_captures)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()