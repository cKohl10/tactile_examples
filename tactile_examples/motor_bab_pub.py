import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import random
import math
"""
This script publishes random joint angles to the Franka robot.
"""

class MotorBabblerPublisher(Node):
    def __init__(self):
        super().__init__('motor_babbler_publisher')
        # Joint state publisher
        self.joint_publisher = self.create_publisher(JointState, '/franka/command/joint_states', 10)
        # Timer for triggering image capture
        self.capture_timer = self.create_timer(7.0, self.request_image_capture)
        # Client for image capture service
        self.capture_client = self.create_client(Trigger, '/camera/capture')
        
        self.joint_limits_deg = {
            'panda_joint1': (-166.0, 166.0),
            'panda_joint2': (-101.0, 101.0),
            'panda_joint3': (-166.0, 166.0),
            'panda_joint4': (-176.0, -4.0),
            'panda_joint5': (-166.0, 166.0),
            'panda_joint6': (-1, 215.0),
            'panda_joint7': (-180.0, 180.0)
        }
        # Convert to radians
        self.joint_limits_rad = {k: (math.radians(v[0]), math.radians(v[1])) for k, v in self.joint_limits_deg.items()}
        
        # Flag to track if we're waiting for camera response
        self.waiting_for_camera = False
        
        # Publish initial joint positions
        self.publish_random_joint_angles()

    def publish_random_joint_angles(self):
        joint_msg = JointState()
        joint_msg.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
                         'panda_joint5', 'panda_joint6', 'panda_joint7']
        joint_msg.position = [random.uniform(self.joint_limits_rad[k][0], 
                            self.joint_limits_rad[k][1]) for k in joint_msg.name]
        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f'Published random joint angles: {joint_msg.position}')
        self.waiting_for_camera = False

    async def request_image_capture(self):
        # Only request image if we're not already waiting for a response
        if not self.waiting_for_camera:
            self.waiting_for_camera = True
            
            # Wait for service to be available
            while not self.capture_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Camera capture service not available, waiting...')
            
            # Create request
            request = Trigger.Request()
            
            try:
                # Call service and wait for response
                self.get_logger().info('Requesting image capture...')
                response = await self.capture_client.call_async(request)
                
                if response.success:
                    self.get_logger().info('Image captured successfully')
                    # Generate new random joint angles after successful capture
                    self.publish_random_joint_angles()
                else:
                    self.get_logger().warn(f'Image capture failed: {response.message}')
                    self.waiting_for_camera = False
                    
            except Exception as e:
                self.get_logger().error(f'Service call failed: {str(e)}')
                self.waiting_for_camera = False

def main(args=None):
    rclpy.init(args=args)
    node = MotorBabblerPublisher()
    node.get_logger().info('Motor babbler node started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

