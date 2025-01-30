import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import random
import math
import time
import asyncio
"""
This script publishes random joint angles to the Franka robot.
"""

class MotorBabblerPublisher(Node):
    def __init__(self):
        super().__init__('motor_babbler_publisher')
        # Joint state publisher
        self.joint_publisher = self.create_publisher(JointState, '/franka/command/joint_states', 1)

        # Create a subscriber that will give the go ahead to move on to the next capture
        self.go_ahead_subscriber = self.create_subscription(Bool, '/go_ahead', self.go_ahead_callback, 1)
        self.go_ahead = True
        
        # List of camera service names
        self.camera_services = ['/camera/capture']  # Add more camera services as needed
        # Create service clients for each camera
        self.capture_clients = {
            service: self.create_client(Trigger, service)
            for service in self.camera_services
        }
        
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
        self.joint_limits_rad = {k: (math.radians(v[0]), math.radians(v[1])) 
                               for k, v in self.joint_limits_deg.items()}
        
        self.running = True

    def go_ahead_callback(self, msg):

        self.go_ahead = msg.data

    def send_joint_angles(self):
        joint_msg = JointState()
        joint_msg.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
                         'panda_joint5', 'panda_joint6', 'panda_joint7']
        joint_msg.position = [random.uniform(self.joint_limits_rad[k][0], 
                            self.joint_limits_rad[k][1]) for k in joint_msg.name]
        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f'\nPublished random joint angles: {joint_msg.position}')

    async def request_camera_capture(self, service_name):
        client = self.capture_clients[service_name]
        
        # Wait for service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Camera service {service_name} not available, waiting...')
            
        try:
            request = Trigger.Request()
            response = await client.call_async(request)
            if response.success:
                self.get_logger().info(f'Image captured successfully for {service_name}')
            else:
                self.get_logger().warn(f'Image capture failed for {service_name}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for {service_name}: {str(e)}')

    async def run_schedule(self):
        if self.go_ahead:
            # Send joint angles
            self.send_joint_angles()
            
            # Wait 7 seconds
            self.get_logger().info('Waiting 7 seconds for robot to move...')
            for i in range(7, 0, -1):
                self.get_logger().info(f'{i} seconds remaining...')
                time.sleep(1)
                
            # Request captures from all cameras
            for camera_service in self.camera_services:
                await self.request_camera_capture(camera_service)


    async def main_loop(self):
        while self.running:
            await self.run_schedule()

def main(args=None):
    rclpy.init(args=args)
    node = MotorBabblerPublisher()
    node.get_logger().info('Motor babbler node started')
    
    # Create an executor and add our node
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # Start the main_loop
    executor.create_task(node.main_loop())
    
    # Spin forever until interrupted
    executor.spin()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

