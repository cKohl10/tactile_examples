import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
import math
"""
This script publishes random joint angles to the Franka robot.
"""

class RandomJointAnglesPublisher(Node):
    def __init__(self):
        super().__init__('random_joint_angles_publisher')
        self.publisher_ = self.create_publisher(JointState, '/franka/command/joint_states', 10)
        self.timer = self.create_timer(7.0, self.publish_random_joint_angles)
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

    def publish_random_joint_angles(self):
        msg = JointState()
        msg.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        msg.position = [random.uniform(self.joint_limits_rad[k][0], self.joint_limits_rad[k][1]) for k in msg.name]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published random joint angles: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = RandomJointAnglesPublisher()
    node.get_logger().info('Random joint angles node started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

