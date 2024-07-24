import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import random

class EndEffectorGraspPublisher(Node):
    def __init__(self):
        super().__init__('grasp_publisher')
        self.publisher_ = self.create_publisher(Pose, 'end_effector_pose', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)  # Timer set to x seconds

    def timer_callback(self):
        msg = Pose()
        # Generate a random position between -5 and 5 for all x, y, and z values
        msg.position.x = float(random.uniform(-0.5, 0.5))
        msg.position.y = float(random.uniform(-0.5, 0.5))
        msg.position.z = float(random.uniform(0.2, 0.5))

        # Generate a slightly random orientation for the end effector (Quaternions)
        msg.orientation.x = float(random.uniform(-0.1, 0.1))
        msg.orientation.y = float(random.uniform(-0.1, 0.1))
        msg.orientation.z = float(random.uniform(-0.1, 0.1))
        msg.orientation.w = float(random.uniform(-0.1, 0.1))

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published end effector pose: {msg.position.x}, {msg.position.y}, {msg.position.z}')

def main(args=None):
    rclpy.init(args=args)
    ee_pub = EndEffectorGraspPublisher()
    ee_pub.get_logger().info('Grasp publisher node started')
    rclpy.spin(ee_pub)
    ee_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()