## Publishes a command line argument to the /contact_dist topic

import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Float32

class SetSpawnDistPub(Node):
    def __init__(self, dist):
        super().__init__('set_spawn_dist_pub')
        self.publisher_ = self.create_publisher(Float32, 'contact_dist', 10)
        self.data = 0.0

        self.dist = float(dist)
        
        # Create a timer that triggers the callback every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = self.dist
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) == 2:
        dist = str(sys.argv[1])
    else:
        dist = 0.0

    dist_pub = SetSpawnDistPub(dist)  # Example distance value
    rclpy.spin(dist_pub)
    dist_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()