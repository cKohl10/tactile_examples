import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class TouchSensorPublisher(Node):
    def __init__(self, num_sensors):
        super().__init__('touch_sensor_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'touch_sensor_val', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer set to 1 second

        self.num_sensors = int(num_sensors)
        self.rand_chance = 0.5/float(self.num_sensors)

    def timer_callback(self):
        msg = Float32MultiArray()
        # Generate an array of 5 values, each is either a random float or 0, with a 5% chance for the random float
        msg.data = [random.uniform(0.0, 100.0) if random.random() < self.rand_chance else 0.0 for _ in range(self.num_sensors)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published touch sensor values: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    # Set the index depending on if a command line argument is given
    if len(sys.argv) == 2:
        num_sensors = str(sys.argv[1])
    else:
        num_sensors = "150"

    touch_sensor_publisher = TouchSensorPublisher(num_sensors)
    touch_sensor_publisher.get_logger().info('Touch sensor publisher node started')
    rclpy.spin(touch_sensor_publisher)
    touch_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()