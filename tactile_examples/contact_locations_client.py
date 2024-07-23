# Author: Carson Kohlbrenner
# Description: This script is an example of how to use the IndexToPos service to get the contact locations of the tactile sensors on the robot.

import sys
import time

#Custom service
from tactile_msgs.srv import IndexToPos
from geometry_msgs.msg import Vector3

# ROS 2 packages
import rclpy
from rclpy.node import Node

class ContactLocationClient(Node):
    
        def __init__(self):
            super().__init__('contact_location_client')
            self.client = self.create_client(IndexToPos, 'index_to_pos')
            self.get_logger().info('Contact Location Client has been started...')
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = IndexToPos.Request()
    
        def send_request(self, index):
            self.get_logger().info('Sending request...')
            self.req.index = index
            self.future = self.client.call_async(self.req)
            self.future.add_done_callback(self.future_callback)
            return self.future
    
        def future_callback(self, future):
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                self.get_logger().info('Contact location: {%0.2f, %0.2f, %0.2f}' % (response.position.x, response.position.y, response.position.z))

def main():
    rclpy.init()

    contact_location_client = ContactLocationClient()
    while rclpy.ok():
        future = contact_location_client.send_request(0)
        rclpy.spin_until_future_complete(contact_location_client, future)
        response = future.result()

        time.sleep(1) # Sleep for 1 second

    contact_location_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()