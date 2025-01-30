#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class FauxCameraService(Node):  # Renamed to better reflect its role
    def __init__(self):
        super().__init__('faux_camera_service')  # Renamed to better reflect its role
        
        # Create the service (not client)
        self.srv = self.create_service(Trigger, '/camera/capture', self.capture_callback)
        
        self.get_logger().info('Faux camera service started')

    def capture_callback(self, request, response):
        # Simulate some camera action
        self.get_logger().info('Received capture request')
        
        # Set success response
        response.success = True
        response.message = "Faux image captured successfully"
        self.get_logger().info('Responding with success')
        return response

def main(args=None):
    rclpy.init(args=args)
    camera_service = FauxCameraService()

    try:
        rclpy.spin(camera_service)
    except KeyboardInterrupt:
        pass
    finally:
        camera_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
