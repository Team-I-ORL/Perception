#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from orbiter_bt.srv import MoveHead  # Replace with the actual package and service name

class ServiceTester(Node):
    def __init__(self):
        super().__init__('service_tester')
        # Create a client for the service
        self.client = self.create_client(MoveHead, '/move_head')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        
        # Create a request
        self.request = MoveHead.Request()
        self.request.what = 'box'  # Replace with your test string
        # self.request.success = True  # Replace with your test boolean

    def send_request(self):
        # Send the request
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info(f'Result: {self.future.result().success}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    service_tester = ServiceTester()
    service_tester.send_request()

    time.sleep(5)

    service_tester.request.what = 'ar1'
    service_tester.send_request()
    service_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
