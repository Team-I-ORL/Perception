#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from orbiter_bt.srv import MoveHead, GetDropPose  # Replace with the actual package and service name

class ServiceTester(Node):
    def __init__(self):
        super().__init__('service_tester')
        # Create a client for the service
        self.client = self.create_client(MoveHead, '/move_head')

        self.aruco_client = self.create_client(GetDropPose, '/get_drop_pose_from_head')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        
        while not self.aruco_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        
        # Create a request
        self.request = MoveHead.Request()
        self.request.what = 'box' 
        self.request.type = True # Replace with your test string
        # self.request.success = True  # Replace with your test boolean

    def send_request(self):
        # Send the request
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info(f'Result: {self.future.result().success}')
        else:
            self.get_logger().error('Service call failed')

    def new_request(self):
        self.new_request  = GetDropPose.Request()
        self.new_request.aruco_id = 1
        self.future = self.aruco_client.call_async(self.new_request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info(f'Result: {self.future.result().pose}')
        else:
            self.get_logger().error('Service call failed')

    

def main(args=None):
    rclpy.init(args=args)
    service_tester = ServiceTester()
    service_tester.send_request()

    time.sleep(1)

    # service_tester.request.what = 'aruco7'
    # service_tester.request.type = False
    # service_tester.send_request()

    # time.sleep(1)

    # service_tester.new_request()

    # service_tester = ServiceTester()
    # service_tester.aruco_client = service_tester.create_client(GetDropPose, '/get_drop_pose_from_head')
    # while not service_tester.aruco_client.wait_for_service(timeout_sec=1.0):
    #     service_tester.get_logger().info('Waiting for service to become available...')
    # request = GetDropPose.Request()
    # request.aruco_id = 1
    # service_tester.aruco_client.call_async(request)
    # rclpy.spin_until_future_complete(service_tester, service_tester.future)
    # if service_tester.future.result() is not None:
    #     service_tester.get_logger().info(f'Result: {service_tester.future.result().pose}')
    # else:
    #     service_tester.get_logger().error('Service call failed')

    service_tester.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
