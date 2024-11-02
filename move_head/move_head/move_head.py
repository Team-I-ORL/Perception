# Import Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from perception_interfaces.srv import FindX
from orbiter_bt.srv import MoveHead

class MoveHeadService(Node):
    def __init__(self):
        super().__init__('move_head_service')
        # Create server
        self.service = self.create_service(MoveHead, '/move_head', self.move_head_server)

        # Create client 
        # self.client = self.create_client(FindX, 'find_x')
        # Wait for the service to be available
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for service to be available...')

        # Create subscriber
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'input_joint_angles',
            10
        )

        # Define the sweep range for head pan and head tilt
        self.head_pan_sweep = [-0.9, 0, 0.9]
        self.head_tilt_sweep = [0.2, 0.9]

    def move_head_server(self, request, response):
        # TODO : Visual Servoing
        print(request.what)
        self.head_pose = None

        self.head_pose = Float64MultiArray()

        for head_pan in self.head_pan_sweep:
            for head_tilt in self.head_tilt_sweep:
                # Move the head
                self.head_pose.data = [head_pan, head_tilt]
                self.publisher.publish(self.head_pose)

                # See if we find the object

                # Stop

        response.success = True
        return response
        
        print(head_pose)
        self.publisher.publish(head_pose)
        print(head_pose)
        response.success = True
        return response


    # def send_request(self):
    #     request = FindX.request()

    #     self.pose_of_x = self.client.call_async(request)

        # TODO: Do something with the pose
        



def main(args=None):

    rclpy.init(args=args)
    node = MoveHeadService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()