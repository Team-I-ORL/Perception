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

        # Define initial pose for each type of object and TODO visual servoing post that
        # Box Pose
        self.box_pose = Float64MultiArray()
        self.box_pose.data = [-0.9, 0.9]

        # Aruco 1 Pose
        self.ar1_pose = Float64MultiArray()
        self.ar1_pose.data = [0.0, 0.2]

        # Aruco 2 Pose
        self.ar2_pose = Float64MultiArray()
        self.ar2_pose.data = [0.0, 0.2]


    def move_head_server(self, request, response):
        # TODO : Visual Servoing
        print(request.what)
        head_pose = None

        if request.what == 'box':
            head_pose = self.box_pose

        elif request.what == 'ar1':
            head_pose = self.ar1_pose

        elif request.what == 'ar2':
            head_pose = self.ar2_pose

        else:
            response.success = False
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