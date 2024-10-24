# Import Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
# from move_head.srv import FindX
# from orbiter_bt.srv import MoveHead

class MoveHeadService(Node):
    def __init__(self):
        super().__init__('move_head_service')
        # Create server
        # self.service = self.create_service(MoveHead, 'move_head', self.move_head_server)

        # Create client 
        # self.client = self.create_client(FindX, 'find_x')
        # Wait for the service to be available
        # while not self.clienzt.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for service to be available...')

        # Define initial pose for each type of object and TODO visual servoing post that
        # Box Pose
        self.box_pose = PointStamped()
        self.box_pose.point.x = 1.0
        self.box_pose.point.y = 1.0
        self.box_pose.point.z = 0.0

        # Aruco 1 Pose
        self.ar1_pose = PointStamped()
        self.ar1_pose.point.x = 1.0
        self.ar1_pose.point.y = 0.0
        self.ar1_pose.point.z = 1.0

        # Aruco 2 Pose
        self.ar2_pose = PointStamped()
        self.ar2_pose.point.x = 1.0
        self.ar2_pose.point.y = -0.5
        self.ar2_pose.point.z = 1.0

    def move_head_server(self, request, response):
        # TODO : Visual Servoing
        if request.what == 'box':
            response.pose = self.box_pose

        elif request.what == 'ar1':
            response.pose = self.ar1_pose

        elif request.what == 'ar2':
            response.pose = self.ar2_pose
    
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