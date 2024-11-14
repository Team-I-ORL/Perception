# Import Libraries
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from perception_interfaces.srv import FindX
from orbiter_bt.srv import MoveHead
from ros2_aruco_interfaces.srv import ArucoPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ros2_aruco.wait_for_msg import wait_for_message
from sensor_msgs.msg import Image
import threading
from enum import Enum
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class Status(Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5 
    PREEMPTING = 6 
    RECALLING = 7
    RECALLED = 8
    LOST = 9

class MoveHeadService(Node):
    def __init__(self):
        super().__init__('move_head_service')
        # Create server
        self.head_status_callback_group = MutuallyExclusiveCallbackGroup()
        self.head_callback_group = MutuallyExclusiveCallbackGroup()
        # self.image_callback_group = MutuallyExclusiveCallbackGroup()

        self.service = self.create_service(MoveHead, 
                                           '/move_head', 
                                           self.move_head_server,
                                           callback_group = self.head_status_callback_group)
        
        # Subscriber for joint state messages
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Create client 
        # Create a service client for the Aruco pose server
        my_callback = MutuallyExclusiveCallbackGroup()
        self.aruco_pose_client = self.create_client(FindX, 'find_x', callback_group = my_callback)
        self.robot_status = None

        # Wait for the service to be available
        while not self.aruco_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Aruco pose server not available, waiting...')

        # Create subscriber
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'input_joint_angles',
            10
        )

        self.head_status = self.create_subscription(
            Int32,
            'head_status',
            self.head_status_callback,
            10,
            callback_group=self.head_callback_group
        )

        # Define the sweep range for head pan and head tilt
        self.head_pan_sweep = [-0.9, 0, 0.9]
        self.head_tilt_sweep = [0.9]

        self.lock = threading.Lock()
        self.done = False

        self.head_joints = ['head_pan_joint', 'head_tilt_joint']
        
        self.num_call = False

        self.Kp = 0.0025
        self.initial_js = [0, 0]


    def joint_state_callback(self, msg):
        # Get the initial joint states
        joint_positions = dict(zip(
            msg.name,
            msg.position
        ))
        try:
            self.initial_js = [joint_positions[joint_name] for joint_name in self.head_joints]
        except KeyError as e:
            self.get_logger().error(f'Joint name {e} not found in joint states.')


    def head_status_callback(self, msg):
        with self.lock:
            # self.get_logger().info(f"Head status: {msg.data}")
            self.robot_status = Status(msg.data)
           

    def wait_for_idle(self):
        # Wait for the robot to become idle using the condition variable
        while rclpy.ok():
            with self.lock:
                # print(f"Robot status: {self.robot_status}")
                if self.robot_status not in [Status.ACTIVE, Status.PREEMPTING]:
                    break
            time.sleep(1)  # Sleep to prevent busy waiting
            self.get_logger().info("Robot is now idle.")

    
    def move_head_server(self, request, response):
        # TODO : Visual Servoing
        print(f"Moving head to find - {request.what}")
        self.head_pose = None

        self.head_pose = Float64MultiArray()
        success = False  


        for head_pan in self.head_pan_sweep:
            for head_tilt in self.head_tilt_sweep:
                diff_x = np.inf
                diff_y = np.inf
                success = False

                while abs(diff_x) >= 50 or abs(diff_y) >= 50:
                    diff_js = [head_pan - self.initial_js[0], head_tilt - self.initial_js[1]]

                    # Move the head
                    print(f"Moving head to pan: {head_pan}, tilt: {head_tilt}, time - {np.max(diff_js)}")
                    self.head_pose.data = [float(head_pan), float(head_tilt), np.max(diff_js) + 3.0]
                    self.publisher.publish(self.head_pose)

                    time.sleep(1.0)

                    # Wait for the head to stop moving
                    self.wait_for_idle()

                    # Create a request for the Aruco pose server
                    fin_x_request = FindX.Request()

                    # If the object is an aruco marker, set the id
                    if "aruco" in request.what:
                        fin_x_request.id = int(request.what[-1])
                        fin_x_request.object = "aruco"

                    else:
                        fin_x_request.object = "box"
                        fin_x_request.id = -1

                    # See if we find the object
                    res = self.aruco_pose_client.call(fin_x_request)
                    
                    if res.x == -1 or res.y == -1:
                        print("Didn't get the pose, moving to next pose")
                        break

                    elif res.x >= 0 and res.y >= 0:
                        print("Got the pose", res.x, res.y)
                        diff_x = 320 - res.x
                        diff_y = 240 - res.y

                        head_pan = head_pan + self.Kp * diff_x
                        head_tilt = head_tilt - self.Kp * diff_y
                    
                    else:
                        print("What the fuck is happening")
                        success = False
                        break

                if abs(diff_x) <= 50 or abs(diff_y) <= 50:
                    success = True
                    break
                   
            if success:
                break

        if success:
            response.success = True
            self.num_call = not self.num_call
            self.head_pan_sweep = list(reversed(self.head_pan_sweep))
            return response

        response.success = False
        self.num_call = not self.num_call
        self.head_pan_sweep = list(reversed(self.head_pan_sweep))
        return response


def main(args=None):

    # rclpy.init(args=args)
    # node = MoveHeadService()
    # rclpy.spin(node)
    # rclpy.shutdown()
    rclpy.init()
    
    drop_pose_service = MoveHeadService()
    executor = MultiThreadedExecutor()
    executor.add_node(drop_pose_service)

    try:
        drop_pose_service.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        drop_pose_service.get_logger().info('Keyboard interrupt, shutting down.\n')

    drop_pose_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()