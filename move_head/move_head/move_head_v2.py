# Import Libraries
import copy
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Bool
from perception_interfaces.srv import FindX
from orbiter_bt.srv import MoveHead, GetDropPose
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
        self.get_drop_pose_from_head_callback_group = MutuallyExclusiveCallbackGroup()
        self.head_callback_group = MutuallyExclusiveCallbackGroup()
        self.did_move_callback_group = MutuallyExclusiveCallbackGroup()

        # Define the services
        self.service = self.create_service(MoveHead, 
                                           '/move_head', 
                                           self.move_head_server,
                                           callback_group = self.head_status_callback_group)

        self.get_drop_pose_from_head = self.create_service(GetDropPose, 
                                                           '/get_drop_pose_from_head', 
                                                           self.get_drop_pose_from_head_server,
                                                           callback_group = self.get_drop_pose_from_head_callback_group)
        
        # Create service client to find an object and 3D pose of aruco markers 
        aruco_callback = MutuallyExclusiveCallbackGroup()
        self.aruco_pose_client = self.create_client(FindX, 'find_x', callback_group = aruco_callback)

        get_drop_pose_callback = MutuallyExclusiveCallbackGroup()
        self.get_drop_pose_client = self.create_client(GetDropPose, 'get_drop_pose', callback_group = get_drop_pose_callback)
        
        # Wait for the service to be available
        while not self.aruco_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Aruco pose server not available, waiting...')

        # Create publisher to move the head
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'input_joint_angles',
            10
        )

        self.update_collision = self.create_publisher(
            Bool,
            'update_collision_env',
            10
        )

        # Create subscribers to get the joint states and head status
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        self.head_status = self.create_subscription(
            Int32,
            'head_status',
            self.head_status_callback,
            10,
            callback_group=self.head_callback_group
        )


        self.did_move_sub = self.create_subscription(
            Bool,
            'moved',
            self.did_move_callback,
            10,
            callback_group=self.did_move_callback_group
        )

        # Multithreading executors
        self.lock = threading.Lock()
        self.done = False

        # Define the sweep range for head pan and head tilt
        self.head_pan_sweep = [-0.9, 0, 0.9]
        self.head_tilt_sweep = [0.9, 0]   
        
        # Create a dict for storing goal poses 
        self.goal_poses = {'aruco1': [],
                           'aruco2': [],
                           'aruco3': [],
                           'aruco7': [],
                           'box': [],
                            'obj1': [[0.0, 0.0]],
                            'obj2': [[0.0, 0.0]],
                            'obj3': [[0.0, 0.0]]}
        
        self.objects_list = ['obj1', 'obj2', 'obj3']

        self.goal_3d_poses = {'aruco1': [],
                              'aruco2': [],
                              'aruco3': [],
                              'aruco7': []}

        # Execution params
        self.head_joints = ['head_pan_joint', 'head_tilt_joint']
        self.Kp = 0.0025
        self.initial_js = [0, 0]
        self.robot_status = Status.PENDING
        self.did_move = True

    def did_move_callback(self, msg):
        with self.lock:
            self.did_move = msg.data

            # If the robot moved, reset the goal poses
            if self.did_move:
                self.get_logger().info("Robot moved")
                self.goal_poses = {'aruco1': [],
                    'aruco2': [],
                    'aruco3': [],
                    'aruco7': [],
                    'box': [],
                    'obj1': [[0.0, 0.0]],
                    'obj2': [[0.0, 0.0]],
                    'obj3': [[0.0, 0.0]]}

                self.goal_3d_poses = {'aruco1': [],
                              'aruco2': [],
                              'aruco3': [],
                              'aruco7': []}

                self.update_collision.publish(Bool(data=False))


    def joint_state_callback(self, msg):
        # Get the initial joint states
        joint_positions = dict(zip(msg.name,msg.position))
        try:
            self.initial_js = [joint_positions[joint_name] for joint_name in self.head_joints]
        except KeyError as e:
            self.get_logger().error(f'Joint name {e} not found in joint states.')


    def head_status_callback(self, msg):
        with self.lock:
            self.get_logger().debug(f"Head status: {msg.data}")
            self.robot_status = Status(msg.data)
           

    def wait_for_idle(self):
        # Wait for the robot to become idle using the condition variable
        while rclpy.ok():
            with self.lock:
                if self.robot_status not in [Status.ACTIVE, Status.PREEMPTING]:
                    break
            time.sleep(0.5)  # Sleep to prevent busy waiting


    def move_head(self, head_pan, head_tilt, time_exec):
        # Move the head
        self.get_logger().info(f"Moving head to pan: {head_pan}, tilt: {head_tilt}, time - {time_exec}")
        self.head_pose = Float64MultiArray()
        self.head_pose.data = [float(head_pan), float(head_tilt), time_exec]
        self.publisher.publish(self.head_pose)

        time.sleep(1.0)

        # Wait for the head to stop moving
        self.wait_for_idle()


    def go_to_location(self, what):
        # Get the goal pose from the dict
        goal_pose = self.goal_poses[what]

        if what not in self.objects_list:
            self.move_head(goal_pose[0], goal_pose[1], 2.0)

        else:
            try:
                self.move_head(goal_pose[0], goal_pose[1], 2.0)
            except Exception as e:
                self.get_logger().error(f"Error moving to location: {e}")
                raise Exception(f"Error moving to location: {e}")


    def servo(self, current_pose, current_head_pose, find_x_request):
        # Get the difference in the pose
        diff_x = 320 - current_pose[0]
        diff_y = 240 - current_pose[1]

        while abs(diff_x) >= 50 or abs(diff_y) >= 50:
            # Get the new pose
            current_head_pose[0] = current_head_pose[0] + self.Kp * diff_x
            current_head_pose[1] = current_head_pose[1] - self.Kp * diff_y

            # Move the head
            self.move_head(current_head_pose[0], current_head_pose[1], 1.0)

            # Get the pose
            res = self.aruco_pose_client.call(find_x_request)

            if res.x == -1 or res.y == -1:
                return current_head_pose

            diff_x = 320 - res.x
            diff_y = 240 - res.y

        return current_head_pose


    def search(self, head_pan_sweep, head_tilt_sweep, what = None, stop_after = False):
        
        # Update goal post list based on what
        if what is not None:
            goal_poses_list = {what: []}
        else:
            goal_poses_list = copy.deepcopy(self.goal_poses)

        # Sweep
        for head_pan_loop in head_pan_sweep:
            for head_tilt_loop in head_tilt_sweep:
                # Find the object
                for key, value in goal_poses_list.items():
                    
                    # Get the difference in the joint states
                    diff_js = [head_pan_loop - self.initial_js[0], head_tilt_loop - self.initial_js[1]]
                    self.move_head(head_pan_loop, head_tilt_loop, 1.0) #np.max(diff_js) + 2.0)

                    update_col_data = Bool()
                    update_col_data.data = True
                    self.update_collision.publish(update_col_data)
                    
                    if key not in self.objects_list and len(value) == 0:    
                        # Create a request for the FindX service
                        find_x_request = FindX.Request()

                        if "aruco" in key:
                            find_x_request.object = "aruco"
                            find_x_request.id = int(key[-1])
                        else:
                            find_x_request.object = key
                            find_x_request.id = -1

                        # Call the service
                        res = self.aruco_pose_client.call(find_x_request)

                        # If the object is found, store the pose
                        if res.x != -1 and res.y != -1:
                            [goal_pan, goal_tilt] = self.servo([res.x, res.y], [copy.deepcopy(head_pan_loop), copy.deepcopy(head_tilt_loop)] ,find_x_request)
                            self.goal_poses[key] =  [goal_pan, goal_tilt]

                            if "aruco" in key:
                                self.get_logger().info(f"Finding 3D pose of Aruco {key[-1]}")
                                aruco_pose_request = GetDropPose.Request()
                                aruco_pose_request.aruco_id = int(key[-1])

                                res_3d = None
                                while res_3d is None:
                                    # Call the service
                                    res_3d = self.get_drop_pose_client.call(aruco_pose_request)

                                    if res_3d.pose.position.x != 0 and res_3d.pose.position.y != 0:
                                        self.goal_3d_poses[key].append([res_3d.pose])
                                    else:
                                        self.get_logger().info(f"Could not find 3D pose of Aruco {key[-1]}")

                            self.get_logger().info(f"Found {key} at {goal_pan}, {goal_tilt}")

                        else:
                            self.get_logger().info(f"Could not find {key}")

        
    def move_head_server(self, request, response):
        # TODO : Visual Servoing
        self.get_logger().info(f"Received request: What - {request.what}, Type - {request.type}")

        # Get the request type
        self.request_type = request.type

        if self.request_type:
            self.get_logger().info(f"Request type: {self.request_type}")

            try:
                if self.did_move:
                    # TODO: If the objects are already found, do we search again?
                    self.search(self.head_pan_sweep, self.head_tilt_sweep)
                    self.get_logger().info("Search complete")
                    self.did_move = False
                    
                self.get_logger().info(str(self.goal_poses))
                response.success = True
                return response

            
            except Exception as e:
                self.get_logger().error(f"Error searching for object: {e}")
                response.success = False
                return response

        else:
            self.get_logger().info(f"Request type: {self.request_type}")
            what = request.what

            try:
                self.go_to_location(what)
                self.get_logger().info(f"Moved to {what}")
                response.success = True
                return response
            
            except Exception as e:
                # Search for the object
                self.search([0.0, -0.9, 0.9], [0.0, 0.9], what = what, stop_after = True)
                self.get_logger().info("Search complete")

                # See if the object was found
                if self.goal_poses[what] == []:
                    self.get_logger().error(f"Error moving to location: {e}")
                    response.success = False
                    return response
                else:
                    self.go_to_location(what)
                    self.get_logger().info(f"Moved to {what}")
                    response.success = True
                    return response

                # self.get_logger().error(f"Error moving to location: {e}")
                # response.success = False
                # return response


    def get_drop_pose_from_head_server(self, request, response):
        # Get the drop pose of the object
        self.get_logger().info(f"Received request: What - {request.aruco_id}")

        try:
            print(self.goal_3d_poses[f"aruco{int(request.aruco_id)}"][0])
            response.pose = self.goal_3d_poses[f"aruco{request.aruco_id}"][0][0]
            return response
        
        except Exception as e:
            self.get_logger().error(f"Error getting drop pose: {e}")
            # response.pose = None
            return response
        
def main(args=None):

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
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
