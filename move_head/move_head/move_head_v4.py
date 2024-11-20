# Import Libraries
import cv2
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
from cv_bridge import CvBridge
from ultralytics import YOLO, SAM 


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

        self.rgb_sub = self.create_subscription(
            Image,
            'head_camera/rgb/image_raw',
            self.image_callback,
            10,
            callback_group=self.did_move_callback_group
        )

        # Define the sweep range for head pan and head tilt
        # self.head_pan_sweep = [-0.9, 0]
        # self.head_tilt_sweep = [0.9, 0.4, 0]   
        self.head_joints = ['head_pan_joint', 'head_tilt_joint']

        # Create a 2D sine wave for head pan and head tilt
        # self.head_pan_sweep = np.linspace(-1.0, 1.0, 100)
        # self.head_tilt_sweep = np.sin(self.head_pan_sweep).tolist()
        self.head_pan_sweep = list(np.linspace(-1.0, 1.0, 20))
        self.head_pan_sweep += [1.0, 1.0]
        self.head_pan_sweep += reversed(self.head_pan_sweep)
        self.head_pan_sweep += [-1.0, -1.0]
        self.head_tilt_sweep = list(np.linspace(0, 1, 20))
        self.head_tilt_sweep += [0.7, 0.3]
        self.head_tilt_sweep += self.head_tilt_sweep
        self.head_tilt_sweep += [0.7, 0.3] 
         #list(np.sin(self.head_pan_sweep), 0, 1)
        #self.head_sweep = np.meshgrid(self.head_pan_sweep, self.head_tilt_sweep)
        # self.head_sweep = [(pan, tilt) for pan in self.head_pan_sweep for tilt in self.head_tilt_sweep]

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.counter = 0

        self.js = [0.0, 0.0]

        self.robot_status = Status.ACTIVE

        self.image_batch = []

        self.yolo_model = YOLO('/workspaces/isaac_ros-dev/src/Perception/grounded_sam/seg_mask/checkpoints/yolo_best.pt')
        self.sam_model = SAM("sam2.1_t.pt")

    def image_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        

    def joint_state_callback(self, msg):
        # Get the initial joint states
        joint_positions = dict(zip(msg.name,msg.position))
        try:
            self.js = [joint_positions[joint_name] for joint_name in self.head_joints]
        except KeyError as e:
            self.get_logger().error(f'Joint name {e} not found in joint states.')


    def head_status_callback(self, msg):
        with self.lock:
            self.get_logger().debug(f"Head status: {msg.data}")
            self.robot_status = Status(msg.data)
           

    def wait_for_idle(self):
        # Wait for the robot to become idle using the condition variable
        while rclpy.ok():
            time.sleep(0.1)  # Sleep to prevent busy waiting
            with self.lock:
                if self.robot_status not in [Status.ACTIVE, Status.PREEMPTING]:
                    break


    def move_head(self, head_pan, head_tilt, time_exec):
        # Move the head
        self.get_logger().info(f"Moving head to pan: {head_pan}, tilt: {head_tilt}, time - {time_exec}")
        self.head_pose = Float64MultiArray()
        self.head_pose.data = [float(head_pan), float(head_tilt), time_exec]
        self.publisher.publish(self.head_pose)

        time.sleep(0.1)

        # Wait for the head to stop moving
        self.wait_for_idle()


    def move_head_server(self, request, response):
        # TODO : Visual Servoing
        self.get_logger().info(f"Received request: What - {request.what}, Type - {request.type}")

        # Get the request type
        self.request_type = request.type

        for i, head_pan_loop in enumerate(self.head_pan_sweep):
            # with self.lock:
                self.counter += 1

                diff_js = [abs(self.head_pan_sweep[i] - self.js[0]), abs(self.head_tilt_sweep[i] - self.js[1])]
                self.move_head(head_pan_loop, self.head_tilt_sweep[i], np.max(diff_js) + 0.1)
                # self.update_collision.publish(Bool(data=True))
                if self.counter%2 == 0 or self.head_tilt_sweep[i] == 0.7 or self.head_tilt_sweep[i] == 0.3:
                    print("Image")
                    time.sleep(0.5)
                    self.image_batch.append(cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR))
                    cv2.imwrite(f"Image_{self.counter}.png", cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR))

        s = time.time()
        yolo_results = self.yolo_model.predict(self.image_batch)
        # boxes = yolo_results[0].boxes.xyxy.cpu().numpy()
        print(time.time() - s)
        # Filter boxes that belong to the object of interest
        filtered_boxes = []
        # for r in yolo_results:
        #     print(r)
        response.success = True
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
