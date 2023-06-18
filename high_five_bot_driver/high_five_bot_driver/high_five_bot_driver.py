#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import math
from std_srvs.srv import Trigger
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import math
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.

    Arguments:
    roll -- rotation around the x-axis (in radians)
    pitch -- rotation around the y-axis (in radians)
    yaw -- rotation around the z-axis (in radians)

    Returns:
    Quaternion as a numpy array: [w, x, y, z]
    """

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

class GoToHand(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Trigger, 'go_to_hand', self.go_to_hand_callback)
        self.go_to_hand_flag = False
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.i = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_position = [0.0, 0.0, 0.0]
        
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.transform = TransformStamped()
        self.x = 0.0
        self.y = 0.0
        self.time = 0.0


    def timer_callback(self):
        # self.broadcast()
        self.go_to_hand_flag = False
        from_frame_rel = 'hand'
        to_frame_rel = 'world'
        try:
            self.transform = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel,rclpy.time.Time())
            self.goal_position[0] = -self.transform.transform.translation.x
            self.goal_position[1] = -self.transform.transform.translation.y
            self.goal_position[2] = self.transform.transform.translation.z
            self.orientation[0] = self.transform.transform.rotation.x
            self.orientation[1] = self.transform.transform.rotation.y
            self.orientation[2] = self.transform.transform.rotation.z
            self.orientation[3] = self.transform.transform.rotation.w

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

    def go_to_hand_callback(self, request, response):
        self.go_to_hand_flag = True
        return response

    def can_go_to_hand(self):
        return self.go_to_hand_flag
    
    def get_goal_position(self):
        return self.goal_position
    
    def get_goal_orientation(self):
        return self.orientation
    
    def broadcast(self):
        t = TransformStamped()
        self.time +=  0.01
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'hand'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = abs(0.25*math.cos(self.time))
        t.transform.translation.y = 0.25*math.sin(self.time) - 0.25
        t.transform.translation.z = 0.35

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = euler_to_quaternion(self.time, 0 , 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # self.get_logger().info(t)
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")
    hand_node = GoToHand()

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.1, -0.3, 0.3])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("cartesian", True)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur.joint_names(''),
        base_link_name=ur.base_link_name(''),
        end_effector_name=ur.end_effector_name(''),
        group_name=ur.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    quat_xyzw = [0.0, 0.0, 0.0, 1.0]
    cartesian = True
    init_position = [0.4, 0.0, 0.2]

    # Move to pose
    while True:
        rclpy.spin_once(hand_node)
        if hand_node.can_go_to_hand():
            hand_node.get_logger().info(
                f"Moving to {{position: {list(hand_node.get_goal_position())}, quat_xyzw: {list(quat_xyzw)}}}"
            )
            moveit2.move_to_pose(position=hand_node.get_goal_position(),
                                 quat_xyzw=quat_xyzw, cartesian=cartesian)
            moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
