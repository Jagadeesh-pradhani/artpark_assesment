#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
import threading

class Turtlebot3Drive(Node):
    def __init__(self):
        super().__init__('turtlebot3_drive_node')
        self.robot_id = self.get_namespace().strip('/')

        """************************************************************
        ** Initialize variables
        ************************************************************"""
        self.scan_data_ = [0.0, 0.0, 0.0]
        self.robot_pose_ = 0.0
        self.prev_robot_pose_ = 0.0
        
        # Lock for thread safety
        self._lock = threading.Lock()

        # Constants
        self.LINEAR_VELOCITY = 0.3
        self.ANGULAR_VELOCITY = 1.5
        self.CENTER = 0
        self.LEFT = 1
        self.RIGHT = 2

        # States
        self.GET_TB3_DIRECTION = 0
        self.TB3_DRIVE_FORWARD = 1
        self.TB3_RIGHT_TURN = 2
        self.TB3_LEFT_TURN = 3
        
        # Current state
        self.turtlebot3_state_num = self.GET_TB3_DIRECTION

        """************************************************************
        ** Initialize ROS publishers and subscribers with callback groups
        ************************************************************"""
        qos = QoSProfile(depth=10)
        
        # Create callback groups
        self.timer_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize publishers
        self.cmd_vel_pub_ = self.create_publisher(
            Twist,
            'cmd_vel',
            qos)

        # Initialize subscribers with callback groups
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10),
            callback_group=self.subscription_callback_group)
            
        self.odom_sub_ = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos,
            callback_group=self.subscription_callback_group)

        """************************************************************
        ** Initialize ROS timers with callback group
        ************************************************************"""
        self.update_timer_ = self.create_timer(
            0.010,  # 10ms
            self.update_callback,
            callback_group=self.timer_callback_group)

        self.get_logger().info('TurtleBot3 simulation node has been initialized')

    def __del__(self):
        self.get_logger().info('TurtleBot3 simulation node has been terminated')

    """************************************************************
    ** Callback functions for ROS subscribers
    ************************************************************"""
    def odom_callback(self, msg):
        with self._lock:
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w])
            
            self.robot_pose_ = yaw

    def scan_callback(self, msg):
        with self._lock:
            scan_angle = [0, 30, 330]
            
            for num in range(3):
                if math.isinf(msg.ranges[scan_angle[num]]):
                    self.scan_data_[num] = msg.range_max
                else:
                    self.scan_data_[num] = msg.ranges[scan_angle[num]]

    def update_cmd_vel(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        
        self.cmd_vel_pub_.publish(cmd_vel)

    """************************************************************
    ** Update functions
    ************************************************************"""
    def update_callback(self):
        with self._lock:
            DEG2RAD = math.pi / 180.0
            
            escape_range = 30.0 * DEG2RAD
            check_forward_dist = 0.7
            check_side_dist = 0.6

            if self.turtlebot3_state_num == self.GET_TB3_DIRECTION:
                if self.scan_data_[self.CENTER] > check_forward_dist:
                    if self.scan_data_[self.LEFT] < check_side_dist:
                        self.prev_robot_pose_ = self.robot_pose_
                        self.turtlebot3_state_num = self.TB3_RIGHT_TURN
                    elif self.scan_data_[self.RIGHT] < check_side_dist:
                        self.prev_robot_pose_ = self.robot_pose_
                        self.turtlebot3_state_num = self.TB3_LEFT_TURN
                    else:
                        self.turtlebot3_state_num = self.TB3_DRIVE_FORWARD
                
                if self.scan_data_[self.CENTER] < check_forward_dist:
                    self.prev_robot_pose_ = self.robot_pose_
                    self.turtlebot3_state_num = self.TB3_RIGHT_TURN

            elif self.turtlebot3_state_num == self.TB3_DRIVE_FORWARD:
                self.update_cmd_vel(self.LINEAR_VELOCITY, 0.0)
                self.turtlebot3_state_num = self.GET_TB3_DIRECTION

            elif self.turtlebot3_state_num == self.TB3_RIGHT_TURN:
                if abs(self.prev_robot_pose_ - self.robot_pose_) >= escape_range:
                    self.turtlebot3_state_num = self.GET_TB3_DIRECTION
                else:
                    self.update_cmd_vel(0.0, -1 * self.ANGULAR_VELOCITY)

            elif self.turtlebot3_state_num == self.TB3_LEFT_TURN:
                if abs(self.prev_robot_pose_ - self.robot_pose_) >= escape_range:
                    self.turtlebot3_state_num = self.GET_TB3_DIRECTION
                else:
                    self.update_cmd_vel(0.0, self.ANGULAR_VELOCITY)

            else:
                self.turtlebot3_state_num = self.GET_TB3_DIRECTION

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    turtlebot3_drive = Turtlebot3Drive()
    
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(turtlebot3_drive)

    try:
        # Spin the executor instead of the node
        executor.spin()
    finally:
        # Cleanup
        executor.shutdown()
        turtlebot3_drive.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()