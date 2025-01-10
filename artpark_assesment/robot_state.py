#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        # Create subscriber for odometry data
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Create publisher for robot pose
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10)
        
        self.get_logger().info('Robot State Publisher Node has been started')

    def odom_callback(self, msg):
        # Create PoseStamped message
        pose_msg = PoseStamped()
        
        # Copy header from odometry message
        pose_msg.header = msg.header
        
        # Copy position data
        pose_msg.pose.position = msg.pose.pose.position
        pose_msg.pose.orientation = msg.pose.pose.orientation
        
        # Extract Euler angles from quaternion
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        # Log position information (optional)
        self.get_logger().info(
            f'Robot Position - X: {pose_msg.pose.position.x:.2f}, '
            f'Y: {pose_msg.pose.position.y:.2f}, Theta: {yaw:.2f}'
        )
        
        # Publish the pose
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_state_publisher = RobotStatePublisher()
    
    try:
        rclpy.spin(robot_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()