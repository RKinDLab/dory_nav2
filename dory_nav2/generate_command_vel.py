#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayLayout, Float64MultiArray
from geometry_msgs.msg import Twist
from control_msgs.msg import DynamicJointState

import numpy as np

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.02, self.publish_message)

        # TODO: implement \clock for this
        self.time = 0

    def publish_message(self):
        cmd_vel = self.gen_cmd_vel()
        self.publisher.publish(cmd_vel)

    def gen_cmd_vel(self):

        cmd_vel_msg = Twist()

        if self.time <= 10000:

            cmd_vel_msg.linear.x = 5.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.0

        elif self.time > 10000 and self.time <=20000:

            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 5.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.0

        elif self.time > 20000:

            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.0

        self.time += 1

        return cmd_vel_msg


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()