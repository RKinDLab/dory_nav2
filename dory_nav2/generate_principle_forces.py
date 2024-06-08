#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayLayout, Float64MultiArray
from geometry_msgs.msg import Twist
from control_msgs.msg import DynamicJointState

import numpy as np
from numpy import cos, sin
from scipy import linalg

# TODO: Bag cmd_vel

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pub_sub_node')
        
        # topics to subscribe
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.sub_vel = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.vel_callback,
            10,
        )

        # topics to publish
        self.effort_pub_rate = 0.02
        self.effort_publisher = self.create_publisher(Float64MultiArray, 'forward_effort_controller/commands', 10)
        self.effort_timer = self.create_timer(self.effort_pub_rate, self.effort_publish_message)

        self.state_thrust_pub_rate = 0.25
        self.state_thrust_publisher = self.create_publisher(Float64MultiArray, '/state_thrust_vector', 10)
        self.state_thrust_timer = self.create_timer(self.state_thrust_pub_rate, self.state_thrust_publish_message)

        # define cmd_vel_vector and vel_vector
        self.cmd_vel_vector = None
        self.vel_vector = None

        self.effort_time_steps = 0


    def effort_publish_message(self):
        effort = self.gen_control_effort()
        self.effort_publisher.publish(effort)
        # self.get_logger().info(f"Published the thrust. Woot! {effort}")
        self.effort_time_steps += 1

    def state_thrust_publish_message(self):

        # TODO: add a try statement for just in case state has not been received yet

        # resultant thrusts and torques on principle axis
        thrust = self.gen_control_force()

        # combine states and thrusts for publishing
        data = [self.state_vector[0],
                self.state_vector[1],
                self.state_vector[2],
                self.state_vector[3],
                self.state_vector[4],
                self.state_vector[5],
                self.state_vector[6],
                self.state_vector[7],
                self.state_vector[8],
                self.state_vector[9],
                self.state_vector[10],
                self.state_vector[11],
                thrust[0],
                thrust[1],
                thrust[2],
                thrust[3],
                thrust[4],
                thrust[5],
                ]

        dim = []

        layout = MultiArrayLayout(dim=dim, data_offset=0)

        state_thrust_msg = Float64MultiArray(layout=layout, data=data)

        self.state_thrust_publisher.publish(state_thrust_msg)
        # self.get_logger().info(f"Published the state thrust. Woot! {state_thrust_msg}")

    def cmd_vel_callback(self, msg):
        # self.get_logger().info(f"Received {msg.linear.x}")
        x_dot_cmd = msg.linear.x
        # self.get_logger().info(f"Received {msg.linear.y}")
        y_dot_cmd = msg.linear.y
        # self.get_logger().info(f"Received {msg.linear.z}")
        z_dot_cmd = msg.linear.z
        # self.get_logger().info(f"Received {msg.angular.x}")
        x_ang_dot_cmd = msg.angular.x
        # self.get_logger().info(f"Received {msg.angular.y}")
        y_ang_dot_cmd = msg.angular.y
        # self.get_logger().info(f"Received {msg.angular.z}")
        z_ang_dot_cmd = msg.angular.z

        self.cmd_vel_vector = np.array([x_dot_cmd, y_dot_cmd, z_dot_cmd, x_ang_dot_cmd, y_ang_dot_cmd, z_ang_dot_cmd])

    def vel_callback(self, msg):
        
        # position.x
        # self.get_logger().info(f"Received {msg.interface_values[8].interface_names[4]}: {msg.interface_values[8].values[4]}")
        x = msg.interface_values[8].values[0]
        
        # position.y
        y = msg.interface_values[8].values[9]
        
        # position.z
        z = msg.interface_values[8].values[1]
        
        # velocity.x
        x_dot = msg.interface_values[8].values[3]
        
        # velocity.y
        y_dot = msg.interface_values[8].values[6]
        
        # velocity.z
        z_dot = msg.interface_values[8].values[5]
        
        # rot.r (roll)
        roll = msg.interface_values[8].values[4]
        
        # rot.p (pitch)
        pitch = msg.interface_values[8].values[10]
        
        # rot.y (yaw)
        yaw = msg.interface_values[8].values[2]
        
        # w.r (roll rate)
        roll_dot = msg.interface_values[8].values[7]

        # w.p (pitch rate)
        pitch_dot = msg.interface_values[8].values[8]

        # w.y (yaw rate)
        yaw_dot = msg.interface_values[8].values[11]

        self.vel_vector = np.array([x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot])
        self.state_vector = np.array([x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot])

    def rotation_to_body_fixed(self,angles,vector):

        # angles
        roll = angles[0]
        pitch = angles[1]
        yaw = angles[2]

        # yaw rotation
        R_z = np.array([
                        [cos(yaw), -sin(yaw), 0],
                        [sin(yaw), cos(yaw), 0],
                        [0, 0, 1]
                        ])

        # pitch rotation
        R_y = np.array([
                        [cos(pitch), 0, sin(pitch)],
                        [0, 1, 0],
                        [-sin(pitch), 0, cos(pitch)]
                        ])

        # roll rotation
        R_x = np.array([
                        [1, 0, 0],
                        [0, cos(roll), -sin(roll)],
                        [0, sin(roll), cos(roll)]
                        ])

        # total rotation matrix
        # X = R_total@x # return relative to space-fixed coordinates
        R_total = R_z@R_y@R_x

        # transpose rotation matrix
        # x = R_trans@X # return relative to body-fixed (rotated) coordinates
        R_trans = np.transpose(R_total)

        # transform from space-fixed (world) to body-fixed coordinates
        vector = R_trans@vector

        return vector

    def gen_control_force(self):

        # TODO: Use PI control
        # calculate forces
        K = np.array([[30,0,0,0,0,0],
                      [0,30,0,0,0,0],
                      [0,0,30,0,0,0],
                      [0,0,0,30,0,0],
                      [0,0,0,0,30,0],
                      [0,0,0,0,0,30]])

        if self.cmd_vel_vector is not None and self.vel_vector is not None:
            # velocity command and velocity response received
            error = self.cmd_vel_vector - self.vel_vector

        elif self.cmd_vel_vector is None or self.vel_vector is None:
            # command or response has not been received yet
            error = np.zeros(6)

        # forces and torques in body-fixed coordinates
        F = K@error
        F = np.zeros(6)
        # ramp input for testing responses
        if self.effort_time_steps*self.effort_pub_rate<=25:
            F[2] = (100/25)*self.effort_time_steps*self.effort_pub_rate
        elif self.effort_time_steps*self.effort_pub_rate<=50:
            F[2] = -(100/25)*(self.effort_time_steps*self.effort_pub_rate-50)
        elif self.effort_time_steps*self.effort_pub_rate>50:
            F[2] = 0
        else:
            print("Something is wrong with the ramp function.")

        return F

    def gen_control_effort(self):

        # calculate required motor torques/effort
        # F = A@Torque
        # Torque = A_pinv@F
        A = np.array([[-0.707, -0.707, 0.707, 0.707, 0.0, 0.0, 0.0, 0.0],
                      [0.707, -0.707, 0.707, -0.707, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0, -1.0, -1.0, 1.0],
                      [0.0, 0.0, 0.0, 0.0, -0.218, -0.218, 0.218, 0.218],
                      [0.0, 0.0, 0.0, 0.0, -0.12, 0.12, -0.12, 0.12],
                      [0.1888, -0.1888, -0.1888, 0.1888, 0.0, 0.0, 0.0, 0.0]])

        # pseudo inverse of A
        A_pinv = np.array([[-3.53606789e-01,  3.53606789e-01,  2.94020928e-16, -9.54893657e-17,  1.15648232e-16,  1.32415254e+00],
                           [-3.53606789e-01, -3.53606789e-01, -2.94020928e-16, -9.54893657e-17,  1.15648232e-16, -1.32415254e+00],
                           [ 3.53606789e-01,  3.53606789e-01, -2.94020928e-16, -9.54893657e-17,  1.15648232e-16, -1.32415254e+00],
                           [ 3.53606789e-01, -3.53606789e-01,  2.94020928e-16, -9.54893657e-17,  1.15648232e-16,  1.32415254e+00],
                           [ 6.26367697e-34, -1.50165300e-17,  2.50000000e-01, -1.14678899e+00, -2.08333333e+00, -3.75413250e-18],
                           [-6.26367697e-34,  1.50165300e-17, -2.50000000e-01, -1.14678899e+00,  2.08333333e+00,  3.75413250e-18],
                           [-6.26367697e-34,  1.50165300e-17, -2.50000000e-01, 1.14678899e+00, -2.08333333e+00,  3.75413250e-18],
                           [ 6.26367697e-34, -1.50165300e-17,  2.50000000e-01, 1.14678899e+00,  2.08333333e+00, -3.75413250e-18]])

        F = self.gen_control_force()

        Torque = A_pinv@F
        # print(Torque)


        # pad with extra zeros 
        data = [0.0, 0.0, 0.0, 0.0, 0.0, Torque[0], Torque[1], Torque[2], Torque[3], Torque[4], Torque[5], Torque[6], Torque[7]]
        # data = [Torque[0], Torque[1], Torque[2], Torque[3], Torque[4], Torque[5], Torque[6], Torque[7], 0.0, 0.0, 0.0, 0.0, 0.0]
        # data = [Torque[0], Torque[5], 0.0, 0.0, Torque[3], Torque[1], 0.0, 0.0, Torque[2], Torque[7], Torque[6], 0.0, Torque[4]]

        dim = []

        layout = MultiArrayLayout(dim=dim, data_offset=0)

        effort_msg = Float64MultiArray(layout=layout, data=data)

        return effort_msg

def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()