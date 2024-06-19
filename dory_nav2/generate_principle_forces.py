#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# MSGs
from std_msgs.msg import String, Float32MultiArray, MultiArrayLayout, Float64MultiArray
from geometry_msgs.msg import Twist
from control_msgs.msg import DynamicJointState

# MATH
import numpy as np
from numpy import cos, sin
from scipy import linalg

# For multi-threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# TODO: Bag cmd_vel

class PubSubNode(Node):
    def __init__(self,waypoints_enabled=False,debug=False):
        super().__init__('pub_sub_node')

        # For multi-threading
        self.reentrant_group_1 = ReentrantCallbackGroup()

        # For waypoint functionality.
        self.waypoints_enabled=waypoints_enabled
        # For debug functionality
        self.debug=debug
        # For External Control functionality 
        self.input_ctrls=[0.0,0.0,0.0,0.0,0.0,0.0]

        # DON't NEED THIS////////////////////////////////////////////////////////////////
        if not waypoints_enabled:
            # Subscribers
            self.sub_cmd_vel = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10,
                callback_group=self.reentrant_group_1
            )

        self.sub_vel = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.vel_callback,
            10,
            callback_group=self.reentrant_group_1
        )

        ## Timers
        # Effort
        self.effort_pub_rate = 0.002
        self.effort_timer = self.create_timer(self.effort_pub_rate, self.effort_publish_message, callback_group=self.reentrant_group_1)

        # Only publish if debugging enabled. FIX LATER////////////////////////////////////////////////////////
        if self.debug:
            # Thrust
            self.state_thrust_pub_rate = 0.25
            self.state_thrust_timer = self.create_timer(self.state_thrust_pub_rate, self.state_thrust_publish_message, callback_group=self.reentrant_group_1)


        ## Publishers
        # Only publish if debugging enabled. FIX LATER///////////////////////////////////////////////////////
        if self.debug:
            self.state_thrust_publisher = self.create_publisher(Float64MultiArray, '/state_thrust_vector', 10)


        self.effort_publisher = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)
        #self.effort_publisher = self.create_publisher(DynamicJointState, 'dynamic_joint_states', 10)


        # define cmd_vel_vector and vel_vector
        self.cmd_vel_vector = None
        self.vel_vector = None

        self.effort_time_steps = 0

        # state vector
        self.state_vector = np.zeros(12)

######
# Timer Publishers
######
    def effort_publish_message(self):
        effort = self.gen_control_effort()
        self.effort_publisher.publish(effort)
        # self.get_logger().info(f"\nPublished the thrust. Woot! {effort}\n")
        self.effort_time_steps += 1

# FIX LATER //////////////////////////////////////////////////////////////////////////////////
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

        # Msg pub
        dim = []
        layout = MultiArrayLayout(dim=dim, data_offset=0)
        state_thrust_msg = Float64MultiArray(layout=layout, data=data)
        self.state_thrust_publisher.publish(state_thrust_msg)
        # self.get_logger().info(f"Published the state thrust. Woot! {state_thrust_msg}")

#####
# Timer Publishers 
#####

#####
# Subscribers
#####
# FIX LATER //////////////////////////////////////////////////////////////////////////////////
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
#####
# Subscribers
#####

#####
# Helper Functions
#####

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

    def gen_control_effort(self):

        # calculate required motor torques/effort
        # F = A@Torque
        # TYPE 1
        # Torque = A_pinv@F
        A = np.array([[-0.707, -0.707,   0.707,  0.707,  0.0,    0.0,   0.0,   0.0],
                      [0.707,  -0.707,   0.707, -0.707,  0.0,    0.0,   0.0,   0.0],
                      [0.0,     0.0,     0.0,    0.0,    1.0,   -1.0,  -1.0,   1.0],
                      [0.0,     0.0,     0.0,    0.0,   -0.218, -0.218, 0.218, 0.218],
                      [0.0,     0.0,     0.0,    0.0,   -0.12,   0.12, -0.12,  0.12],
                      [0.1888, -0.1888, -0.1888, 0.1888, 0.0,    0.0,   0.0,   0.0]])
        # TYPE 2


        # pseudo inverse of A
        # A_pinv = np.array([[-3.53606789e-01,  3.53606789e-01,  2.94020928e-16, -9.54893657e-17,  1.15648232e-16,  1.32415254e+00],
        #                    [-3.53606789e-01, -3.53606789e-01, -2.94020928e-16, -9.54893657e-17,  1.15648232e-16, -1.32415254e+00],
        #                    [ 3.53606789e-01,  3.53606789e-01, -2.94020928e-16, -9.54893657e-17,  1.15648232e-16, -1.32415254e+00],
        #                    [ 3.53606789e-01, -3.53606789e-01,  2.94020928e-16, -9.54893657e-17,  1.15648232e-16,  1.32415254e+00],
        #                    [ 6.26367697e-34, -1.50165300e-17,  2.50000000e-01, -1.14678899e+00, -2.08333333e+00, -3.75413250e-18],
        #                    [-6.26367697e-34,  1.50165300e-17, -2.50000000e-01, -1.14678899e+00,  2.08333333e+00,  3.75413250e-18],
        #                    [-6.26367697e-34,  1.50165300e-17, -2.50000000e-01,  1.14678899e+00, -2.08333333e+00,  3.75413250e-18],
        #                    [ 6.26367697e-34, -1.50165300e-17,  2.50000000e-01,  1.14678899e+00,  2.08333333e+00, -3.75413250e-18]])

        A_pinv = np.linalg.pinv(A)

        # F is 1x6 matrix. 
        # F[2] only val filled. So only A_Pinv col 3 used. 
        F = self.gen_control_force()

        # Matrix Multiplication. Gives 8x1 list currently. 
        Torque = A_pinv@F
        #print(Torque)


        # pad with extra zeros 
        data = [0.0, 0.0, 0.0, 0.0, 0.0, Torque[0], Torque[1], Torque[2], Torque[3], Torque[4], Torque[5], Torque[6], Torque[7]]

        # Equal Thrust
        thrust = 0.5
        #data = [thruster1, thruster6, 0.0, 0.0, thruster4, thruster2, 0.0, 0.0, IMU, thruster3, thruster8, thruster7, 0.0, thruster5]
        # Thruster 1-4 on the bottom. Thrusters 5-8 on top. 
        #data = np.zeros(13)
        # # NOTHING
        # data[0]=thrust
        # # NOTHING
        # data[1]=thrust
        # # NOTHING
        # data[2]=thrust
        # # NOTHING
        # data[3]=thrust
        # # NOTHING
        # data[4]=thrust
        # # PURE ROLL///////////////////////////////////////////////////////
        #data[5]=thrust
        # # DONT KNOW. Roll and Pitch///////////////////////////////////////
        # data[6]=10.0
        # # NOTHING
        # data[7]=thrust
        # # Opposite of 6 ????/////////////////////////////////////////////
        # data[8]=-10.0
        # # Pitch?///////////////////////////////////////////////////
        # data[9]=thrust
        # # NOTHING?????????????????????/////////////////////////////
        # data[10]=thrust
        # # BACK RIGHT DOWN////////////////////////////////////////
        # data[11]=10.0
        # NOTHING???????????????????/////////////////////////////
        # data[12]=thrust


        # data = [Torque[0], Torque[1], Torque[2], Torque[3], Torque[4], Torque[5], Torque[6], Torque[7], 0.0, 0.0, 0.0, 0.0, 0.0]
        # data = [Torque[0], Torque[5], 0.0, 0.0, Torque[3], Torque[1], 0.0, 0.0, Torque[2], Torque[7], Torque[6], 0.0, Torque[4]]

        # Msg saving. 
        # data = np.zeros(13)
        dim = []
        layout = MultiArrayLayout(dim=dim, data_offset=0)
        effort_msg = Float64MultiArray(layout=layout, data=data)


        # self.get_logger().warning("\nSTUFF: {0}\n".format(1))

        return effort_msg

    def gen_control_force(self,Fx=0.0,Fy=0.0,Fz=0.0,Tx=0.0,Ty=0.0,Tz=0.0):

        # TODO: Use PI control
        # calculate forces
        K = np.array([[30,0,0,0,0,0],
                      [0,30,0,0,0,0],
                      [0,0,30,0,0,0],
                      [0,0,0,30,0,0],
                      [0,0,0,0,30,0],
                      [0,0,0,0,0,30]])

        # Based on:
            # cmd_vel_vector published via generate command_vel
            # vel_vector taken from IMU

        #NOTE THE DEBUG FLAG MESSES WITH THIS //////////////////////////////////////////////////

        # If get vals from both:
        if self.cmd_vel_vector is not None and self.vel_vector is not None:
            # velocity command and velocity response received
            error = self.cmd_vel_vector - self.vel_vector
        # Otherwise
        elif self.cmd_vel_vector is None or self.vel_vector is None:
            # command or response has not been received yet
            error = np.zeros(6)

        # forces and torques in body-fixed coordinates

        # Matrix multiplication
        F = K@error

        # ????? And then 0's ???????
        F = np.zeros(6)

        # ????? And then 0's ???????

        # ramp input for testing responses//////////////////////////////////////////////////////////////////////////////////
        if self.debug:
            i =3 
            if self.effort_time_steps*self.effort_pub_rate<=25:
                F[i] = (100/25)*self.effort_time_steps*self.effort_pub_rate
            elif self.effort_time_steps*self.effort_pub_rate<=50:
                F[i] = -(100/25)*(self.effort_time_steps*self.effort_pub_rate-50)
            elif self.effort_time_steps*self.effort_pub_rate>50:
                F[i] = 0
            else:
                print("Something is wrong with the ramp function.")
        else:
            # INPUT FORCE CONTROLs. DO PID somewhere else. 
            F = self.input_ctrls

        return F

#####
# Helper functions
#####

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Quaternion

#####
# Waypoint P Controller Class 
# MAKE AFTER PubSubNode
#####
class DoryWaypointFollower(Node):
    def __init__(self, object):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('dory_waypoint_follower')

        # Timer determines the publish rate of the waypoint controls.
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback,callback_group=object.reentrant_group_1)

        # To store PubSubNode to access contents 
        self.object = object
        # To store Force and Torque Vectors
        self._ctrl_inputs = [0.0,0.0,0.0,0.0,0.0,0.0]

        # TF for BUOYANCY CONTROL
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Timer for Buoyancy CTRL FOR NOW
        #self.timer = self.create_timer(self.timer_period, self.anti_buoyancy_ctrl,callback_group=object.reentrant_group_1)


        ### DYNAMIC RECONFIG https://hippocampusrobotics.github.io/fav_docs/tutorials/dynamic_reconfigure.html
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('gains.p', rclpy.Parameter.Type.DOUBLE),
        #         ('gains.i', rclpy.Parameter.Type.DOUBLE),
        #         ('gains.d', rclpy.Parameter.Type.DOUBLE),
        #     ],
        # )
        # the calls to get_parameter will raise an exception if the paramter
        # value is not provided when the node is started.
        # param = self.get_parameter('gains.p')
        # self.get_logger().info(f'{param.name}={param.value}')
        # self.p_gain = param.value

        # param = self.get_parameter('gains.i')
        # self.get_logger().info(f'{param.name}={param.value}')
        # self.i_gain = param.value

        # param = self.get_parameter('gains.d')
        # self.get_logger().info(f'{param.name}={param.value}')
        # self.d_gain = param.value

        # self.add_on_set_parameters_callback(self.on_params_changed)

        ### DYNAMIC RECONFIG

        # To store current robot navigation data
        self.robot_data = {"CurrentXYZ":[0.0,0.0,0.0],"WaypointXYZ":[[0.0,1.0,0.0]],"Time":0.0}
        # To store robot updates
        self.robot_err = {"orient_err":{"xy":0.0,"xz":0.0},"xyz_err":[0.0,0.0,0.0],"dist_err":0.0}

# DYNAMIC RECONFIG CALLBACK
    # def on_params_changed(self, params):
    #     param: rclpy.Parameter

    #     for param in params:
    #         self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
    #         if param.name == 'gains.p':
    #             self.p_gain = param.value
    #         elif param.name == 'gains.i':
    #             self.i_gain = param.value
    #         elif param.name == 'gains.d':
    #             self.d_gain = param.value
    #         else:
    #             continue

    #     return SetParametersResult(succesful=True, reason='Parameter set')

        # PID CTRL
        self.kp = np.zeros(6)
        self.ki = np.zeros(6)
        self.kd = np.zeros(6)
        self.integral_error = np.zeros(6)
        self.error_last = np.zeros(6)
        self.derivative_error = np.zeros(6)

        # Anti buoyancy
        self.flat = False
        self.orient=[0,0,0]


# Runs based on defined refresh rate
    def timer_callback(self):
        # Time update for DEBUG
        # self.robot_data["Time"]=self.get_clock().now()
        # First retrieve the current X,Y,Z positions. 
        # self.robot_data["CurrentXYZ"]=self.object.state_vector[0:3]

        #print(self.robot_data)

        # Recursively feed waypoints seeking. TO DO LOOP ////////////////////////////////////////////////
        # self.p_controller(p_gains=[0.0,0.0,0.0,0.0,0.0,0.3],waypoint=self.robot_data["WaypointXYZ"][0])
        
        # Input control forces 
        #self._ctrl_inputs[0]=0.1
        # self._ctrl_inputs[1]=-0.1
        # self._ctrl_inputs[2]=2.5
        # self._ctrl_inputs[3]=0.5
        # self._ctrl_inputs[4]=0.5
        self._ctrl_inputs[5]=-1.5

        self.object.input_ctrls=self._ctrl_inputs

# CONTROLLER. Based on gains for principle forces and torques. Waypoint based on desired x,y,z.
    def p_controller(self,p_gains=np.ones(6),waypoint=[0.0,0.0,0.0]):
        # Call the force handle object to achieve waypoint. 
        ## Orientation. 
        # XY ORIENTATION
        orient_error_xy=np.arctan2(np.subtract(self.robot_data["CurrentXYZ"][1],waypoint[1]), np.subtract(self.robot_data["CurrentXYZ"][0],waypoint[0]))
        self.robot_err["orient_err"]["xy"] = orient_error_xy
        
        # # XZ ORIENTATION
        # orient_error_xz=self.angle_wrap(np.arctan2(np.subtract(self.robot_data["CurrentXYZ"][2],waypoint[2]), np.subtract(self.robot_data["CurrentXYZ"][0],waypoint[0])))
        # self.robot_err["orient_err"]["xz"] = orient_error_xz

        ## OFFSETS 
        for i,coord in enumerate(self.robot_err["xyz_err"]):
            self.robot_err["xyz_err"][i]=np.subtract(waypoint[i],self.robot_data["CurrentXYZ"][i])

        # Get DISTANCE 
        self.robot_err["dist_err"]=np.sqrt(np.square(self.robot_err["xyz_err"][0])+np.square(self.robot_err["xyz_err"][1])+np.square(self.robot_err["xyz_err"][2]))

        # DEBUG np.multiply(p_gains[0],self.robot_err["dist_err"])
        # print(self.robot_err)
        
        ## CONTROLS
        # Reorient in XY Plane. Go opposite of the angle diff. 
        #self._ctrl_inputs[4]=-np.multiply(p_gains[4],self.robot_err["orient_err"]["xy"])
        #self._ctrl_inputs[5]=-np.multiply(p_gains[5],self.robot_err["orient_err"]["xy"])

        # HOVER TEST
        # self._ctrl_inputs[5]=1.0


        # LOGS
        # self.get_logger().warning("\nOrient XY err: {0},\nDist err: {1}\n".format(orient_error_xy,self.robot_err["dist_err"]))
        #self.get_logger().warning("\nZ err: {0}\n".format(self.robot_err["xyz_err"][2]))
        #self.get_logger().warning("\nCtrl x: {0},\nCtrl y: {1},\nCtrl z: {2}\n".format(self._ctrl_inputs[0],self._ctrl_inputs[1],self._ctrl_inputs[2]))
        #self.get_logger().warning("\nCtrl Roll: {0},\nCtrl Pitch: {1},\nCtrl Yaw: {2}\n".format(self._ctrl_inputs[3],self._ctrl_inputs[4],self._ctrl_inputs[5]))

        # Input control forces 
        self.object.input_ctrls=self._ctrl_inputs
        return
    

    def anti_buoyancy_ctrl(self):

        # Retrieve the roll, pitch, yaw data from the imu. 
        roll,pitch,yaw = self.object.state_vector[3],self.object.state_vector[4],self.object.state_vector[5]

        # Reorient if not flat
        if not self.flat:

            ang_tol = 3.0 # deg 
            ang = 0.0
            # Perform torques if not within angle tolerance.
            # Roll 
            i = 3   
            if (roll>ang+ang_tol) or (roll<ang-ang_tol):
                self.kp[i] = 10.0
                self.ki[i] = 0.0
                self.kd[i] = 5.0
                ctrl_roll = self.computePID(-roll,index=i)
                self._ctrl_inputs[i]=ctrl_roll
                self.orient[0]=0
            else:
                self._ctrl_inputs[i]=0.0
                self.orient[0]=1
            # Pitch 
            i = 4 
            if (pitch>ang+ang_tol) or (pitch<ang-ang_tol):
                self.kp[i] = 45.0
                self.ki[i] = 0.0
                self.kd[i] = 10.0
                ctrl_pitch = self.computePID(-pitch,index=i)
                self._ctrl_inputs[i]=ctrl_pitch
                self.orient[1]=0
            else:
                self._ctrl_inputs[i]=0.0
                self.orient[1]=1
            # yaw 
            i = 4 
            if (yaw>ang+ang_tol) or (yaw<ang-ang_tol):
                self.kp[i] = 10.0
                self.ki[i] = 0.0
                self.kd[i] = 5.0
                ctrl_yaw = self.computePID(-yaw,index=i)
                self._ctrl_inputs[i]=ctrl_yaw
                self.orient[2]=0
            else:
                self._ctrl_inputs[i]=0.0
                self.orient[2]=1

            # If all oriented, then can dive. 
            if self.orient == [1,1,1]:
                self.flat=True

        # Otherwise Dive
        else:
            i = 2   
            self.kp[i] = 260.0
            self.ki[i] = 0.0
            self.kd[i] = 180.0
            ctrl_z = self.computePID(-self.object.state_vector[2],index=i)
            self._ctrl_inputs[i]=ctrl_z 

            # To check again before diving again.
            self.flat=False 

        self.object.input_ctrls=self._ctrl_inputs

        self.get_logger().warning("\n Roll: {0},\n Pitch: {1},\n Yaw: {2}".format(roll,pitch,yaw))

        return

    # def anti_buoyancy_ctrl(self):
    #     from_frame_rel = 'base_link'
    #     to_frame_rel = 'alphabase_footprint'

    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             to_frame_rel,
    #             from_frame_rel,
    #             rclpy.time.Time())
    #         # Attempt to lookup TF. If cannot do so, exit. 
    #     except TransformException as ex:
    #         self.get_logger().info(
    #             f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
    #         return

    #     # Create Inverse for ctrls.
    #     roll,pitch,yaw = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
        
    #     # If can lookup TF, add thrust commands to hover based on this. 
    #     # XYZ
    #     # p1 = 0.5
    #     # trans_x= -p1 * t.transform.translation.x
    #     # self._ctrl_inputs[0]=trans_x
    #     i = 0   
    #     self.kp[i] = 10.0
    #     self.ki[i] = 0.0
    #     self.kd[i] = 5.0
    #     ctrl_x = self.computePID(-t.transform.translation.x,index=i)
    #     self._ctrl_inputs[i]=ctrl_x  

    #     # p2 = 0.5
    #     # trans_y= -p2 * t.transform.translation.y 
    #     # self._ctrl_inputs[1]=trans_y  
    #     i = 1   
    #     self.kp[i] = 10.0
    #     self.ki[i] = 0.0
    #     self.kd[i] = 5.0
    #     ctrl_y = self.computePID(-t.transform.translation.y,index=i)
    #     self._ctrl_inputs[i]=ctrl_y   

    #     # p3 = 1.0
    #     # fz_offset = -9.8
    #     # trans_z= -p3 * t.transform.translation.z + fz_offset
    #     # self._ctrl_inputs[2]=trans_z
    #     i = 2   
    #     self.kp[i] = 260.0
    #     self.ki[i] = 0.0
    #     self.kd[i] = 180.0
    #     ctrl_z = self.computePID(-t.transform.translation.z,index=i)
    #     self._ctrl_inputs[i]=ctrl_z   


    #     # # # RPY
    #     # p4 = 30.0
    #     # rot_roll = t.transform.rotation.x
    #     # self._ctrl_inputs[3]=-p4*roll

    #     i = 3   
    #     self.kp[i] = 0.4
    #     self.ki[i] = 0.025
    #     self.kd[i] = 0.1
    #     err = -t.transform.rotation.x/3.14
    #     ctrl_roll = self.computePID(err,index=i)
    #     self._ctrl_inputs[i]=ctrl_roll   

    #     # p5 = 15.0
    #     # rot_pitch = t.transform.rotation.y
    #     # self._ctrl_inputs[4]=-p5*pitch

    #     i = 4   
    #     self.kp[i] = 0.9
    #     self.ki[i] = 0.0
    #     self.kd[i] = 0.1
    #     # Scale the values between 0 and 3.14 
    #     err = -t.transform.rotation.y/3.14
    #     ctrl_pitch = self.computePID(err,index=i)
    #     self._ctrl_inputs[i]=ctrl_pitch  

    #     # p6 = 15.0
    #     # rot_yaw = t.transform.rotation.z
    #     # self._ctrl_inputs[5]=-p6*yaw

    #     # i = 5   
    #     self.kp[i] = 2.0
    #     self.ki[i] = 0.4
    #     self.kd[i] = 1.0
    #     err = t.transform.rotation.z/3.14
    #     ctrl_yaw = self.computePID(err,index=i)
    #     self._ctrl_inputs[i]=ctrl_yaw 

    #     # Input control forces 
    #     self.object.input_ctrls=self._ctrl_inputs
    #     #self.object.input_ctrls=np.zeros(6)
        
    #     #self.get_logger().warning("\n Roll: {0},\n Pitch: {1},\n Yaw: {2}".format(roll/3.14,pitch/3.14,yaw/3.14))
    #     #self.get_logger().warning("\n x: {0},\n y: {1},\n z: {2}".format(t.transform.translation.x,t.transform.translation.y,t.transform.translation.z))
    #     #self.get_logger().warning("\nCtrl x: {0},\nCtrl y: {1},\nCtrl z: {2}".format(self._ctrl_inputs[0],self._ctrl_inputs[1],self._ctrl_inputs[2]))
    #     #self.get_logger().warning("\nCtrl Roll: {0},\nCtrl Pitch: {1},\nCtrl Yaw: {2}\n".format(self._ctrl_inputs[3],self._ctrl_inputs[4],self._ctrl_inputs[5]))
    #     return
    
    def computePID(self,error,index):
        self.integral_error[index]+=error*self.timer_period
        self.derivative_error[index]=(error-self.error_last[index])/self.timer_period
        self.error_last[index]=error
        output=self.kp[index]*error+self.ki[index]*self.integral_error[index]+self.kd[index]*self.derivative_error[index]
        return output


#####
# Waypoint P Controller Class 
#####

##########################################################################
# Helper Functions
##########################################################################

    def angle_wrap(self, ang):
        '''
        Return the angle normalized between [-pi, pi].

        Works with numbers and numpy arrays.

        :param ang: the input angle/s.
        :type ang: float, numpy.ndarray
        :returns: angle normalized between [-pi, pi].
        :rtype: float, numpy.ndarray
        '''
        ang = ang % (2 * np.pi)
        if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
            ang -= 2 * np.pi
        elif isinstance(ang, np.ndarray):
            ang[ang > np.pi] -= 2 * np.pi
        return ang

import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

############
# Dynamic Reconfigure 
############

from rcl_interfaces.msg import SetParametersResult

class DynamicReconfig(Node):

    def __init__(self, object):

        super().__init__(node_name='dynamic_reconfigure')
        # To give access to all other objects in script. 
        self.object = object
        self.init_params()

    # Dynamic Reconfigure init and config. 
    def init_params(self):
        # Must be declared for params to be accessible. 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains.p', rclpy.Parameter.Type.DOUBLE),
                ('gains.i', rclpy.Parameter.Type.DOUBLE),
                ('gains.d', rclpy.Parameter.Type.DOUBLE),
            ],
        )

        # the calls to get_parameter will raise an exception if the paramter
        # value is not provided when the node is started.
        param = self.get_parameter('gains.p')
        self.get_logger().info(f'{param.name}={param.value}')
        self.p_gain = param.value

        param = self.get_parameter('gains.i')
        self.get_logger().info(f'{param.name}={param.value}')
        self.i_gain = param.value

        param = self.get_parameter('gains.d')
        self.get_logger().info(f'{param.name}={param.value}')
        self.d_gain = param.value

        # To be called everytime ANYTHING is changed in RQT.
        self.add_on_set_parameters_callback(self.on_params_changed)


    def on_params_changed(self, params):
        param: rclpy.Parameter
        # Checking all parameter changes. 
        for param in params:

            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'gains.p':
                self.p_gain = param.value

            elif param.name == 'gains.i':
                self.i_gain = param.value

            elif param.name == 'gains.d':
                self.d_gain = param.value
            else:
                # To prevent program freezing. 
                return SetParametersResult(successful=False, reason='Parameter not found')

            self.get_logger().info(f'{param.name} = {param.value} SET')

        return SetParametersResult(successful=True, reason='Parameter set')
##########################################################################
# Helper Functions
##########################################################################

def main(args=None):
    rclpy.init(args=args)
    # Waypoint enabled vs testing controls
    dory_pub_sub = PubSubNode(waypoints_enabled=True)
    # Must be made after dory_pub_sub
    dory_waypoints = DoryWaypointFollower(object=dory_pub_sub)
    # For TESTING reconfigure
    dory_reconfig = DynamicReconfig()

    # Multi-threading functionality.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(dory_pub_sub)
    executor.add_node(dory_waypoints)
    executor.add_node(dory_reconfig)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        dory_pub_sub.destroy_node()
        dory_waypoints.destroy_node()
        dory_reconfig.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()