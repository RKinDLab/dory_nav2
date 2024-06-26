#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# MSGs
from std_msgs.msg import MultiArrayLayout, Float64MultiArray, Header
from geometry_msgs.msg import Twist
from control_msgs.msg import DynamicJointState
# MATH
import numpy as np
from numpy import cos, sin
from scipy import linalg
# For multi-threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# Custom msg interface for data visualization 
from dory_visuals_msgs.msg import State

# To allow selective data handling. All objects have access to this object
# and its contents, but not the other way around. This is the Object master. 
class MainObjectHandle(Node):
    def __init__ (self,pub_state=False):
        super().__init__('dory_main_handle')

        # For handling multithreads. 
        self.reentrant_group_1 = ReentrantCallbackGroup()
        # For handling IMU, TF, and markers.
        self.reentrant_group_2 = ReentrantCallbackGroup()

        # TO DO: CHANGE THE SIM BASED ON IF INPUT WAYPTS OR SAVED PATH FROM CONFIG OR 
        # Path generation function.
        # Configuration: Path or Waypoint enabled. 
        self.configuration = {"Waypoint":False}

        # Publish rate handle
        self.rates={"MainObjectHandle":{"publish_state_message":0.2},
                           "ImuHandle":{"dynamic_joint_states_callback":0.0},
                           "EffortHandle":{"effort_publish_message":0.002},
                           "WaypointFollower":{"timer_callback":0.02},
                           "Dynamic_Reconfig":None,
                           "PDControl":None,
                           "OdomPublisher":{"tf_odom":None},
                           "WaypointVisuals":{"timer_callback":5.0}}
        self.header_main = Header() # To be copied from the IMU, because publishing based on that data. 

        # For handling body fixed body based (F: xyz,rpy) input controls.
        self.input_ctrls = np.zeros(6)
        # PD parameters for waypoint following. Based on Dynamic Reconfigure Config. 
        # Can be reconfigured realtime.
        self.kp = np.zeros(6)
        self.kd = np.zeros(6)

        # Current state: [x y z wq xq yq zq u v w p q r]
        self.state_current = [0.0]*13
        # Current state simple: [xyz rpy]
        # To prevent waypoint list follower from skipping first index. 
        self.state_current_simple = [100.0]*6

        # for Plotjuggler plotting. Human readable data. 
        self.pub_state = pub_state
        # For waypoint on access. 
        self.waypoint_on = [0.0]*6
        self.waypoint_on = list(map(float,self.waypoint_on))

        if self.pub_state:
            ## Pubs for Roll, Pitch, Yaw.
            self._state_timer = self.create_timer(self.rates["MainObjectHandle"]["publish_state_message"], self.publish_state_message, callback_group=self.reentrant_group_1)

            ## Publishers
            self.state_visuals = State()
            self._publish_state = self.create_publisher(State, '/dory/visuals/state', 10)

    def publish_state_message(self):
        # Header same as IMU
        self.state_visuals.header = self.header_main
        # Position Data
        self.state_visuals.pose.position.x = self.state_current[0]
        self.state_visuals.pose.position.y = self.state_current[1]
        self.state_visuals.pose.position.z = self.state_current[2]
        # Euler Orientation
        self.state_visuals.pose.orientation.euler.roll = self.state_current_simple[3]
        self.state_visuals.pose.orientation.euler.pitch = self.state_current_simple[4]
        self.state_visuals.pose.orientation.euler.yaw = self.state_current_simple[5]
        # Velocity Linear
        self.state_visuals.twist.linear.x = self.state_current[7]
        self.state_visuals.twist.linear.y = self.state_current[8]
        self.state_visuals.twist.linear.z = self.state_current[9]
        # Velocity Angular
        self.state_visuals.twist.angular.x = self.state_current[10]   
        self.state_visuals.twist.angular.y = self.state_current[11]       
        self.state_visuals.twist.angular.z = self.state_current[12]     
        # Body Forces X Y Z
        self.state_visuals.forces.fx = float(self.input_ctrls[0])
        self.state_visuals.forces.fy = float(self.input_ctrls[1])
        self.state_visuals.forces.fz = float(self.input_ctrls[2])
        # Torque Forces X Y Z
        self.state_visuals.forces.tx = float(self.input_ctrls[3])
        self.state_visuals.forces.ty = float(self.input_ctrls[4])
        self.state_visuals.forces.tz = float(self.input_ctrls[5])
        # Waypoint on
        self.state_visuals.waypoint = self.waypoint_on

        self._publish_state.publish(self.state_visuals)
        return

# To handle pose and velocity data
class ImuHandle(Node):
    def __init__(self,object_master):
        super().__init__('dory_imu_handle')
        # For master object access. 
        self.object_master = object_master

        # For finding the IMU subscription rate. 
        self.prev_time = None

        self.sub_vel = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_states_callback,
            10,
            callback_group=object_master.reentrant_group_2
        )
        # For time synch across nodes. 
        self.header_stamp = None

    # Subscribers
    def dynamic_joint_states_callback(self, msg):
        # Pull the header if visual publications on. 
        if self.object_master.pub_state:
            self.object_master.header_main = msg.header

        # Positions XYZ
        x = msg.interface_values[8].values[9]
        y = msg.interface_values[8].values[2]
        z = msg.interface_values[8].values[0]
        # Orientation
        ox = msg.interface_values[8].values[10]
        oy = msg.interface_values[8].values[6]
        oz = msg.interface_values[8].values[3]
        ow = msg.interface_values[8].values[1]
        # Velocities XYZ
        x_dot = msg.interface_values[8].values[4]
        y_dot = msg.interface_values[8].values[5]
        z_dot = msg.interface_values[8].values[7]
        # RPY Rates
        roll_dot = msg.interface_values[8].values[8]
        pitch_dot = msg.interface_values[8].values[11]
        yaw_dot = msg.interface_values[8].values[12]
        
        # PD Input is current state. 
        self.object_master.state_current=[x,y,z,ow,ox,oy,oz,x_dot,y_dot,z_dot,roll_dot,pitch_dot,yaw_dot]
        # For simplified current state.
        roll,pitch,yaw = euler_from_quaternion(ox,oy,oz,ow)
        self.object_master.state_current_simple=[x,y,z,roll,pitch,yaw]

        # For getting the sub frequency for timing. 
        self. imu_rate_get(secs=msg.header.stamp.sec,nanosecs=msg.header.stamp.nanosec)
    # For getting the subscription rate of the IMU 
    def imu_rate_get(self,secs,nanosecs):
        current_time = float(str(secs)+str(nanosecs))
        # Copy time, get time diff, get sub freq in hz. 
        if self.prev_time == None:
            self.prev_time = current_time
        else:
            # Find rate in hz and publish.
            time_diff = 1/((current_time-self.prev_time)/(10**9))
            # For smoothing subscription rates
            if time_diff > 0.1:
                self.object_master.rates["ImuHandle"]["dynamic_joint_states_callback"] = time_diff
                # self.get_logger().info(f'{self.object_master.rates}')
            self.prev_time = current_time

# Effort publish to thrusters and body force-> thruster mapping.
class EffortHandle(Node):
    def __init__(self,object_master):
        super().__init__('dory_effort_handle')
        # For master object access. 
        self.object_master = object_master

        ## Effort pub. Pubs for 8 thruster jts.
        self.effort_timer = self.create_timer(self.object_master.rates["EffortHandle"]["effort_publish_message"], self.effort_publish_message, callback_group=object_master.reentrant_group_1)

        ## Publishers
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)
        self.effort_time_steps = 0

    # Timer Publishers
    def effort_publish_message(self):
        effort = self.gen_control_effort()
        self.effort_publisher.publish(effort)
        # self.get_logger().info(f"\nPublished the thrust. Woot! {effort}\n")
        self.effort_time_steps += 1

    # Conversion from body fixed forces to thruster forces.
    def gen_control_effort(self):

        # calculate required motor torques/effort from body based
        A = np.array([[-0.707, -0.707,   0.707,  0.707,  0.0,    0.0,   0.0,   0.0],
                      [0.707,  -0.707,   0.707, -0.707,  0.0,    0.0,   0.0,   0.0],
                      [0.0,     0.0,     0.0,    0.0,    1.0,   -1.0,  -1.0,   1.0],
                      [0.0,     0.0,     0.0,    0.0,   -0.218, -0.218, 0.218, 0.218],
                      [0.0,     0.0,     0.0,    0.0,   -0.12,   0.12, -0.12,  0.12],
                      [0.1888, -0.1888, -0.1888, 0.1888, 0.0,    0.0,   0.0,   0.0]])

        A_pinv = np.linalg.pinv(A)

        # F is 1x6 matrix body based forces: xyz rpy. 
        F = self.object_master.input_ctrls

        # Matrix Multiplication. Gives 8x1 list currently. 
        Torque = A_pinv@F

        # Pad with extra zeros. The first FIVE are for the arm. The last EIGHT are for the thrusters. 
        # The way the controllers are setup, need to give thruster commands. Sim converts these back to body based.
        data = [0.0, 0.0, 0.0, 0.0, 0.0, Torque[0], Torque[1], Torque[2], Torque[3], Torque[4], Torque[5], Torque[6], Torque[7]]

        # Msg saving. 
        dim = []
        layout = MultiArrayLayout(dim=dim, data_offset=0)
        effort_msg = Float64MultiArray(layout=layout, data=data)
        return effort_msg

# Waypoint P Controller Class 
class WaypointFollower(Node):
    def __init__(self, object_master,object_pd):
        super().__init__('dory_waypoints_handle')
        # For master object access. 
        self.object_master = object_master
        # For PD object access
        self.object_pd = object_pd

        # Timer determines the publish rate of the waypoint controls and PD requests.
        self.timer = self.create_timer(self.object_master.rates["WaypointFollower"]["timer_callback"], self.timer_callback,callback_group=object_master.reentrant_group_1)

        # To store robot updates
        err_deg = 10
        self.err_angle = err_deg *3.14/180 # quat
        self.err_dist = 0.1 # m

        # Waypoints
        # Set 1
        right = 0.0
        up = 1.57
        left = 3.14
        down=-1.57
        leftdown=-3.14

        self.waypoints = [
                # Forwards path
                [0.0,0.0,0.0,0.0,0.0,0.0],
                [5.0,0.0,0.0,0.0,0.0,0.0],
                [5.0,0.0,0.0,0.0,0.0,1.57],
                [5.0,1.0,0.0,0.0,0.0,1.57],
                [5.0,1.0,0.0,0.0,0.0,3.14],#
                [0.0,1.0,0.0,0.0,0.0,3.14],
                [0.0,1.0,0.0,0.0,0.0,1.57],#
                [0.0,2.0,0.0,0.0,0.0,1.57],
                [0.0,2.0,0.0,0.0,0.0,0.0],#
                [2.5,2.0,0.0,0.0,0.0,0.0],
                [5.0,2.0,0.0,0.0,0.0,0.0],
                [5.0,2.0,0.0,0.0,0.0,1.57],#
                [5.0,3.0,0.0,0.0,0.0,1.57],
                [5.0,3.0,0.0,0.0,0.0,3.14],#
                [0.0,3.0,0.0,0.0,0.0,3.14],
                [0.0,3.0,0.0,0.0,0.0,1.57],#
                [0.0,4.0,0.0,0.0,0.0,1.57],
                [0.0,4.0,0.0,0.0,0.0,0.0],#
                [2.5,4.0,0.0,0.0,0.0,0.0],
                [5.0,4.0,0.0,0.0,0.0,0.0],
                [5.0,5.0,0.0,0.0,0.0,1.57],
                [5.0,5.0,0.0,0.0,0.0,1.57],#
                [5.0,5.0,0.0,0.0,0.0,3.14],#
                [0.0,5.0,0.0,0.0,0.0,3.14],
                [0.0,5.0,0.0,0.0,0.0,1.57],#
                [0.0,6.0,0.0,0.0,0.0,1.57],
                [0.0,6.0,0.0,0.0,0.0,0.0],#
                [5.0,6.0,0.0,0.0,0.0,0.0],
                [5.0,6.0,0.0,0.0,0.0,1.57],#
                [5.0,7.0,0.0,0.0,0.0,1.57],
                [5.0,7.0,0.0,0.0,0.0,3.14],#
                [0.0,7.0,0.0,0.0,0.0,3.14],
                # Backwards path
                [0.0,7.0,0.0,0.0,0.0,right],
                [5.0,7.0,0.0,0.0,0.0,right],#
                [5.0,7.0,0.0,0.0,0.0,down],
                [5.0,6.0,0.0,0.0,0.0,down],#
                [5.0,6.0,0.0,0.0,0.0,leftdown],
                [0.0,6.0,0.0,0.0,0.0,leftdown],#
                [0.0,6.0,0.0,0.0,0.0,down],
                [0.0,5.0,0.0,0.0,0.0,down],#
                [0.0,5.0,0.0,0.0,0.0,right],
                [5.0,5.0,0.0,0.0,0.0,right],#
                [5.0,5.0,0.0,0.0,0.0,down],#
                [5.0,5.0,0.0,0.0,0.0,down],
                [5.0,4.0,0.0,0.0,0.0,leftdown],
                [2.5,4.0,0.0,0.0,0.0,leftdown],
                [0.0,4.0,0.0,0.0,0.0,leftdown],#
                [0.0,4.0,0.0,0.0,0.0,down],
                [0.0,3.0,0.0,0.0,0.0,down],#
                [0.0,3.0,0.0,0.0,0.0,right],
                [5.0,3.0,0.0,0.0,0.0,right],#
                [5.0,3.0,0.0,0.0,0.0,down],
                [5.0,2.0,0.0,0.0,0.0,down],#
                [5.0,2.0,0.0,0.0,0.0,leftdown],
                [2.5,2.0,0.0,0.0,0.0,leftdown],
                [0.0,2.0,0.0,0.0,0.0,leftdown],#
                [0.0,2.0,0.0,0.0,0.0,down],
                [0.0,1.0,0.0,0.0,0.0,down],#
                [0.0,1.0,0.0,0.0,0.0,right],
                [5.0,1.0,0.0,0.0,0.0,right],#
                [5.0,1.0,0.0,0.0,0.0,down],
                [5.0,0.0,0.0,0.0,0.0,down],
                [5.0,0.0,0.0,0.0,0.0,leftdown],
                [0.0,0.0,0.0,0.0,0.0,leftdown],
                ]
        # Set 2
        # self.waypoints = [
        #         [0.0,0.0,3.0,0.0,0.0,1.57],
        #         [0.0,0.0,3.0,0.0,0.0,3.14],
        #         [0.0,0.0,3.0,0.0,0.0,-3.14],
        #         [0.0,0.0,3.0,0.0,0.0,-1.57],
        #         [0.0,0.0,3.0,0.0,0.0,0.0]
        #         ]

        self.waypoint_index = 0
        # Waypoint val reached for xyz rpy goals.
        self.waypoint_reached = np.zeros(7)
        # Need this for all check to work.
        self.waypoint_reached[-1] = 1

    # Runs based on defined refresh rate
    def timer_callback(self):
        # waypoint_on is the goal waypoint. Based on set from rqt or loaded from path. 
        # FOLLOWING PATH. IF a waypoint is inserted by RQT, will seek it then return to the path. 
        # Update counter if NOT DONE WITH PATH. 
        if not self.object_master.configuration["Waypoint"]:
            # Check if goal reached 
            self.goal_check(goal=self.object_master.waypoint_on)  
            # Update waypoint seeking. Will NOT update if at end of list. 
            self.object_master.waypoint_on = self.waypoints[self.waypoint_index]

        # Input controls 
        # self.get_logger().info(f'{self.object_master.waypoint_on}')
        self.object_master.input_ctrls= self.object_pd.goal_handle(goal=self.object_master.waypoint_on) 

    # Goal needs to be [XYZ,RPY].
    # BECAUSE BEING CALLED BY A TIMER, CANNOT use a WHILE LOOP.
    def goal_check(self, goal):
        # Current state simple: [xyz rpy]
        # 1st check if waypt reached. 
        if not all(self.waypoint_reached):

            # Check all vals. If not all reached at once, then reset the vals.
            for i,val in enumerate(goal):
                # If i in xyz and not reached
                if i in [0,1,2]:
                    if self.object_master.state_current_simple[i] > val - self.err_dist and self.object_master.state_current_simple[i] < val + self.err_dist:
                        self.waypoint_reached[i]=1
                # If i in rpy and not reached
                elif i in [3,4,5]:
                    if self.object_master.state_current_simple[i] > val - self.err_angle and self.object_master.state_current_simple[i] < val + self.err_angle:
                        self.waypoint_reached[i]=1
                else:
                    self.waypoint_reached[0:6] = 0

                    
        # Then give response.
        # True if reached. Reset the waypoint reached param. 
        # Update the goal index. 
        if all(self.waypoint_reached):
            # Check if at end of waypoint list. Will NOT UPDATE if AT END OF LIST. 
            if self.waypoint_index==(len(self.waypoints)-1):
                self.get_logger().warning(f"\n AT END OF LIST. Current index: {self.waypoint_index}",once=True)
                return
            else:
                self.waypoint_index+=1

            # Reset. Update. 
            self.waypoint_reached[0:6] = 0
            self.get_logger().info(f"\n WAYPOINT REACHED. Next Goal: {self.waypoints[self.waypoint_index]}")
            return
        else:
            return

    # Outputs a list for the lawnpath to follow. 
    # def lawnpath_gen(self,pass_cnt=3,pass_len_x = 1,pass_len_y=2,quad=0):
        
# Dynamic Reconfigure
from rcl_interfaces.msg import SetParametersResult
class DynamicReconfig(Node):
    def __init__(self, object_master):
        super().__init__(node_name='dory_reconfig_handle')
        # To give selective access to all other objects info in script. 
        self.object_master = object_master
        self.init_params()

    # Dynamic Reconfigure init and config. 
    def init_params(self):
        param_type_str = rclpy.Parameter.Type.STRING
        param_type_bool = rclpy.Parameter.Type.BOOL
        params = ["gains.kp",'gains.kd','waypoint.seeking','configuration.following_waypoint']

        # Must be declared for params to be accessible. 
        self.declare_parameters(
            namespace='',
            parameters=[
                (params[0], param_type_str),
                (params[1], param_type_str),
                (params[2], param_type_str),
                (params[3], param_type_bool),
            ],
        )
        ## Init params.
        for i,val in enumerate(params):
            param = self.get_parameter(val)
            self.get_logger().info(f'{param.name}={param.value}')
            # PD Control, Waypoint Seeking.
            if i in [0,1,2]:
                # String Handle
                response = self.string_list_handle(input_str=param.value)
                # Gains kp, kd
                if i in [0,1]:
                    self.pd_change_handle(input_data=response,input_type=val[6:])
                # Waypoint Handle
                else:
                    self.waypoint_handle(input_data=response)
            # Follow set waypoint or path (set of)
            elif i == 3:
                self.object_master.configuration["Waypoint"] = param.value

        # To be called everytime ANYTHING is changed in RQT.
        self.add_on_set_parameters_callback(self.on_params_changed)

    # Param change in Rqt handle.
    def on_params_changed(self, params):
        param: rclpy.Parameter
        # Checking all parameter changes. 
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            # PD control param updates.
            if param.name == 'gains.kp' or param.name == 'gains.kd':
                response = self.string_list_handle(input_str=param.value)
                self.pd_change_handle(input_data=response,input_type=param.name[6:])

            # WAYPOINT Updates
            elif param.name == 'waypoint.seeking':
                response = self.string_list_handle(input_str=param.value)
                self.waypoint_handle(input_data=response)

            # Configuration Updates
            elif param.name == 'configuration.following_waypoint':
                self.object_master.configuration["Waypoint"] = param.value
                # Update the waypoint seeking to whatever is in RQT. 
                if param.value:
                    response = self.string_list_handle(self.get_parameter("waypoint.seeking").value)
                    self.waypoint_handle(input_data=response)

            # OTHER
            else:
                # To prevent program freezing. 
                return SetParametersResult(successful=False, reason='Parameter not found.')

            self.get_logger().info(f'{param.name} = {param.value} SET')

        return SetParametersResult(successful=True, reason='Parameter set')

    # For the dynamic reconfigure interface to work, strings must be used. Lists don't work. 
    # So need to convert the inputted strings to lists. 
    def string_list_handle(self, input_str):
        # Remove the front and back brackets. 
        output_list = input_str[1:-1]
        self.get_logger().debug(f'{output_list}')
        # Slice by commas.
        output_list = output_list.split(",")
        self.get_logger().debug(f'{output_list}')
        # Convert to list
        output_list = list(output_list)
        # Handle input errors.
        try:
            # Change the type of the contents 
            output_list = list(map(float,output_list))
            if len(output_list)>6:
                self.get_logger().warning(f'{output_list}. Too many inputs. Check input.')
                output_list = None
        except:
            self.get_logger().warning(f'{output_list}. Could not cast to list of floats. Check input.')
            output_list = None
            
        self.get_logger().debug(f'{output_list}')
        return output_list

    # To publish changes to master data handler if PID inputs are correct.
    def pd_change_handle(self,input_data,input_type):
        if input_data == None:
            self.get_logger().warning(f'PD controls NOT updated.')
        else:
            if input_type == "kp":
                self.object_master.kp=input_data
            elif input_type == "kd":
                self.object_master.kd=input_data     

    def waypoint_handle(self,input_data):
        # Do not update waypoints if input data is wrong.
        if input_data == None:
            self.get_logger().warning(f'Waypoints updated.')
        else:
            self.object_master.waypoint_on=input_data     

# PD Script was shared via Casadi.
# Found on: https://github.com/edxmorgan/Diff_UV
from casadi import Function
# For file pathing
from ament_index_python.packages import get_package_share_directory
class PDControl(Node):
    def __init__ (self,object_master):
        super().__init__(node_name='dory_pd_handle')

        # For master object variable access. 
        self.object_master = object_master
        # Finding the path to the casadi file for PD.
        sub_package_share_directory = get_package_share_directory('dory_nav2')
        pd_path = f'{sub_package_share_directory}/pid.casadi'
        self.solver_object = Function.load(pd_path)

    # Handles desired states. Returns the body fixed forces to reach said states. 
    def goal_handle(self,goal):
        # Does not like a combo of 
        # Current state: [x y z w xq yq zq u v w p q r]
        state_current = self.object_master.state_current
        kp=self.object_master.kp
        kd=self.object_master.kd
        #self.get_logger().info(f'kp {kp} kd {kd} xk: {xf}')
        response = self.solver_object(kp,kd,state_current,goal)
        return response

# Odom TF Handle
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
class OdomPublisher(Node):

    def __init__(self, object_master):
        super().__init__('dory_odom')
        # For master object variable access. 
        self.object_master = object_master

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer. 
        rate = 1/144
        self.object_master.rates["OdomPublisher"]["tf_odom"] = 1/rate
        self.tf_timer = self.create_timer(rate, self.tf_odom, callback_group=self.object_master.reentrant_group_2)


    def tf_odom(self):
        t = TransformStamped()
        # For time sync
        t.header.stamp = self.object_master.header_main.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # Invert the y and z axis vals. For PD controller handling.
        t.transform.translation.x= self.object_master.state_current[0]
        t.transform.translation.y = self.object_master.state_current[1]
        t.transform.translation.z= -self.object_master.state_current[2]

        # # Set new quaternion.
        t.transform.rotation.x = self.object_master.state_current[4]
        t.transform.rotation.y = self.object_master.state_current[5]
        t.transform.rotation.z = self.object_master.state_current[6]
        t.transform.rotation.w = self.object_master.state_current[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

# Visualization Handle
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
class WaypointVisuals(Node):
    def __init__(self,object_master):
        # For master object variable access. 
        self.object_master = object_master

        super().__init__('dory_visuals')

        # To prevent remaking marker many times since not updating. 
        self.waypoints = [
                     [0.0,0.0,0.0],
                     [50.0,0.0, 0.0],
                     [50.0,10.0,0.0],
                     [0.0,10.0, 0.0],
                     [0.0,20.0, 0.0],
                     [50.0,20.0,0.0],
                     [50.0,30.0,0.0],
                     [0.0,30.0,0.0],
                     [0.0,40.0,0.0],
                     [50.0,40.0,0.0],
                     [50.0,50.0,0.0],
                     [0.0,50.0,0.0],
                     [0.0,60.0,0.0],
                     [50.0,60.0,0.0],
                     [50.0,70.0,0.0],
                     [0.0,70.0,0.0],
                     ]
        # To make array once.
        self.init = False
        self.marker_array = MarkerArray()

        # Pub
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)

        # Timer 
        rate = self.object_master.rates["WaypointVisuals"]["timer_callback"]
        self.timer = self.create_timer(rate, self.timer_callback,callback_group=self.object_master.reentrant_group_2)

    def timer_callback(self):
        # Spawn Markers, forget about them.
        if not self.init:
            self.marker_init(array=self.waypoints)
            self.init=True

        # Publish
        self.marker_pub.publish(self.marker_array)
        return

    def marker_init(self,array):
        # Markers
        for i,pt in enumerate(array):
            # Make the markers so can publish it 
            marker = Marker()
            marker.header.frame_id="world"
            # Copy header stamp for time sync.
            marker.header.stamp=self.object_master.header_main.stamp

            # Make ns. Any marker with same namespace and ID will be overwritten. 
            marker.ns="waypoint_markers"

            # Set marker type. Could be Cube, Cylinder, etc. Circle.
            marker.type=int(2)

            # Set marker action. ADD: 0, Modify: 0, Delete: 2 (ns and id), Delete all objects: 3 (or in ns)
            marker.action = int(0)


            # Identifies individual marker
            marker.id=i
            # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = pt[0]/10
            marker.pose.position.y = pt[1]/10
            marker.pose.position.z = pt[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            # Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            self.marker_array.markers.append(marker)

        ## LINES
        for i in range(0,len(array)-1):

            # Make the markers so can publish it 
            marker = Marker()
            marker.header.frame_id="world"
            # Copy header stamp for time sync.
            marker.header.stamp=self.object_master.header_main.stamp

            # Make ns. Any marker with same namespace and ID will be overwritten. 
            marker.ns="waypoint_lines"

            # Set marker type. Could be Cube, Cylinder, etc. Line List.
            marker.type=int(4)

            # Set marker action. ADD: 0, Modify: 0, Delete: 2 (ns and id), Delete all objects: 3 (or in ns)
            marker.action = int(0)

            # Identifies individual marker
            marker.id=i

            # Specifically for lines
            point = Point()
            point.x=array[i][0]/10
            point.y=array[i][1]/10
            point.z=array[i][2]
            marker.points.append(point)

            point = Point()
            point.x=array[i+1][0]/10
            point.y=array[i+1][1]/10
            point.z=array[i+1][2]
            marker.points.append(point)

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.3

            # Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            self.marker_array.markers.append(marker)

        return
    
# Helper functions
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
def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


#######################################################

def main(args=None):
    rclpy.init(args=args)

    # Change thread count based on if visualizing or not. 
    visuals = True
    # The master object responsible for handling data sharing. 
    dory_master = MainObjectHandle(pub_state=visuals)

    # IMU data handling.
    dory_imu_handle = ImuHandle(object_master=dory_master)
    # Waypoint enabled vs testing controls
    dory_effort_handle = EffortHandle(object_master=dory_master)
    # For PD Control 
    dory_pd_handle = PDControl(object_master=dory_master)
    # Must be made after dory_pub_sub
    dory_waypoints_handle = WaypointFollower(object_master=dory_master,object_pd=dory_pd_handle)
    # For Dynamic reconfigure
    dory_reconfig_handle = DynamicReconfig(object_master=dory_master)
    # Odom TF Handling 
    dory_tf_handle = OdomPublisher(object_master=dory_master)
    # RVIZ2 Visuals Handling 
    dory_visuals_handle = WaypointVisuals(object_master=dory_master)

    # Multi-threading functionality.
    # num_threads based on available threads on machine and how many things 
    # need to run at once. 
    if visuals:
        thread_cnt = 4
    else:
        thread_cnt= 3

    # Main executor handle
    executor_main = MultiThreadedExecutor(num_threads=thread_cnt)    
    executor_2 = MultiThreadedExecutor(num_threads=3)    

    # Nodes
    executor_main.add_node(dory_master)
    executor_main.add_node(dory_effort_handle)
    executor_main.add_node(dory_pd_handle)
    executor_main.add_node(dory_waypoints_handle)
    executor_main.add_node(dory_reconfig_handle)

    executor_2.add_node(dory_imu_handle)
    executor_2.add_node(dory_tf_handle)
    executor_2.add_node(dory_visuals_handle)

    while True:
        try:
            # Handle subscriptions and odom separately because odom depends on subscription
            executor_2.spin_once()
            # Handle everything after.
            executor_main.spin_once()
            
        except KeyboardInterrupt:
            dory_reconfig_handle.destroy_node()
            dory_waypoints_handle.destroy_node()
            dory_pd_handle.destroy_node()
            dory_effort_handle.destroy_node()
            dory_imu_handle.destroy_node()

            rclpy.shutdown()
            break

if __name__ == '__main__':
    main()

# TODO lawn mower path gen
# TODO config file load waypoints for path 
# TODO autosave CSV. 
# TODO Convert to internal dynamics 