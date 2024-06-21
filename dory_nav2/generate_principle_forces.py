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
        # For handling body fixed body based (F: xyz,rpy) input controls.
        self.input_ctrls = np.zeros(6)

        # PID parameters for waypoint following. Based on Dynamic Reconfigure Config. 
        #Can be reconfigured realtime.
        self.kp = np.zeros(6)
        self.kd = np.zeros(6)
        # Current state: [x y z wq xq yq zq u v w p q r]
        self.state_current = [0.0]*13
        # Current state simple: [xyz rpy]
        # To prevent waypoint list follower from skipping first index. 
        self.state_current_simple = [100.0]*6

        ## TO DO: CHANGE THE SIM BASED ON IF INPUT WAYPTS OR SAVED PATH FROM CONFIG OR 
        # Path generation function.
        # Waypoints
        self.waypoint = np.zeros(6)

        # for Plotjuggler plotting. Human readable data. 
        self.pub_state = pub_state
        if self.pub_state:
            ## Pubs for Roll, Pitch, Yaw.
            self._pub_state_rate = 0.002
            self._state_timer = self.create_timer(self._pub_state_rate, self.publish_state_message, callback_group=self.reentrant_group_1)

            ## Publishers
            self.state_visuals = State()
            self.state_header = Header() # To be copied from the IMU, because publishing based on that data. 
            self._publish_state = self.create_publisher(State, '/dory/visuals/state', 10)

    def publish_state_message(self):
        # Header same as IMU
        self.state_visuals.header = self.state_header
        # Position Data
        self.state_visuals.pose.position.x = self.state_current[0]
        self.state_visuals.pose.position.y = self.state_current[1]
        self.state_visuals.pose.position.z = self.state_current[2]
        # Euler Orientation
        self.state_visuals.pose.orientation.euler.roll = self.state_current_simple[3]
        self.state_visuals.pose.orientation.euler.pitch = self.state_current_simple[4]
        self.state_visuals.pose.orientation.euler.yaw = self.state_current_simple[5]
        # Quaternion Orientation
        self.state_visuals.pose.orientation.quaternion.w = self.state_current[3]
        self.state_visuals.pose.orientation.quaternion.x = self.state_current[4]
        self.state_visuals.pose.orientation.quaternion.x = self.state_current[5]
        self.state_visuals.pose.orientation.quaternion.x = self.state_current[6]
        # Velocity Linear
        self.state_visuals.twist.linear.x = self.state_current[7]
        self.state_visuals.twist.linear.y = self.state_current[8]
        self.state_visuals.twist.linear.z = self.state_current[9]
        # Velocity Angular
        self.state_visuals.twist.angular.x = self.state_current[10]   
        self.state_visuals.twist.angular.y = self.state_current[11]       
        self.state_visuals.twist.angular.z = self.state_current[12]       

        self._publish_state.publish(self.state_visuals)
        return

# To handle pose and velocity data
class ImuHandle(Node):
    def __init__(self,object_master):
        super().__init__('dory_imu_handle')
        # For master object access. 
        self.object_master = object_master

        self.sub_vel = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_states_callback,
            10,
            callback_group=object_master.reentrant_group_1
        )

    # Subscribers
    def dynamic_joint_states_callback(self, msg):
        # Pull the header if visual publications on. 
        if self.object_master.pub_state:
            self.object_master.state_header = msg.header

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

# Effort publish to thrusters and body force-> thruster mapping.
class EffortHandle(Node):
    def __init__(self,object_master):
        super().__init__('dory_effort_handle')
        # For master object access. 
        self.object_master = object_master

        ## Effort pub. Pubs for 8 thruster jts.
        self.effort_pub_rate = 0.002
        self.effort_timer = self.create_timer(self.effort_pub_rate, self.effort_publish_message, callback_group=object_master.reentrant_group_1)

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
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback,callback_group=object_master.reentrant_group_1)

        # To store robot updates
        err_deg = 15
        self.err_angle = err_deg *3.14/180 # quat
        self.err_dist = 0.1 # m

        # Waypoints
        # Set 1
        # self.waypoints = [
        #         [0.0,0.0,0.0,0.0,0.0,0.0],
        #         [5.0,0.0, 0.0,0.0,0.0,0.0],
        #         [5.0,1.0,0.0,0.0,0.0,0.0],
        #         [0.0,1.0,0.0,0.0,0.0,0.0],
        #         [0.0,2.0,0.0,0.0,0.0,0.0],
        #         [5.0,2.0,0.0,0.0,0.0,0.0],
        #         [5.0,3.0,0.0,0.0,0.0,0.0],
        #         [0.0,3.0,0.0,0.0,0.0,0.0],
        #         [0.0,4.0,0.0,0.0,0.0,0.0],
        #         [5.0,4.0,0.0,0.0,0.0,0.0],
        #         [5.0,5.0,0.0,0.0,0.0,0.0],
        #         [0.0,5.0,0.0,0.0,0.0,0.0],
        #         [0.0,6.0,0.0,0.0,0.0,0.0],
        #         [5.0,6.0,0.0,0.0,0.0,0.0],
        #         [5.0,7.0,0.0,0.0,0.0,0.0],
        #         [0.0,7.0,0.0,0.0,0.0,0.0],
        #         ]
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
        # xf is the goal waypoint 
        goal = self.object_master.waypoint
        # Load Goal
        # goal = self.waypoints[self.waypoint_index]
        # Input controls 
        self.object_master.input_ctrls= self.object_pd.goal_handle(goal=goal) 
        # Check if goal reached 
        # self.goal_check(goal=goal)  


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
            # Check if at end of waypoint list
            if self.waypoint_index==(len(self.waypoints)-1):
                self.get_logger().warning(f"\n AT END OF LIST. Current index: {self.waypoint_index}",once=True)
                return None

            # Reset. Update. 
            self.waypoint_reached[0:6] = 0
            self.waypoint_index+=1
            self.get_logger().info(f"\n WAYPOINT REACHED. Next Goal: {self.waypoints[self.waypoint_index]}")
            return True
        else:
            return False

    # Outputs a list for the lawnpath to follow. 
    # def lawnpath_gen(self,pass_cnt=3,pass_len_x = 1,pass_len_y=2,quad=0):
        
from rcl_interfaces.msg import SetParametersResult
class DynamicReconfig(Node):
    def __init__(self, object_master):
        super().__init__(node_name='dory_reconfig_handle')
        # To give selective access to all other objects info in script. 
        self.object_master = object_master
        self.init_params()

    # Dynamic Reconfigure init and config. 
    def init_params(self):
        # Must be declared for params to be accessible. 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains.kp', rclpy.Parameter.Type.STRING),
                ('gains.kd', rclpy.Parameter.Type.STRING),
                ('waypoint.pt1', rclpy.Parameter.Type.STRING)
            ],
        )
        ## PD Control
        # Gains kp
        param = self.get_parameter('gains.kp')
        self.get_logger().info(f'{param.name}={param.value}')
        response = self.string_list_handle(input_str=param.value)
        self.pd_change_handle(input_data=response,input_type="kp")
        # Gains kd
        param = self.get_parameter('gains.kd')
        self.get_logger().info(f'{param.name}={param.value}')
        response = self.string_list_handle(input_str=param.value)
        self.pd_change_handle(input_data=response,input_type="kd")

        # Waypoint Control 
        param = self.get_parameter('waypoint.pt1')
        self.get_logger().info(f'{param.name}={param.value}')
        response = self.string_list_handle(input_str=param.value)
        self.waypoint_handle(input_data=response)

        # To be called everytime ANYTHING is changed in RQT.
        self.add_on_set_parameters_callback(self.on_params_changed)

    # Param change in Rqt handle.
    def on_params_changed(self, params):
        param: rclpy.Parameter
        # Checking all parameter changes. 
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            # PD control param updates.
            if param.name == 'gains.kp':
                response = self.string_list_handle(input_str=param.value)
                self.pd_change_handle(input_data=response,input_type="kp")
            elif param.name == 'gains.kd':
                response = self.string_list_handle(input_str=param.value)
                self.pd_change_handle(input_data=response,input_type="kd")

            # WAYPOINT Updates
            elif param.name == 'waypoint.pt1':
                response = self.string_list_handle(input_str=param.value)
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
        if input_data == None:
            self.get_logger().warning(f'Waypoints updated.')
        else:
            self.object_master.waypoint=input_data     

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

import math
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


    # Multi-threading functionality.
    # num_threads based on available threads on machine and how many things 
    # need to run at once. 
    if visuals:
        thread_cnt = 5
    else:
        thread_cnt= 4
    executor = MultiThreadedExecutor(num_threads=thread_cnt)

    # Nodes
    executor.add_node(dory_master)
    executor.add_node(dory_imu_handle)
    executor.add_node(dory_effort_handle)
    executor.add_node(dory_pd_handle)
    executor.add_node(dory_waypoints_handle)
    executor.add_node(dory_reconfig_handle)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        dory_reconfig_handle.destroy_node()
        dory_waypoints_handle.destroy_node()
        dory_pd_handle.destroy_node()
        dory_effort_handle.destroy_node()
        dory_imu_handle.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()