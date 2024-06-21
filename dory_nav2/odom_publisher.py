import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

# The msg type of the incoming data. 
from control_msgs.msg import DynamicJointState

# For multi-threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class FramePublisher(Node):

    def __init__(self,reentrant_group):
        super().__init__('dory_tf2_frame_publisher')
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.subscription = self.create_subscription(
            DynamicJointState,
            'dynamic_joint_states',
            self.handle_odom,
            10,
            callback_group=reentrant_group
            )
        self.subscription  # prevent unused variable warning

        # For time synch across nodes. 
        self.header_stamp = None

    def handle_odom(self, msg):
        t = TransformStamped()

        # For sharing headers for time sync
        self.header_stamp = msg.header.stamp
        # For time sync
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # Invert the y and z axis vals. For PD controller handling.
        t.transform.translation.x= msg.interface_values[8].values[9]
        t.transform.translation.y = -msg.interface_values[8].values[2]
        t.transform.translation.z= -msg.interface_values[8].values[0]

        # Invert the pitch and yaw vals. For PD Controller handling.
        roll,pitch,yaw = euler_from_quaternion(msg.interface_values[8].values[10],msg.interface_values[8].values[6],msg.interface_values[8].values[3],msg.interface_values[8].values[1])
        pitch = -pitch
        yaw = -yaw
        quat = quaternion_from_euler(roll,pitch,yaw)
        
        # Set new quaternion.
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

# Visualization Class
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class WaypointVisuals(Node):
    def __init__(self,reentrant_group):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('waypoint_visual_handle')

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
        self.timer = self.create_timer(5.0, self.timer_callback,callback_group=reentrant_group)

    def timer_callback(self):
        # Spawn Markers, forget about them.
        if not self.init:
            self.marker_init(array=self.waypoints)
            self.init=True

        # Publish
        self.marker_pub.publish(self.marker_array)
        return

    def marker_init(self,array):
        for i,pt in enumerate(array):
            # Make the markers so can publish it 
            marker = Marker()
            marker.header.frame_id="world"
            # Copy header stamp for time sync.
            marker.header.stamp=self.get_clock().now().to_msg()

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
            marker.header.stamp=self.get_clock().now().to_msg()

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


def main(args=None):
    rclpy.init(args=args)
    # Reentrant Groups for Multithreading
    reentrant_group_1 = ReentrantCallbackGroup()

    # Odom Handle
    handle_odom = FramePublisher(reentrant_group=reentrant_group_1)
    # Visuals Handle
    handle_visuals = WaypointVisuals(reentrant_group=reentrant_group_1)

    # Multi-threading functionality.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(handle_odom)
    executor.add_node(handle_visuals)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        handle_odom.get_logger().info('handle_odom Node has been shut down.')
        handle_visuals.get_logger().info('handle_visuals Node has been shut down.')

        handle_odom.destroy_node()
        handle_visuals.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()