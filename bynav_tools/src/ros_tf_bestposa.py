#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.dirname(__file__))
import rospy
import tf2_ros
import tf
from pyproj import Transformer
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from novatel_oem7_msgs.msg import BESTPOS as Bestposa
from novatel_oem7_msgs.msg import INSPVAX as Inspvax
from plot_map import get_route, get_Lat_Lon
from map_transform import GCJ02ToWGS84

class TfPublisherNode:
    def __init__(self):
        # Setup ROS node parameters
        rospy.init_node('tf_publisher_node')

        # Constants
        self.ATTITUDE_TOPIC = '/bynav/inspvax'    
        self.POS_TOPIC = '/bynav/bestpos'    
        self.ROBO_PATH_TOPIC = '/robot_path'
        self.MAP_PATH_TOPIC = '/map_path'
        self.EPSG_4326 = "EPSG:4326"
        self.EPSG_32633 = "EPSG:32633"

        # Initialize variables
        self.origin_x = self.origin_y = self.origin_z = None
        self.Lat = []
        self.Lon = []
        self.path_msg = Path()
        self.map_path_msg = Path()
        self.path_msg.header.frame_id = self.map_path_msg.header.frame_id = "odom_gnss"

        # Initialize publishers and transformers
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.path_publisher = rospy.Publisher(self.ROBO_PATH_TOPIC, Path, queue_size=10)
        self.map_path_publisher = rospy.Publisher(self.MAP_PATH_TOPIC, Path, queue_size=10)
        self.transformer = Transformer.from_crs(self.EPSG_4326, self.EPSG_32633, always_xy=True)

        # Define routes using your specific keys
        AMAP_KEY = '82ed988a4b5e5da017c5f4b038604822'
        START = '113.967847,22.592495'
        END = '113.98053,22.585877'
        self.routes = get_route(START, END, mode=32, amap_key=AMAP_KEY)

        # Setup subscribers
        rospy.Subscriber(self.POS_TOPIC, Bestposa, self.position_callback)
        rospy.Subscriber(self.ATTITUDE_TOPIC, Inspvax, self.attitude_callback)

        # Extract routes and transform coordinates
        self.extract_routes()

    def extract_routes(self):
        for route in self.routes:
            gjc_Lat, gjc_Lon = get_Lat_Lon(route['steps'])
            for gjc_lat, gjc_lon in zip(gjc_Lat, gjc_Lon):
                lon, lat = GCJ02ToWGS84(gjc_lon, gjc_lat)
                self.Lat.append(lat)
                self.Lon.append(lon)
            break

    def initialize_origin(self, x, y, z):
        """Initialize the origin coordinates."""
        self.origin_x, self.origin_y, self.origin_z = x, y, z

    def transform_coordinates(self, lon, lat):
        """Transform longitude and latitude to UTM coordinates."""
        return self.transformer.transform(lon, lat)

    def create_pose_stamped(self, dx, dy, dz, x, y, z, w):
        """Create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom_gnss"
        pose.pose.position.x = dx
        pose.pose.position.y = dy
        pose.pose.position.z = dz
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def position_callback(self, pos_msg):
        """Callback for processing incoming position data."""
        self.current_pos_msg = pos_msg  # Store the current position message
        # print("Received position data:", pos_msg)

        # If we have the latest attitude data, process the data
        if hasattr(self, 'current_att_msg'):
            self.process_data()

    def attitude_callback(self, att_msg):
        """Callback for processing incoming attitude data."""
        self.current_att_msg = att_msg  # Store the current attitude message
        # print("Received attitude data:", att_msg)

        # If we have the latest position data, process the data
        if hasattr(self, 'current_pos_msg'):
            self.process_data()

    def process_data(self):
        """Process both position and attitude data."""
        pos_msg = self.current_pos_msg
        att_msg = self.current_att_msg
        
        # Transform current latitude and longitude to UTM coordinates
        x, y = self.transform_coordinates(pos_msg.lon, pos_msg.lat)
        z = pos_msg.hgt
        roll, pitch, yaw = att_msg.roll, att_msg.pitch, att_msg.azimuth
        # yaw = yaw -150

        # Transfer from degree to radian
        roll, pitch, yaw = roll / 180.0 * 3.1415926, pitch / 180.0 * 3.1415926, yaw / 180.0 * 3.1415926

        # Initialize origin if not already done
        if self.origin_x is None and self.origin_y is None and self.origin_z is None:
            self.initialize_origin(x, y, z)
            for lat, lon in zip(self.Lat, self.Lon):
                m_x, m_y = self.transform_coordinates(lon, lat)
                self.map_path_msg.poses.append(self.create_pose_stamped(m_x - self.origin_x, m_y - self.origin_y, 0, 0, 0, 0, 1))

        # Calculate displacement from origin
        dx, dy, dz = x - self.origin_x, y - self.origin_y, z - self.origin_z
        # Create and publish TransformStamped message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = "odom_gnss"
        transform_msg.child_frame_id = "base_footprint"
        transform_msg.transform.translation.x = dx
        transform_msg.transform.translation.y = dy
        transform_msg.transform.translation.z = dz
        x, y, z, w = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        transform_msg.transform.rotation.x = x
        transform_msg.transform.rotation.y = y
        transform_msg.transform.rotation.z = z
        transform_msg.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(transform_msg)

        # Create and publish PoseStamped message for path
        self.path_msg.poses.append(self.create_pose_stamped(dx, dy, dz, x, y, z, w))
        self.path_publisher.publish(self.path_msg)
        self.map_path_publisher.publish(self.map_path_msg)

if __name__ == "__main__":
    node = TfPublisherNode()
    rospy.spin()
