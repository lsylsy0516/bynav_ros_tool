#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.dirname(__file__))

import rospy
import tf2_ros
from pyproj import Transformer
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from novatel_oem7_msgs.msg import BESTPOS as Bestposa
from plot_map import get_route, get_Lat_Lon
from map_transform import GCJ02ToWGS84



topic_name = '/bynav/bestpos'
transformer = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
map_transformer = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
origin_x = origin_y = origin_z = None

# 初始化 TF 广播器和 Path 发布器
rospy.init_node('tf_publisher_node')
tf_broadcaster = tf2_ros.TransformBroadcaster()
path_publisher = rospy.Publisher('/robot_path', Path, queue_size=10)
map_path_publisher = rospy.Publisher('/map_path', Path, queue_size=10)


path_msg = Path()
path_msg.header.frame_id = "odom_gnss"
map_path_msg = Path()
map_path_msg.header.frame_id = "odom_gnss"

amap_key = '82ed988a4b5e5da017c5f4b038604822'
list_latlon=[]
Lon = []
Lat = []
# 仓库经纬度（格式：经度,纬度）
warehouse_location = '113.967847,22.592495'

# 农田经纬度（格式：经度,纬度）,换成自己的
farm_location = '113.98053,22.585877'
strategy = 32
routes = get_route(warehouse_location, farm_location, strategy, amap_key)

def callback(msg):
    global origin_x, origin_y, origin_z,Lat,Lon
    # 转换当前经纬度为 UTM 米坐标
    x, y = transformer.transform(msg.lon, msg.lat)
    z = msg.hgt
    
    # 如果没有记录起点，则设置为当前坐标
    if origin_x is None and origin_y is None and origin_z is None:
        origin_x, origin_y, origin_z = x, y, z 
        for lat,lon in zip(Lat,Lon):
            m_x, m_y = transformer.transform(lon, lat)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom_gnss"
            pose.pose.position.x = m_x - origin_x
            pose.pose.position.y = m_y - origin_y
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            map_path_msg.poses.append(pose)

    
    # 计算相对于起点的位移
    dx = x - origin_x
    dy = y - origin_y
    dz = z - origin_z

    print(f"dx: {dx}, dy: {dy}, dz: {dz}")
    # 创建 TransformStamped 消息
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom_gnss"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = dx
    t.transform.translation.y = dy
    t.transform.translation.z = dz
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    # 发布 TF 变换
    tf_broadcaster.sendTransform(t)

    # 创建 PoseStamped 消息并添加到 Path
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "odom_gnss"
    pose.pose.position.x = dx
    pose.pose.position.y = dy
    pose.pose.position.z = dz
    pose.pose.orientation = t.transform.rotation  # 沿用四元数

    path_msg.poses.append(pose)

    # 发布 Path 消息
    path_publisher.publish(path_msg)

    map_path_publisher.publish(map_path_msg)


if __name__ =="__main__":
    rospy.Subscriber(topic_name, Bestposa, callback)

    
    for route in routes:
        gjc_Lat, gjc_Lon = get_Lat_Lon(route['steps'])
        for gjc_lat,gjc_lon in zip(gjc_Lat,gjc_Lon):
            lon,lat = GCJ02ToWGS84(gjc_lon,gjc_lat)
            # print("before:",gjc_lat,gjc_lon)
            # print("after:",lat,lon)
            Lat.append(lat)
            Lon.append(lon)
        break
    while not rospy.is_shutdown():
        rospy.spin()