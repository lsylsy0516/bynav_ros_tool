#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(__file__))
from ros_tf_bestposa import TfPublisherNode
# import ros_tf_bestposa
# print("ros_tf_bestposa",ros_tf_bestposa.__file__)
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
import math
class TransformSync:
    def __init__(self):
        rospy.init_node('tf_transform_sync')
        self.tf_node = TfPublisherNode()
        # 创建TF2 Buffer和Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅 /tf 话题
        self.tf_suber = rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        # 存储最近接收到的变换
        self.camera_to_body = None
        self.odom_to_base = None
        self.first_sync_received = True

    def tf_callback(self, msg):
        # 遍历 TFMessage 中的每个 TransformStamped
        for transform in msg.transforms:
            if transform.header.frame_id == 'camera_init' and transform.child_frame_id == 'body':
                self.camera_to_body = transform
            elif transform.header.frame_id == 'odom_gnss' and transform.child_frame_id == 'base_footprint':
                self.odom_to_base = transform

        # 当两个变换都收到时，计算关系
        # if self.camera_to_body and self.odom_to_base and not self.first_sync_received:
        if self.camera_to_body and self.odom_to_base and  self.first_sync_received:
            self.process_transforms()
            # rospy.logerr("process_transforms")
            # 结束程序
            # rospy.signal_shutdown("Transforms processed")

    def process_transforms(self):
        try:
            # 获取当前变换
            camera_to_body = self.tf_buffer.lookup_transform(
                'body', 'camera_init', rospy.Time(0), rospy.Duration(1.0)
            )
            odom_to_base = self.tf_buffer.lookup_transform(
                'base_footprint', 'odom_gnss', rospy.Time(0), rospy.Duration(1.0)
            )

            # rospy.loginfo("第一次同步变换计算完成:")
            # rospy.loginfo(f"camera_init 到 body: {camera_to_body}")
            # rospy.loginfo(f"odom_gnss 到 base_footprint: {odom_to_base}")

            self.first_sync_received = True

            # 计算 camera_init 到 odom_gnss 的变换
            self.publish_static_transform(camera_to_body, odom_to_base)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF查找失败: {e}")

            
    def publish_static_transform(self, camera_to_body, odom_to_base):
        # 创建 TransformStamped 对象
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'odom_gnss'
        t.child_frame_id = 'camera_init'

        # 计算平移
        # t.transform.translation.x = camera_to_body.transform.translation.x - odom_to_base.transform.translation.x
        # t.transform.translation.y = camera_to_body.transform.translation.y - odom_to_base.transform.translation.y
        # t.transform.translation.z = camera_to_body.transform.translation.z - odom_to_base.transform.translation.z
        # print("camera_to_body",camera_to_body)
        # print("odom_to_base",odom_to_base)
        yaw = self.tf_node.yaw
        t.transform.translation.x =  camera_to_body.transform.translation.x - odom_to_base.transform.translation.x
        t.transform.translation.y =  camera_to_body.transform.translation.y - odom_to_base.transform.translation.y
        temp_x = t.transform.translation.x
        temp_y = t.transform.translation.y
        t.transform.translation.x = temp_x * math.cos(yaw) - temp_y * math.sin(yaw)
        t.transform.translation.y = temp_x * math.sin(yaw) + temp_y * math.cos(yaw)
        t.transform.translation.z =  camera_to_body.transform.translation.z - odom_to_base.transform.translation.z

        # 计算相对旋转
        camera_quaternion = [
            camera_to_body.transform.rotation.x,
            camera_to_body.transform.rotation.y,
            camera_to_body.transform.rotation.z,
            camera_to_body.transform.rotation.w
        ]
        odom_quaternion = [
            odom_to_base.transform.rotation.x,
            odom_to_base.transform.rotation.y,
            odom_to_base.transform.rotation.z,
            odom_to_base.transform.rotation.w
        ]

        # 计算相对旋转，使得 camera_init 在 odom_gnss 中重合
        relative_quaternion = tft.quaternion_multiply(
            tft.quaternion_inverse(odom_quaternion), camera_quaternion
        )

        # relative_quaternion的yaw旋转角度要加上180度
        # 设置旋转
        t.transform.rotation.x = relative_quaternion[0]
        t.transform.rotation.y = relative_quaternion[1]
        t.transform.rotation.z = relative_quaternion[2]
        t.transform.rotation.w = relative_quaternion[3]

        # 广播静态变换
        tf2_ros.StaticTransformBroadcaster().sendTransform(t)
        # rospy.loginfo("已发布 camera_init -> odom_gnss 的静态变换")
        # rospy.loginfo(t)

if __name__ == '__main__':
    try:
        node = TransformSync()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
