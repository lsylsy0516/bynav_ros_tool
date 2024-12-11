#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import tf
from tf2_msgs.msg import TFMessage

class Dis_calc:
    def __init__(self):
        rospy.init_node("dis_calc")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_suber = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.lio_dis = 0
        self.gnss_dis = 0
        self.camera_to_body = None
        self.odom_to_base = None
        rospy.loginfo("Dis_calc node initialized")
    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'camera_init' and transform.child_frame_id == 'body':
                self.camera_to_body = transform
            elif transform.header.frame_id == 'odom_gnss' and transform.child_frame_id == 'base_footprint':
                self.odom_to_base = transform
        if self.camera_to_body and self.odom_to_base:
            self.process_transforms()
    
    def process_transforms(self):
        try:
            camera_to_body = self.tfBuffer.lookup_transform(
                'body', 'camera_init', rospy.Time(), rospy.Duration(1.0)
            )
            odom_to_base = self.tfBuffer.lookup_transform(
                'base_footprint', 'odom_gnss', rospy.Time(), rospy.Duration(1.0)
            )
            lio_dis = math.sqrt((camera_to_body.transform.translation.x)**2 + (camera_to_body.transform.translation.y)**2)
            gnss_dis = math.sqrt((odom_to_base.transform.translation.x)**2 + (odom_to_base.transform.translation.y)**2)
            # if lio_dis > self.lio_dis:
            self.lio_dis = lio_dis
            # if gnss_dis > self.gnss_dis:
            self.gnss_dis = gnss_dis
            rospy.loginfo(f"lio_dis: {self.lio_dis}")
            rospy.loginfo(f"gnss_dis: {self.gnss_dis}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")

if __name__ == "__main__":
    dis_calc = Dis_calc()
    rospy.spin()