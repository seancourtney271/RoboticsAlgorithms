#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def groundPos_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = 'robot_1/odom'
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_pursuer_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    velocity_publisher2 = rospy.Publisher('/robot_1/cmd_vel', geometry_msgs.msg.Twist, queue_size = 10)
    groundPos_subscriber = rospy.Subscriber('/robot_1/base_pose_ground_truth', Odometry, groundPos_callback)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('robot_1/odom', 'robot_0/odom', rospy.Time(), rospy.Duration(3.0))
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        msg = geometry_msgs.msg.Twist()
        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        velocity_publisher2.publish(msg)

        rate.sleep()