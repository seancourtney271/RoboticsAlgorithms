#!/usr/bin/env python2
import rospy
import tf2_ros
import sys
import tf
import tf_conversions
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random

def groundPos_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = 'robot_0/odom'
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)


def rotate(degree, direction, velocity, velocity_publisher):
    if direction == 0: #Rotate Right
        direction = -1
    PI = 3.1415926535897
    velocity.linear.x = 0
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = direction * 2.0

    # angular_speed = 2 * 2 * PI / 360
    # relative_angle = degree * 2 * PI / 360

    # velocity.angular.z = angular_speed * direction
    # t0 = rospy.Time.now().to_sec()
    # current_angle = 0

    # while(current_angle < relative_angle):
    #     velocity_publisher.publish(velocity)
    #     t1 = rospy.Time.now().to_sec()
    #     current_angle = angular_speed * (t1 - t0)

    # velocity.angular.z = 0
    # velocity_publisher.publish(velocity)
    velocity_publisher.publish(velocity)
    waitSec = rospy.Rate(float(degree * (PI / (180 * 2)))) #Wait in seconds

def callback(msg):
    maxDist = 1.25
    if msg.ranges[0] < maxDist: 
        #Possible ways to move are turn left between 45* and 190*
        degree = random.randint(45, 190)
        rotate(degree, 1, velocity, velocity_publisher)
    elif msg.ranges[len(msg.ranges) / 2] < maxDist: 
        #Possible ways to move are to turn left/right between 90 and 270
        degree = random.randint(90, 270)
        direction = random.randint(0, 1)
        rotate(degree, direction, velocity, velocity_publisher)
    elif msg.ranges[len(msg.ranges) - 1] < maxDist: 
        #Possible ways to move are to turn right between 45* and 190*
        degree = random.randint(45, 190)
        rotate(degree, 0, velocity, velocity_publisher)
    else:
        velocity.linear.x = 2.0 #Go Straight at 2m/s
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0
        velocity_publisher.publish(velocity)

if __name__ == '__main__':
    #handles movement of bot
    velocity_publisher = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('tf2_evader_broadcaster')
    velocity = Twist()
    velocity.linear.x = 2.0
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = 0.0

    rate = rospy.Rate(10) #10 Hz

    while not rospy.is_shutdown():
        baseScan_subscriber = rospy.Subscriber('/robot_0/base_scan', LaserScan, callback)    
        groundPos_subscriber = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, groundPos_callback)
        rate.sleep()