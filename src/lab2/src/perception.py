#!/usr/bin/env python2
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import random
import math

laserScan = LaserScan()

def laserScan_callback(msg):
    global laserScan
    laserScan = msg
    RANSAC()

def RANSAC():
    #Creating a line marker in RVIS
    points, line, line_list = Marker(), Marker(), Marker()

    points.header.frame_id = line.header.frame_id = line_list.header.frame_id = "/my_frame"
    points.header.stamp = line.header.stamp = line_list.header.stamp = rospy.Time.now()

    points.ns = line.ns = "bot"
    points.action = line.action = line_list.action = Marker.ADD

    points.pose.orientation.w = line.pose.orientation.w = line_list.pose.orientation.w = 1.0

    points.id = 0
    line.id = 1
    line_list.id = 2

    points.type = Marker.POINTS
    line.type = Marker.LINE_STRIP
    line_list.type = Marker.LINE_LIST

    #Points get set
    points.scale.x = 0.2
    points.scale.y = 0.2

    #Line's scales get set
    line.scale.x = 0.1
    line_list.scale.x = 0.1

    # Points are green
    points.color.g = 1.0
    points.color.a = 1.0

    # Line strip is blue
    line.color.b = 1.0
    line.color.a = 1.0

    # Line list is red
    line_list.color.r = 1.0
    line_list.color.a = 1.0

    #Line position being set
    line.pose.position.x = 0.0
    line.pose.position.y = 0.0
    line.pose.position.z = 0.0

    #Line points array being created
    line.points = []

    for i in range(len(laserScan.ranges)):
        if(not laserScan.ranges[i] == 3.0):
            angle = laserScan.angle_min + (i * laserScan.angle_increment)
            p = Point()
            p.x = laserScan.ranges[i] * math.cos(angle)
            p.y = laserScan.ranges[i] * math.sin(angle)
            p.z = 0.0
            points.points.append(p)
        
    #RANSAC Algorthm
    n = 2  #Minumum number of data points required to estimate model parameters
    k = 75 #Max iterations
    t = 5 #Threshold value to determine if a data point fits well in a model 
    d = 40 #Number of data points required to assume a model fits well in the data
    p = Point()
    p.x = p.y = p.z = 0.0
    bestLine = ([p,p], 0)

    if(len(points.points) >= 2):
        iter = 0
        while iter < k: 
            inliers = 0
            pointPair = random.sample(points.points, 2)
            m = (pointPair[0].y - pointPair[1].y) / (pointPair[0].x - pointPair[1].x) #If I get shorter lines possibly dont treat all lines as infinite distance
            b = (m * pointPair[0].x) - pointPair[0].y
            for i in points.points:
                line_x = (i.y - b) / m 
                line_y = (m * i.x) - b
                dist = math.sqrt(((line_x - i.x)**2) + ((line_y - i.y)**2))
                if dist <= t:
                    inliers += 1
                    if (inliers > bestLine[1] and inliers >= d):
                        bestLine = (pointPair, inliers)
            iter = iter + 1
        line.points.append(bestLine[0][0])
        line.points.append(bestLine[0][1])

        # marker_pub.publish(points)
        marker_pub.publish(line)


if __name__ == '__main__':
    #handles movement of bot
    rospy.init_node("bot")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)

    baseScan_subscriber = rospy.Subscriber('/base_scan', LaserScan, laserScan_callback, queue_size=1)

    rospy.spin()