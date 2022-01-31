#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist

import math
import numpy as np
import heapq


centerOffset = [9, 9] #Y, X
    #  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7
map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0, #0
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0, #1
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, #2
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, #3
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, #4
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0, #5
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0, #6
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0, #7 
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1, #8
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1, #9
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1, #10
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0, #11
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0, #12
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0, #13
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, #14
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0, #15
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0, #16
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0, #17
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0, #18
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1] #19
#Center is around [9-10, 8-9] Y, X
#World start pos is [-8, -2] X, Y start map pos is [12, 1] Y, X

#World stuff
x = 0.0
y = 0.0
yaw = 0.0

class Node:
    nodeCount = 0
    def __init__(self, parent = None, action = None, cost = 0, point = [None, None]):
        self.__class__.nodeCount += 1
        self.nodeID = self.nodeCount
        self.parent = parent
        self.cost = cost #g
        self.action = action
        self.point = point
        self.f = 0

def child_node(n, action, point):
    return Node(n, action, n.cost + 1, point)

def mapToWorld(array): #This point is not true center it is bottom right of true center... may cause issues in the future
    #Flip values to be (X, Y), Negate Y value, Subtract center point offset
    return [array[1] - centerOffset[1], centerOffset[0] - array[0]]

def worldToMap(array): #This point is not true center it is bottom right of true center... may cause issues in the future
    #Flip values to be (Y, X), Negate Y value, Add center point offset
    return [(-array[1]) + centerOffset[0], array[0] + centerOffset[1]]

def validPoint(point):
    #If value is a double check 
    X = int(round(point[1]))
    Y = int(round(point[0]))
    uBnd = map.shape
    isWall = False

    if(X >= 0 and Y >= 0 and X < uBnd[1] and Y < uBnd[0]): #Not Out of bounds
        if(map[Y][X] == 0): #Not a wall
            return [Y, X]
        isWall = True
    if(isWall): #This check has an issue with whole numbers since 1.0 = 1.0
        case = 0
        if(X >= point[1]):              
            if(Y >= point[0]):          #X and Y were rounded up
                case = 4
            else:                       #X was rounded up and Y was not
                case = 2
        else:
            if(Y >= point[0]):          #X was not rounded up and Y was rounded up
                case = 3
            else:                       #X and Y were not rounded up
                case = 1
        X = int(point[1])
        Y = int(point[0])
        if(case == 1): #Top Left block is the closest check Bottom Left and Top right
            if(map[Y + 1][X] == 0): #Bottom Left
                #Take Bottom Left Value
                return [Y + 1, X]
                
            elif(map[Y][X + 1] == 0): #Top Right
                #Take Top Right Value
                return [Y, X + 1]

            elif(map[Y + 1][X + 1] == 0):#Bottom Right
                #Take Bottom Right Value
                return [Y + 1, X + 1]
            else:
                return None               
            
        elif(case == 2): #Top Right block is the closest check Top Left and Bottom Right
            if(map[Y][X] == 0): #Top Left
                #Take Top Left Value
                return [Y, X]
                
            elif(map[Y + 1][X + 1] == 0): #Bottom Right
                #Take Bottom Right Value
                return [Y + 1, X + 1]

            elif(map[Y + 1][X] == 0):#Bottom Left
                #Take Bottom Left Value
                return [Y + 1, X]
                
            else:
                return None

        elif(case == 3): #Bottom Left block is the closest check Top Left and Bottom Right
            if(map[Y][X] == 0): #Top Left
                #Take Top Left Value
                return [Y, X]
                
            elif(map[Y + 1][X + 1] == 0): #Bottom Right
                #Take Bottom Right Value
                return [Y + 1, X + 1]

            elif(map[Y + 1][X] == 0):#Top Right
                #Take Top Right Value
                return [Y + 1, X]
                
            else:
                return None

        elif(case == 4): #Bottom Right block is the closest check Bottom Left and Top Right
            if(map[Y + 1][X] == 0): #Bottom Left
                #Take Bottom Left Value
                return [Y + 1, X]
                
            elif(map[Y][X + 1] == 0): #Top Right
                #Take Top Right Value
                return [Y, X + 1]

            elif(map[Y][X] == 0):#Top Left
                #Take Top Left Value
                return [Y, X]
                
            else:
                return None
    return None

def validActionList(state_M): #Point in (Y, X)
    #8 Possible states
    #Check if points are in map bounds or end up being a wall
    #If diagonal check if instruction that leads to it are open EX Up Right, Check if Up and Right are open
    #If they are not then the diagonal intrstruction is not valid
    actionList = []
    points = []
    X = state_M[1]
    Y = state_M[0]
    uBnd = map.shape

    #Up             (-1, 0)     
    if(Y - 1 >= 0 and Y - 1 < uBnd[0]): #Not Out of bounds
        if(map[Y - 1][X] == 0): #Not a wall
            actionList.append("U")
            points.append([Y - 1, X])

    #Left           (0, -1)
    if(X - 1 >= 0 and X - 1 < uBnd[1]): #Not Out of bounds
        if(map[Y][X - 1] == 0): #Not a wall
            actionList.append("L")
            points.append([Y, X - 1])

    #Right          (0, 1)
    if(X + 1 >= 0 and X + 1 < uBnd[1]): #Not Out of bounds
        if(map[Y][X + 1] == 0): #Not a wall
            actionList.append("R")
            points.append([Y, X + 1])

    #Down           (1, 0)
    if(Y + 1 >= 0 and Y + 1 < uBnd[0]): #Not Out of bounds
        if(map[Y + 1][X] == 0): #Not a wall
            actionList.append("D")
            points.append([Y + 1, X])

    #Up Left        (-1, -1)
    if("U" in actionList and "L" in actionList):
        if(X - 1 >= 0 and Y - 1 >= 0 and X - 1 < uBnd[1] and Y - 1 < uBnd[0]): #Not Out of bounds
            if(map[Y - 1][X - 1] == 0): #Not a wall
                actionList.append("UL")
                points.append([Y - 1, X - 1])

    #Up Right       (-1, 1)
    if("U" in actionList and "R" in actionList):
        if(X + 1 >= 0 and Y - 1 >= 0 and X + 1 < uBnd[1] and Y - 1 < uBnd[0]): #Not Out of bounds
            if(map[Y - 1][X + 1] == 0): #Not a wall
                actionList.append("UR")
                points.append([Y - 1, X + 1])
                
    #Down Left      (1, -1)
    if("D" in actionList and "L" in actionList):
        if(X - 1 >= 0 and Y + 1 >= 0 and X - 1 < uBnd[1] and Y + 1 < uBnd[0]): #Not Out of bounds
            if(map[Y + 1][X - 1] == 0): #Not a wall
                actionList.append("DL")
                points.append([Y + 1, X - 1])
                
    #Down Right     (1, 1)
    if("D" in actionList and "R" in actionList):
        if(X + 1 >= 0 and Y + 1 >= 0 and X + 1 < uBnd[1] and Y + 1 < uBnd[0]): #Not Out of bounds
            if(map[Y + 1][X + 1] == 0): #Not a wall
                actionList.append("DR")
                points.append([Y + 1, X + 1])
    return actionList, points

def heuristic(currentPoint, goalPoint): #h
    return (goalPoint[1] - currentPoint[1])**2 + (goalPoint[0] - currentPoint[0])**2

def wallCheck(point):
    proximity = 0
    for i in range(-1, 2):
        for j in range(-1, 2):
            if(map[i][j] == 1):
                proximity += 1
    return proximity

def solution(node):
    data = []
    while node.parent != None:
        data.insert(0, (node.point, node.action))
        node = node.parent
    return data

def a_star_graph(start_M, goal_M):
    openList = []
    closedList = []
    initialNode = Node(None, None, 0, start_M)
    goalNode = Node(None, None, 0, goal_M)
    f = heuristic(initialNode.point, goalNode.point) + initialNode.cost


    heapq.heappush(openList, (f, initialNode))
    heapq.heapify(openList)

    while(openList):
        tup = heapq.heappop(openList)
        fValue = tup[0]
        node = tup[1]

        if(node.point == goalNode.point):
            return solution(node)
        
        closedList.append((fValue, node))

        actionList, points = validActionList(node.point)
        for n in range(len(actionList)):
            child = child_node(node, actionList[n], points[n])
            foundIO = False
            foundIC = False

            for i in range(len(closedList)):
                if (child.point == closedList[i][1].point):
                    foundIC = True
                    break
            if(foundIC):
                continue
            childF = heuristic(child.point, goalNode.point) + child.cost + wallCheck(child.point)

            for i in range(len(openList)):
                if(child.point == openList[i][1].point):
                    foundIO = True
                    if(childF < openList[i][0]):
                        openList[i] = (childF, child)
                    break
            if(foundIO):
                continue
            if(not foundIC and not foundIO):
                heapq.heappush(openList, (childF, child))
    return None

def odom_Msg(msg):
    global x
    global y
    global yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rotation = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])

def path_Follow(data, goalF):
    vel = Twist()
    for n in data:
        point_W = mapToWorld(n[0])
        while heuristic([x, y], point_W) > .5:
            inc_x = point_W[0] - x
            inc_y = point_W[1] - y

            angle_to_goal = math.atan2(inc_y, inc_x)

            if angle_to_goal - yaw > 0.1:
                vel.linear.x = 0.0
                vel.angular.z = 0.3
            elif angle_to_goal - yaw < -0.1:
                vel.linear.x = 0.0
                vel.angular.z = -0.3
            else:
                vel.linear.x = 0.5
                vel.angular.z = 0.0

            velocity_pub.publish(vel)

def printPath(path):
    map_s = np.chararray(map.shape)
    for y in range(len(map)):
        for x in range(len(map[y])):
            if(map[y][x] == 0):
                map_s[y][x] = '_'
            else:
                map_s[y][x] = '1'
    for n in data:
        y = n[0][0]
        x = n[0][1]
        map_s[y][x] = '*'
    for y in range(len(map)):
        line_S = ""
        for x in range(len(map[y])):
            if(map_s[y][x] == '_'):
                line_S = line_S + ' '
            else:
                line_S = line_S + map_s[y][x]
        print(line_S)


if __name__ == "__main__":
    goalParam = rospy.get_param("/goal")
    goal = [goalParam["goalx"], goalParam["goaly"]]
    start = [-8, -2]
    actionList = []

    rospy.init_node("a_star_bot")
    velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, odom_Msg)

    map = np.asarray(map).reshape(20, 18)
    start_M = worldToMap(start)
    goal_M = worldToMap(goal)
    validStart = validPoint(start_M)
    validGoal = validPoint(goal_M)
    if(validStart == None or validGoal == None):
        print("Invalid Points Given")
    else:
        data = a_star_graph(validStart, validGoal)
        printPath(data)
        data.append((goal_M, "End"))
        path_Follow(data, goal)
        rospy.spin()