#!/usr/bin/env python

import heapq
import rospy
import roslib
import time
import math
import Queue
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
import tf
import numpy


#####============================== AStar and Path ==============================#####

def AStar(start, goal):
    global done 
    CurrGrid = SquareGrid()
    frontier = Queue.PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[0] = start
    cost_so_far[start] = 0
    done = 0 

    while not frontier.empty():
        current = frontier.get()
        
        if done == 1:
            break
        results = CurrGrid.neighbors(current)

        for next in CurrGrid.neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far and heuristic(goal, next) < heuristic(goal, current):
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                #print next
                came_from[new_cost] = current
            if (heuristic(goal, next) < 1):
                done = 1 
                break

    cost = new_cost + 1
    came_from[cost] = next
    #newPathCallback(came_from)
    print "done"
    #print came_from
    return came_from


#still needs to be altered****************
def AStar2(start, goal):
    global done 
    CurrGrid = SquareGrid()
    frontier = Queue.PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[0] = start
    cost_so_far[start] = 0
    done = 0 

    while not frontier.empty():
        current = frontier.get()
        
        if done == 1:
            break
        results = CurrGrid.neighbors(current)

        for next in CurrGrid.neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far and heuristic(goal, next) < heuristic(goal, current):
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                #print next
                came_from[new_cost] = current
            if (heuristic(goal, next) < 1):
                done = 1 
                break

    cost = new_cost + 1
    came_from[cost] = next
    #newPathCallback(came_from)
    print "done"
    #print came_from
    return came_from

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b 
    return abs(x1 - x2) + abs(y1 - y2)


def displayPath(path, start, goal):
    pathPoints = []
    pathCells = []
    length = len(path)
    i = 0

    while(i < length):
        pathPoints.append(path.values()[i])
        pathCells.append(pathPoints[i])
        i += 1

    print "printing path"
    waypoints = []
    j = 1
    length = len(path)

    waypoints.append(start)
    straightCounter = 0

    while(j + 1 < length):
        if(pathPoints[j][1] == pathPoints[j-1][1] and pathPoints[j][0] == pathPoints[j+1][0]):
            waypoints.append(pathPoints[j])
            straightCounter = 0

        elif(pathPoints[j][0] == pathPoints[j-1][0] and pathPoints[j][1] == pathPoints[j+1][1]):
            waypoints.append(pathPoints[j])
            straightCounter = 0
        else:
            straightCounter += 1

        if(straightCounter == 5):
            waypoints.append(pathPoints[j])
            straightCounter = 0

        j += 1

    waypoints.append(goal)

    #publish path
    for k in range(0, len(pathCells)):
        tmpX = int(pathCells[k][0])
        tmpY = int(pathCells[k][1])

        p_cells.addPoint(tmpX, tmpY, res)

    #publish waypoints
    for k in range(0, len(waypoints)):
        tmpX = int(waypoints[k][0])
        tmpY = int(waypoints[k][1])

        w_cells.addPoint(tmpX,tmpY,res)

    path_pub.publish(p_cells.a)
    pathPub.publish(w_cells.a)
    frontier_pub.publish(f_cells.a)


    print "done"
    return waypoints


#####============================== Grid ==============================#####

class SquareGrid: 
    def __init__(self):
        self = mapgrid

    def in_bounds(self, point):
        currentX = point[0]
        currentY = point[1]
        return 0 <= currentX < width and 0<= currentY < height

    def passable(self, point):  

        tmpX = int(point[0])
        tmpY = int(point[1])

        index = (37*tmpY) + tmpX

        return 100 != mapData[index]


        #return point not in b_cells.a.cells

    def neighbors(self, point):
        global flag

        (x, y) = point 

        right_cell = (x+1, y)
        bottom_cell = (x, y-1)
        left_cell = (x-1, y)
        top_cell = (x, y+1)
        #add neighbor cells to frontier list
        f_cells.addPoint(x+1,y, res)
        f_cells.addPoint(x, y-1, res)
        f_cells.addPoint(x-1,y, res)
        f_cells.addPoint(x, y+1, res)
        #neighbor cells are filtered to make sure they are not an obstacle and are on the map 
        results = [right_cell, bottom_cell, left_cell, top_cell]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)

        return results


class newCell:
    def __init__(self, size):
        self.a = GridCells()    
        self.a.header.frame_id = 'map'
        self.a.cell_width = size
        self.a.cell_height = size
        self.a.cells = []           
        
    def get_Size(self):
        return self.a.cell_width

    def addPoint(self, x, y,res):
        point = Point()
        point.x = (x*res) + .65
        point.y = (y*res) + .15
        point.z=0
        self.a.cells.append(point)


def displayGrid(grid):
    global frontier_pub
    global unknown_pub
    global boundary_pub
    global explored_pub
    global e_cells
    global b_cells
    global f_cells
    global u_cells
    global w_cells
    global p_cells

    e_cells = newCell(res)
    b_cells = newCell(res)
    f_cells = newCell(res)
    u_cells = newCell(res)
    w_cells = newCell(res)
    p_cells = newCell(res)
        
    cell = 0    
    for y in range(0,height):
        for x in range (0,width):        
            if (mapData[cell] == 100):
                b_cells.addPoint(x,y,res)
            #Explored is 0-99
            else:
                 if (0.0 <= mapData[cell] <= 99):
                    e_cells.addPoint(x,y,res)
            cell += 1
    #Results are published
    explored_pub.publish(e_cells.a) 
    boundary_pub.publish(b_cells.a)
    unknown_pub.publish(u_cells.a)  


#####============================== Movement ==============================#####
 
def driveStraight(speed, distance):
    global odom_list
    global pose
    global twistPub

    # Set current location to (x0, y0) - Origin
    x0 = pose.pose.position.x / .3
    y0 = pose.pose.position.y / .3

    moving = Twist()
    stop = Twist()
    moving.linear.x = speed
    stop.linear.x = 0



    done = False
    while (not done and not rospy.is_shutdown()):
        # Set current location to (x1, y1)
        x1 = pose.pose.position.x / .3
        y1 = pose.pose.position.y / .3

        # Distance between reference frame and current frame
        d = math.sqrt(math.pow((x1 - x0), 2) + math.pow((y1 - y0), 2))
        
        # Checks to see if the required distance has been attained
        if (abs(distance - d) < 0.1):
            # Change flag from False to True and stop the motors
            done = True
        
        twistPub.publish(moving)
        rospy.sleep(0.1)

    # After driving the desired amount, stop
    twistPub.publish(stop)
    rospy.sleep(0.1)


def rotate(angle):
    if(angle > numpy.pi):
        angle -= (numpy.pi * 2)
    elif(angle < -1 * numpy.pi):
        angle += (numpy.pi * 2)

    global odom_list
    global pose

    CW = Twist()
    CCW = Twist()
    stop = Twist()
    CW.angular.z = -.3
    CCW.angular.z = .3
    stop.angular.z = 0

    # Extends the base class tf.Transform, adding methods for handling ROS messages
    transformer = tf.TransformerROS()

    # Create goal rotation matrix from the desired change in the angle
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],
                            [math.sin(angle),  math.cos(angle), 0],
                            [0,                0,               1]])

    # Waits for transformation from "base_footprint" to "odom" 
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))

    # Returns the current frame of the robot
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    
    # Converts the current frame of the robot into a 4x4 transformation matrix
    T_o_t = transformer.fromTranslationRotation(trans, rot)

    # Retrieves the rotation matrix transform from the 4x4 transformation matrix of the current position
    R_o_t = T_o_t[0:3, 0:3]

    # Setup Goal matrix
    # Determine rotation matrix to achieve correct rotation
    goal_rot = numpy.dot(rotation, R_o_t)

    # Set up 4x4 transformation matrix of the goal frame of the correct rotation
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                          [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                          [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                          [0,             0,             0,             1]])

    # Loop until the angle between the attached frame and the origin is equal to the angle specified
    done = False
    while (not done and not rospy.is_shutdown()):
        # Returns the transform from "base_footprint" to "odom"
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))

        # Sets the current location to state
        state = transformer.fromTranslationRotation(trans, rot)
 
        # Creates a 4x4 matrix to check to see how close every value is to the value int the desired frame
        within_tolerance = abs((state - goal_o)) < 0.15
     
        # Checks to see if the current frame is close enough to the desired
        if ( within_tolerance.all() ):
            # If it is, stop the wheels
            twistPub.publish(stop)
            rospy.sleep(0.1)
            # Set flag to be True
            done = True
        else:
            # Checks to see if the desired turn was clockwise or counter clockwise

            if (angle > 0):
                # If turn was counter clockwise
                twistPub.publish(CCW)
                rospy.sleep(0.1)
            else:
                # If turn was clcokwise
                twistPub.publish(CW)
                rospy.sleep(0.1)


def goToWaypoint(goal):
    global pose

    x0 = robotX
    y0 = robotY
    theta0 = robotTheta

    #desired orientation
    quat1 = pose.pose.orientation
    q1 = [quat1.x, quat1.y, quat1.z, quat1.w]
    roll1, pitch1, yaw1 = tf.transformations.euler_from_quaternion(q1)
    x1 = goal[0]
    y1 = goal[1]
    theta1 = yaw1

    #rotation
    print "rotating"
    theta2 = math.atan2(y1 - y0, x1 - x0) - theta0

    rotate(theta2)

    #move forward
    print "driving"
    dist = (((x1 - x0) ** 2) + ((y1 - y0) ** 2)) ** 0.5

    print dist

    driveStraight(.2, dist)

    print "goToWaypoint done"


def driveToGoal(waypoints, start, goal):
    length = len(waypoints)

    for i in range(1, length - 1):
        print robotX
        print robotY
        print robotTheta
        print "---"

        goToWaypoint(waypoints[i])

    goToWaypoint(goal)

    theta = thetaEnd - robotTheta 

    rotate(theta) #to final pose


#####============================== Callbacks ==============================#####

def mapCallBack(data):
    global mapData
    global grid
    global width
    global height
    global mapgrid
    global res
    global origin
    origin = data.info.origin.position
    res = data.info.resolution
    mapgrid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height


def readGoal(msg):
    px = (msg.pose.position.x - .5) / .3
    py = msg.pose.position.y / .3
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xEnd
    global yEnd
    global thetaEnd
    goalPos = (px, py)
    xEnd = px
    yEnd = py
    thetaEnd = yaw * 180.0 / math.pi
    print "Goal obtained"
    print goalPos
    print "---"
    PathToGoal = AStar(startPos, goalPos)
    Waypoints = displayPath(PathToGoal, startPos, goalPos)

    #####possible Astar2 implementation#####
    for i in range(len(Waypoints)):
        Waypoints2 = Astar2(Waypoints[i], Waypoints[i+1])
        driveToGoal(Waypoints2, Waypoints[i], Waypoints[i+1])
    print "Lab complete"

    
def startCallBack(data):
    px = (data.pose.pose.position.x - .5) / .3
    py = data.pose.pose.position.y / .3
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    global startPos
    xInit = px
    yInit = py
    startPos = (px, py)
    thetaInit = yaw * 180.0 / math.pi
    print "Start obtained"
    print startPos


def odomCallback(data):
    global odom_tf
    global pose

    # Set pose to to be pose of the robot
    pose = data.pose
    # Set geo_quat to be the orientation of the pose of the robot
    geo_quat = pose.pose.orientation

    # Broadcast the transformation from the tf frame "base_footprint" to tf frame "odom" 
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), 
                          (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), 
                          rospy.Time.now(), 
                          "base_footprint", 
                          "odom")
    

def localCostCallBack(data):
    global localMapData
    global localHeight
    global localWidth
    global localRes

    localMapData = data.data
    localHeight = data.info.height
    localWidth = data.info.width
    localRes = data.info.resolution


def robotLocationCallBack(data):
    global robotX
    global robotY
    global robotTheta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
    robotX = (position[0] - .5) / .3
    robotY = position[1] / .3
    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    robotTheta = yaw


#####============================== MAIN ==============================#####

if __name__ == '__main__':

    global target
    global gridPub
    global pathPub
    global startPos
    global frontier_pub
    global unknown_pub
    global boundary_pub
    global explored_pub
    global path_pub
    global twistPub
    global pose
    global odom_tf
    global odom_list

    global robotX
    global robotY
    global robotTheta

    AMap = 0
    path = 0

    rospy.init_node('lab3')

    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
    odomSub = rospy.Subscriber("/odom", Odometry, odomCallback)
    markerSub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, readGoal)
    sub = rospy.Subscriber("/start", PoseWithCovarianceStamped, startCallBack)
    localCostSub = rospy.Subscriber("/move_base/local_stmap/costmap", OccupancyGrid, localCostCallBack)

    pathPub = rospy.Publisher('/waypoints', GridCells, queue_size=1)
    gridPub = rospy.Publisher("/boundary", GridCells, queue_size=1)

    frontier_pub = rospy.Publisher("/frontier", GridCells, queue_size=1)            
    boundary_pub = rospy.Publisher("/boundary", GridCells, queue_size=1)
    explored_pub = rospy.Publisher("/explored", GridCells, queue_size=1)
    unknown_pub = rospy.Publisher("/unknown", GridCells, queue_size=1)
    path_pub = rospy.Publisher("/pathcells", GridCells, queue_size=1)

    twistPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist ,queue_size=2)


    rospy.sleep(1)

    rospy.Timer(rospy.Duration(.2), robotLocationCallBack)

    while (1 and not rospy.is_shutdown()):
        #publishing map data every 2 seconds
        displayGrid(mapData)  
        rospy.sleep(2)