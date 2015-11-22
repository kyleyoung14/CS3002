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

def displayPath(path, start, goal):
    pathPoints = []
    p_cells = newCell(res)
    length = len(path)
    i = 0

    while(i < length):
        pathPoints.append(path.values()[i])
        i += 1

    print "printing path"
    waypoints = []
    j = 1
    length = len(path)

    while(j + 1 < length):
        if(pathPoints[j][1] == pathPoints[j-1][1] and pathPoints[j][0] == pathPoints[j+1][0]):
            waypoints.append(pathPoints[j])

        elif(pathPoints[j][0] == pathPoints[j-1][0] and pathPoints[j][1] == pathPoints[j+1][1]):
            waypoints.append(pathPoints[j])

        j += 1

    waypoints.append(goal)

    #publish path
    for k in range(0, len(pathPoints)):
        tmpX = pathPoints[k][0]
        tmpY = pathPoints[k][1]

        tmpX = tmpX - (tmpX % 1)
        tmpY = tmpY - (tmpY % 1)

        p_cells.addPoint(tmpX, tmpY, res)

    path_pub.publish(p_cells.a)

    #publish waypoints
    for k in range(0, len(waypoints)):
        tmpX = waypoints[k][0]
        tmpY = waypoints[k][1]

        tmpX = tmpX - (tmpX % 1)
        tmpY = tmpY - (tmpY % 1)

        w_cells.addPoint(tmpX,tmpY,res)

    pathPub.publish(w_cells.a)


    print "done"
    return waypoints

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
    

    e_cells = newCell(res)
    b_cells = newCell(res)
    f_cells = newCell(res)
    u_cells = newCell(res)
    w_cells = newCell(res)
        
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
    frontier_pub.publish(f_cells.a)  

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b 
    return abs(x1 - x2) + abs(y1 - y2)


# reads in map data
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
    displayPath(PathToGoal, startPos, goalPos)

    
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

    

##### MAIN #####

if __name__ == '__main__':

    global target
    global gridPub
    global pathPub
    global startPos
    global frontier_pub
    global unknown_pub
    global boundary_pub
    global explored_pub

    AMap = 0
    path = 0

    rospy.init_node('lab3')
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
    markerSub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, readGoal)
    sub = rospy.Subscriber("/start", PoseWithCovarianceStamped, startCallBack)

    pathPub = rospy.Publisher('/waypoints', GridCells, queue_size=1)
    path_pub = rospy.Publisher('/pathcells', GridCells, queue_size=1)
    gridPub = rospy.Publisher("/boundary", GridCells, queue_size=1)

    frontier_pub = rospy.Publisher("/frontier", GridCells, queue_size=1)            
    boundary_pub = rospy.Publisher("/boundary", GridCells, queue_size=1)
    explored_pub = rospy.Publisher("/explored", GridCells, queue_size=1)
    unknown_pub = rospy.Publisher("/unknown", GridCells, queue_size=1)

    rospy.sleep(1)

    while (1 and not rospy.is_shutdown()):
        #publishing map data every 2 seconds
        displayGrid(mapData)  
        rospy.sleep(2)
