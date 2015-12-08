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
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
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


def centroidSearch(start):
	CurrGrid = SquareGrid()
	frontier = []
	midPoint = 0
	dest = Point()
	toSearch = Queue.Queue()
	toSearch2 = Queue.Queue()
	done = 0
	numVisited = 0
	visited1 = Queue.Queue()
	visited2 = Queue.Queue()
	unknown = Point()

	while not done and not rospy.is_shutdown():
		for next in CurrGrid.allNeighbors(start):
			toSearch.put(next)
			if unexplored(next) == True:
				unknown = next
				done = 1
				break
		start = toSearch.get()

	done = 0

	while not done and not rospy.is_shutdown():
		for next in CurrGrid.unkNeighbors(unknown):
			toSearch2.put(next)
			if next not in visited2 and unexplored(next) == True:
				numVisited += 1
				visited2.put(next)
				#not sure if I can reference mapData like this
				if (mapData[next] >= 0):
					frontier.put(next)
		if toSearch2.empty():
			done = 1

	midPoint = len(frontier)
	dest.x = frontier[midPoint][0]
	dest.y = frontier[midPoint][1]

	return dest


def unexplored(point):
	global mapData

	tmpX = int(point[0])
	tmpY = int(point[1])

	index = (37*tmpY) + tmpX

	if mapData[index] == -1:
		return True
	else:
		return False


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

        if(straightCounter == 20):
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
    waypoint_pub.publish(w_cells.a)
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

    def notPassable(self, point):
    	tmpX = int(point[0])
        tmpY = int(point[1])

        index = (37*tmpY) + tmpX

        return (mapData[index] > 90)

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

    def allNeighbors(self, point):
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

        return results

    def unkNeighbors(self, point):
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
        results = filter(self.notPassable, results)

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
    global boundary_pub
    global b_cells
    global f_cells
    global w_cells
    global p_cells

    b_cells = newCell(res)
    f_cells = newCell(res)
    w_cells = newCell(res)
    p_cells = newCell(res)
        
    cell = 0    
    for y in range(0,height):
        for x in range (0,width):        
            if (mapData[cell] == 100):
                b_cells.addPoint(x,y,res)
            cell += 1

    #Results are published
    boundary_pub.publish(b_cells.a)


#####============================== Movement ==============================#####
 
def scan():
    rotPose.pose.orientation.w -= (numpy.pi / 2)

    move_pub.publish(rotPose)

    while(not move_done and not rospy.is_shutdown()):
        pass

    rotPose.pose.orientation.w += (numpy.pi / 4)

    move_pub.publish(rotPose)

    while(not move_done and not rospy.is_shutdown()):
        pass


def goToWaypoint(point):
    newPose = PoseStamped()
    newPose.pose.position.x = point[0]
    newPose.pose.position.y = point[1]
    newPose.pose.orientation.w = robotTheta 
    newPose.header.frame_id = "map"
    newPose.header.stamp = rospy.Time.now()

    move_pub.publish(newPose)


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


def robotLocationCallBack(data):
    global robotX
    global robotY
    global robotTheta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
    robotX = ((position[0] - 0.5) / 0.3)
    robotY = (position[1] / 0.3)
    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    robotTheta = yaw


def odomCallBack(data):
    global odom_tf
    global pose

    # Set pose to to be pose of the robot
    pose = data.pose
    # Set geo_quat to be the orientation of the pose of the robot
    geo_quat = pose.pose.orientation


def statusCallBack(data):
    global move_done
    status = data.status_list
    print status
    move_done = (status == 3 or status == 4)


#####============================== MAIN ==============================#####

if __name__ == '__main__':

    global target
    global waypoint_pub
    global frontier_pub
    global boundary_pub
    global path_pub
    global move_pub
    global pose
    global odom_tf
    global odom_list
    global robotX
    global robotY
    global robotTheta
    global move_done

    robotX = 0
    robotY = 0
    robotTheta = 0

    rospy.init_node('lab5')

    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
    odomSub = rospy.Subscriber('/odom', Odometry, odomCallBack)
    baseStatusSub = rospy.Subscriber("move_base/status", GoalStatusArray, statusCallBack)

    waypoint_pub = rospy.Publisher('/waypoints', GridCells, queue_size=1)
    frontier_pub = rospy.Publisher("/frontier", GridCells, queue_size=1)            
    boundary_pub = rospy.Publisher("/boundary", GridCells, queue_size=1)
    path_pub = rospy.Publisher("/pathcells", GridCells, queue_size=1)
    move_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    displayGrid(mapData)

    rospy.Timer(rospy.Duration(2), displayGrid(mapData))
    rospy.Timer(rospy.Duration(.2), robotLocationCallBack)

    rospy.sleep(1)

    boundaries = True

    #rotate 360
    # print("Trying to rotate")
    # rotPose = PoseStamped()
    # rotPose.pose.position.x = robotX
    # rotPose.pose.position.y = robotY
    # rotPose.pose.orientation.w = robotTheta 
    # rotPose.header.frame_id = "map"

    # for i in range(0,3):
    # 	rotPose.pose.orientation.w += (numpy.pi / 2)
    # 	rotPose.header.stamp = rospy.Time.now()

    # 	move_pub.publish(rotPose)

    # 	while (not move_done and not rospy.is_shutdown()):
    # 		pass


    # print "finished rotating"
    startPos = (robotX, robotY)
    goalPos = (0, 0)

    examplePos = (robotX + 2, robotY + 2)

    goToWaypoint(examplePos)

    #fill map
	# while (boundaries and not rospy.is_shutdown()):
 #    	#calculate boundary centroid
	# 	startPos = (robotX, robotY)

	# 	goalPos = Point()
	# 	goalPos = centroidSearch(startPos)

	# 	print(goalPos)

 #        #AStar to centroid
	# 	PathToGoal = AStar(startPos, goalPos)
	# 	Waypoints = displayGrid(PathToGoal, startPos, goalPos)

 #        #go to first waypoint
 #        goToWaypoint(Waypoints[0])


 #        #scan
	# 	scan()

 #        #update boundary
 #        #if(no boundary):
 #            #boundaries = False
	# 	pass
        

    print "Lab Complete!"
