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
from move_base_msgs.msg import MoveBaseActionResult
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

def centroidSearch2():
    global f_cells
    global noFrontiers

    cells = newCell(res)
    cells = f_cells

    allFrontiers = []
    frontier = []
    newfrontier = Queue.Queue()
    distances = []
    sortedFrontiers = []
    goalPos = []

    averageX = 0
    averageY = 0
    noFrontiers = 0

    goal = Point()
    centroid = Point()
    n = Point()
    m = Point()

    print "length of f_cells.a.cells is"
    print len(f_cells.a.cells)
    print "length of frontier list is"
    print len(cells.a.cells)

    #sort f_cells 
    print "about to sort f_cells"
    while(cells.a.cells and not rospy.is_shutdown()):
	print "length of frontier list is"
	print len(cells.a.cells)
        # get start of new frontier
        n = cells.a.cells[0]
        frontier.append(n)

        #find cells next to the starting cell
        for next in cells.a.cells:
            if (next.x <= n.x+1 and next.x >= n.x-1) and (next.y <= n.y+1 and next.y >= n.y-1):
                print(next)
                newfrontier.put(next)
                frontier.append(next)
                cells.a.cells.remove(next)

        #find remaining cells next to the current ones in frontier
        while(newfrontier.qsize() > 0 and not rospy.is_shutdown()):
            m = newfrontier.get()
            for next in cells.a.cells:
                if(next.x <= m.x+1 and next.x >= m.x-1) and (next.y <= m.y+1 and next.y >= m.y-1):
                    print(next)
                    newfrontier.put(next)
                    frontier.append(next)
                    cells.a.cells.remove(next)
                else:
                    print newfrontier.qsize()

        #add frontier to list of frontiers
        allFrontiers.append(frontier)
        print "added a Frontier"

    
    #evaluate frontiers based on centroid's distance from robot
    print "about to calculate centroids"
    for i in range(len(allFrontiers)):
        averageX = 0
        averageY = 0

        for next in allFrontiers[i]:
            averageX += next.x
            averageY += next.y

        averageX /= len(allFrontiers[i])
        averageY /= len(allFrontiers[i])

        centroid.x = averageX
        centroid.y = averageY

        dist = ((centroid.x - robotX)**2 + (centroid.y - robotY)**2)**0.5

        #make list of indeces. used to remove largest distance centroid from list later
        dist = int(dist*10000)
        print dist
        distances.append(dist)
        sortedFrontiers.append(centroid) 
        print "added a centroid"


    #attempt to go to first frontier centroid
    numLeftOfFrontiers = len(distances)
    while(not rospy.is_shutdown()):
        maxDist = max(distances)
        indexOfMax = distances.index(maxDist)
        
        #remove in case this is not passable and needs to find next highest distance
        distances.remove(maxDist)

        goal = sortedFrontiers[indexOfMax]

        #remove so that the indeces are not out of sync with distances list
        sortedFrontiers.remove(goal)
        goalPos = (goal.x, goal.y)
        #goalPos[0] = goal.x
        #goalPos[1] = goal.y

        if(passable(goalPos)):
            print goalPos
            goToWaypoint(goalPos)
            break

        numLeftOfFrontiers -= 1

        if(numLeftOfFrontiers == 0):
            noFrontiers = 1
            break

    if(noFrontiers == 1):
       # print "Lab complete"
        return



# def centroidSearch(start):
#     global f_cells

#     CurrGrid = SquareGrid()
#     frontier = []
#     midPoint = 0
#     noMoreFrontiers = False
#     dest = Point()
#     toSearch = Queue.Queue()
#     toSearch2 = Queue.Queue()
#     done = False
#     visited = []
#     unknown = []
#     cellsAdded = 0
#     done = 0

#     print "look for first frontier"
#     while (not done and not rospy.is_shutdown()):
#        # print "checking cell"
#         for next in CurrGrid.allNeighbors(start):
#             if(not inList(visited,next)):
#                # print "adding to visited"
#                 visited.append(next)
#         #print visited
#                 toSearch.put(next)
#                # print next

#                 startPoint = Point()
#                 startPoint.x = (next[0]*res) + origin.x #+ .025
#                 startPoint.y = (next[1]*res) + origin.y #+ .025
#                 startPoint.z=0

#         print startPoint
#                 if (f_cells.inNewCell(startPoint)):
#                     print "first boundary found"
#                     done = True
#                     unknown[0] = startPoint.x
#                     unknown[1] = startPoint.y
#                     print unknown
#             else:
#                 pass
#                # print "not adding to visited"
#         # start = next
#         try:
#             start = toSearch.get(block = False)
#            # nextCell = toSearch.get(block = False)
#            # start[0] = nextCell.x
#            # start[1] = nextCell.y
#         except Queue.Empty:
#             print "Done with centroidsearch"
#             return


#     done = False

#     #search the entire boundary
#     while (not done and not rospy.is_shutdown()):
#         print "searching along boundary"

#         for next in CurrGrid.unkNeighbors(unknown):
#             if(not inList(visited,next) and unexplored(next)):
#                # print "adding to visited"
#                 visited.append(next)
#                 toSearch2.put(next)

#                 nextPoint = Point()
#                 nextPoint.x = (next[0]*res) + origin.x + .025
#                 nextPoint.y = (next[1]*res) + origin.y + .025
#                 nextPoint.z=0

#                 if (nextPoint in f_cells):
#                     print nextPoint
#                     cellsAdded += 1
#                     frontier.append(next)

#         # if (cellsAdded == 0):
#         #     done = True

#         # cellsAdded = 0

#         try:
#             unknown = toSearch2.get(block = False)
#            # nextCell = toSearch2.get(block = False)
#            # unknown[0] = nextCell.x
#            # unknown[1] = nextCell.y
#         except Queue.empty:
#             done = True


#     # midPoint = int(len(frontier)/2)
#     # print midPoint
#     # dest = frontier[midPoint]

#  #    return dest

#     average = Point()

#     for next in frontier:
#         average.x += next.x
#         average.y += next.y

#     average.x /= len(frontier)
#     average.y /= len(frontier)

#     return average

def inList(list, point):
    for next in list:
        if point[0] == next[0] and point[1] == next[1]:
            return True

    return False


def unexplored(point):
    global mapData

    tmpX = int(point[0])
    tmpY = int(point[1])

    index = (width*tmpY) + tmpX

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

def passable(point):  

        tmpX = int(point[0])
        tmpY = int(point[1])

        index = (height*tmpY) + tmpX

        return 100 != mapData[index]

class SquareGrid: 
    def __init__(self):
        self = mapgrid

    def in_bounds(self, point):
        currentX = point[0]
        currentY = point[1]
        return currentX < width and currentY < height


    def unknown(self, point):
        tmpX = int(point[0])
        tmpY = int(point[1])

        index = (height*tmpY) + tmpX

        return (mapData[index] == -1)

        #return point not in b_cells.a.cells

    def neighbors(self, point):
        global flag

        (x, y) = point 

        right_cell = (x+1, y)
        bottom_cell = (x, y-1)
        left_cell = (x-1, y)
        top_cell = (x, y+1)

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
        top_left_cell = (x-1,y+1)
        top_right_cell = (x+1,y+1)
        bottom_right_cell = (x+1,y-1)
        bottom_left_cell = (x-1,y-1)

        #neighbor cells are filtered to make sure they are not an obstacle and are on the map 
        results = [right_cell, bottom_cell, left_cell, top_cell, top_left_cell, top_right_cell, bottom_right_cell, bottom_left_cell]
        
        results = filter(self.in_bounds, results)
    
        return results

    def unkNeighbors(self, point):
        global flag

        (x, y) = point 

        right_cell = (x+1, y)
        bottom_cell = (x, y-1)
        left_cell = (x-1, y)
        top_cell = (x, y+1)
        top_left_cell = (x-1,y+1)
        top_right_cell = (x+1,y+1)
        bottom_right_cell = (x+1,y-1)
        bottom_left_cell = (x-1,y-1)
   
        #neighbor cells are filtered to make sure they are not an obstacle and are on the map 
        results = [right_cell, bottom_cell, left_cell, top_cell, top_left_cell, top_right_cell, bottom_right_cell, bottom_left_cell]
        results = filter(self.in_bounds, results)
        results = filter(self.unknown, results)
        print results
        return results


class newCell:
    def __init__(self, size):
        self.a = GridCells()    
        self.a.header.frame_id = 'map'
        self.a.cell_width = size
        self.a.cell_height = size
        self.a.cells = []           
        
    def inNewCell(self, comp):
        for next in self.a.cells:
            if (comp.x > next.x-.025 and comp.x < next.x+.025) and (comp.y > next.y-.025 and comp.y < next.y+.025):
                return True
        return False

    def get_Size(self):
        return self.a.cell_width

    def addPoint(self, x, y,res):
        point = Point()
        point.x = (x*res) + origin.x + .025
        point.y = (y*res) + origin.y + .025
        point.z=0
        self.a.cells.append(point)


def displayGrid(data):
    global frontier_pub
    global boundary_pub
    global b_cells
    global f_cells
    global w_cells
    global p_cells
    global height
    global width

    b_cells = newCell(res)
    f_cells = newCell(res)
    w_cells = newCell(res)
    p_cells = newCell(res)

    CurrGrid = SquareGrid()
        
    cell = 0    
    for y in range(0,height):
        for x in range (0,width):        
            if (mapData[cell] >= 80):
                b_cells.addPoint(x,y,res)
            
            if (mapData[cell] == -1):
                if(mapData[(cell-1)] >= 0 and mapData[(cell-1)] < 80):
                    f_cells.addPoint(x,y,res)  
                if(mapData[(cell-width)] >= 0 and mapData[(cell-width)] < 80):
                    f_cells.addPoint(x,y,res)
                if((cell+1) < ((width-1) * (height-1))):
                    if(mapData[(cell+1)] >= 0 and mapData[(cell+1)] < 80):
                        f_cells.addPoint(x,y,res)
                    if(mapData[(cell+width)] >= 0 and mapData[(cell+width)] < 80):
                        f_cells.addPoint(x,y,res)


            cell += 1
   
    #Results are published
    boundary_pub.publish(b_cells.a)
    frontier_pub.publish(f_cells.a)


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
    global move_done
    newPose = PoseStamped()
    newPose.pose.position.x = point[0]
    newPose.pose.position.y = point[1]

    #print newPose.pose.position.x
    #print newPose.pose.position.y

    newPose.pose.orientation.w = robotTheta 
    newPose.header.frame_id = "map"
    newPose.header.stamp = rospy.Time.now()

    move_pub.publish(newPose)

    move_done = False

    while(not move_done and not rospy.is_shutdown()):
       # print "Moving..."
        pass

    
def publishTwist(lin_speed, ang_speed):
    # Retrieve one global variable
    #   pub - Publisher for Twist messages
    global pub 

    # Create a Twist message
    twist_msg = Twist();        

    # Set the linear and angular components of Twist message
    twist_msg.linear.x = lin_speed  
    twist_msg.angular.z = ang_speed

    # Publish Twist message

    pub.publish(twist_msg)
    rospy.loginfo(twist_msg)
    rospy.sleep(0.1) 

def spinWheels(u1, u2, time):

    wheel_base = 23.0
    wheel_rad = 3.5

    # Create variable called pub, r, and b
    # radius is the distance from the ICC and the center point between the two wheels
    # b is the is the distance between the centers of the two wheels

    # Calculation for the radius of the arc
    if (u1 == u2):
        radius = 0.0 # No rotation
    else:
        radius = (wheel_base / 2.0) * (u1 + u2) / (u1 - u2)

    # Calculation for the linear and angular velocities
    lin_speed = (wheel_rad / 2.0 * u1) + (wheel_rad / 2.0 * u2)

    # Speed Restraint
    if (lin_speed > 1):
        lin_speed = 1

    ang_speed = (wheel_rad / (2.0 * wheel_base) * u1) - (wheel_rad / (2.0 * wheel_base) * u2)
    print(ang_speed)
    # Retrieve the current time in seconds
    now = rospy.Time.now().secs

    # While the elapsed time has not been reached and not shut down ...
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        t = rospy.Time.now().secs - now

        linear = lin_speed
        angular = ang_speed

        # Publish twist_msg to topic
        publishTwist(linear, angular)
        rospy.sleep(0.1)

    # After the required time has passed or has been shut down ...
    # Publish stop command to topic
    publishTwist(0, 0)
    rospy.sleep(0.1)


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
    robotX = position[0]
    robotY = position[1]
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
    print "Finished Moving!"
    global move_done
    move_done = True



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
    global pub
    global noFrontiers

    noFrontiers = 0
    robotX = 0
    robotY = 0
    robotTheta = 0

    rospy.init_node('lab5')

    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
    odomSub = rospy.Subscriber('/odom', Odometry, odomCallBack)
    baseStatusSub = rospy.Subscriber("move_base/result", MoveBaseActionResult, statusCallBack)

    waypoint_pub = rospy.Publisher('/waypoints', GridCells, queue_size=1)
    frontier_pub = rospy.Publisher("/frontier", GridCells, queue_size=1)            
    boundary_pub = rospy.Publisher("/boundary", GridCells, queue_size=1)
    path_pub = rospy.Publisher("/pathcells", GridCells, queue_size=1)
    move_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist ,queue_size=2)

    rospy.sleep(1)

    rospy.Timer(rospy.Duration(1), displayGrid)
    rospy.Timer(rospy.Duration(.2), robotLocationCallBack)

    rospy.sleep(1)

    boundaries = True

    #rotate 360
    goToWaypoint(((robotX + .3), robotY))
    rospy.sleep(.5)
    goToWaypoint(((robotX - .3), robotY))
    data = []
    displayGrid(data)


    print "finished rotating"
    rospy.sleep(3)
    while(not rospy.is_shutdown()):
        centroidSearch2()

    #fill map
    #while (boundaries and not rospy.is_shutdown()):
        #print "starting main while loop in search"
 #  #calculate boundary centroid
    # startPos = (robotX, robotY)
    # goalPos = Point()
    # goalPos = centroidSearch(startPos)

    # print(goalPos)

    #boundaries = False

 #        #AStar to centroid
    #   PathToGoal = AStar(startPos, goalPos)
    #   Waypoints = displayGrid(PathToGoal, startPos, goalPos)

 #        #go to first waypoint
 #        goToWaypoint(Waypoints[0])


 #        #scan
    #   scan()

 #        #update boundary
 #        #if(no boundary):
 #            #boundaries = False
    #   pass
        

    print "Lab Complete!"
    rospy.sleep(1)

