from dis import dis
from re import T
from tabnanny import check
import cv2
import numpy as np
import math
import time

font = cv2.FONT_HERSHEY_COMPLEX

graph = {}
graphKeyERR = set()
checkPosERR = {}
d = set()
inf = 9999999999
img = []
height, width = 0, 0
colorCenter = set()
Center = []
constDistance = 0
antidose = []

c = 'circle'
t = 'triangle'
s = 'square'

first_shape = s
second_shape = c
third_shape = t


######################################################
######################################################
############                                ##########
############            ERROR               ##########
############                                ##########
######################################################
######################################################

# to get same point as determined by contours

def RemoveErr(pointByCalculation):
    f = 0
    for node in colorCenter:
        if(distance(pointByCalculation, node) < 10):
            return node[0], node[1]

    return pointByCalculation[0], pointByCalculation[1]


######################################################
######################################################
############                                ##########
############       Dijisktra                ##########
############                                ##########
######################################################
######################################################


def getPathFromDjisktra(child, path, k, startToEnd):
    if (path[child] == child):
        startToEnd.append(child)
        return startToEnd

    getPathFromDjisktra(path[child], path, k+1, startToEnd)

    startToEnd.append(child)

    print(child)


# sorting set according to weights
def by_wt(node):
    return node[0]


def dijisktra(src, end):
    startToEnd = []
    dist_from_src = {}
    visit = {}
    path = {}
    weightQueue = set()  # s

    for node in graph.keys():
        dist_from_src[node] = inf

    weightQueue.add((0, src))
    dist_from_src[src] = 0

    path[src] = src

    ###########################################

    # cv2.putText(img, 'start',(src[0],src[1]), font, 1, (255, 255, 255))
    # cv2.putText(img, 'end',(end[0],end[1]), font, 1, (255, 255, 255))

    #####################################################

    while(len(weightQueue) != 0):
        weightQueue = sorted(weightQueue, key=by_wt, reverse=True)
        weightQueue = set(weightQueue)

        front = next(iter(weightQueue))

        weightQueue.discard(next(iter(weightQueue)))

        parent_dist_from_src = front[0]
        parent_name = front[1]

        visit[parent_name] = 1

        for z in graph[front[1]]:
            if(z is None):
                continue

            child_wt = z[1]
            child_name = z[0]

            child_dist_from_src = child_wt + parent_dist_from_src

            if (child_dist_from_src < dist_from_src[child_name]):
                dist_from_src[child_name] = child_wt + parent_dist_from_src
                weightQueue.add((child_dist_from_src, child_name))
                weightQueue = sorted(weightQueue, key=by_wt, reverse=True)
                weightQueue = set(weightQueue)

                #  for path
                path[child_name] = parent_name

    k = 65
    getPathFromDjisktra(end, path, k, startToEnd)

    return startToEnd


######################################################
######################################################
############                                ##########
############  position of neighbouring      ##########
############           boxes                ##########
############                                ##########
######################################################
######################################################

def distance(a, b):
    x = (a[0]-b[0])*(a[0]-b[0])
    y = (a[1]-b[1])*(a[1]-b[1])
    return np.sqrt(x+y)

##############################


def PositioncolorTopRight(presentPos):
    x = presentPos[0] + constDistance*np.cos(np.pi/3)
    y = presentPos[1] - constDistance*np.sin(np.pi/3)

    x = int(x)
    y = int(y)

    return findcolor(presentPos, x, y)


def PositioncolorBottomRight(presentPos):
    x = presentPos[0] + constDistance*np.cos(np.pi/3)
    y = presentPos[1] + constDistance*np.sin(np.pi/3)

    x = int(x)
    y = int(y)

    return findcolor(presentPos, x, y)

#########################


def PositioncolorTopLeft(presentPos):
    x = presentPos[0] - constDistance*np.cos(np.pi/3)
    y = presentPos[1] - constDistance*np.sin(np.pi/3)

    x = int(x)
    y = int(y)

    return findcolor(presentPos, x, y)


def PositioncolorBottomLeft(presentPos):
    x = presentPos[0] - constDistance*np.cos(np.pi/3)
    y = presentPos[1] + constDistance*np.sin(np.pi/3)

    x = int(x)
    y = int(y)

    return findcolor(presentPos, x, y)


#########################
def PositioncolorRight(presentPos):
    x = presentPos[0] + constDistance
    y = presentPos[1]

    x = int(x)
    y = int(y)

    return findcolor(presentPos, x, y)


def PositioncolorLeft(presentPos):
    x = presentPos[0] - constDistance
    y = presentPos[1]

    x = int(x)
    y = int(y)
    return findcolor(presentPos, x, y)

######################################################
######################################################
############                                ##########
############       TYPE OF COLOR            ##########
############                                ##########
######################################################
######################################################

# to check color of boxes and getting thier weights

def findcolor(presentPos, cx, cy):

    if(cx > width or cy > height or cx < 0 or cy < 0):
        return None

    cx, cy = RemoveErr((cx, cy))

    pixel_center = hsv[cy, cx]
    h = pixel_center[0]
    s = pixel_center[1]
    v = pixel_center[2]

    if(h >= 1 and h <= 163 and s <= 220):
        # green
        return ((cx, cy), 4)
    elif(s >= 200 and v <= 120):
        # purple
        return ((cx, cy), 3)
    elif(h >= 5 and h <= 55):
        # yellow
        return ((cx, cy), 2)
    elif(h >= 1 and h <= 23):
        # red
        return ((cx, cy), 1)
    elif(h >= 1 and s <= 179):
        # pink
        return ((cx, cy), 1)
    elif(h <= 21 and v >= 198):
        # white
        return ((cx, cy), 1)
    elif(h >= 62 and s >= 231):
        # blue
        return ((cx, cy), 20)
    else:
        # black
        return None


# adding edges/neighbours of single tile

def addNeighbour(point):

    neighbour = []

    neighbour.append(PositioncolorTopLeft(point))
    neighbour.append(PositioncolorBottomLeft(point))

    neighbour.append(PositioncolorTopRight(point))
    neighbour.append(PositioncolorBottomRight(point))

    neighbour.append(PositioncolorRight(point))
    neighbour.append(PositioncolorLeft(point))

    graph[point] = neighbour


######################################################
######################################################
############                                ##########
############            villains            ##########
############                                ##########
######################################################
######################################################

# this function is for to check wheather the shape is on villains tile

def checkVillain(presentPos, Villainhsv):
    x = presentPos[0] + constDistance/2.3
    y = presentPos[1]

    cx = int(x)
    cy = int(y)

    pixel_center = Villainhsv[cy, cx]
    h = pixel_center[0]
    s = pixel_center[1]
    v = pixel_center[2]

    if(h >= 1 and h <= 163 and s <= 220):
        return 1
    elif(s >= 200 and v <= 120):
        return 1
    elif(h >= 1 and h <= 23):
        return 0
    elif(h >= 5 and h <= 55):
        return 1
    elif(h >= 1 and s <= 179):
        return 0
    elif(h <= 21 and v >= 198):
        return 1
    elif(h >= 62 and s >= 231):
        return 1
    else:
        return 0


def getVillains(frame):
    Villainhsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width, _ = frame.shape

    lower_blue = np.array([0, 201, 1])
    upper_blue = np.array([127, 253, 194])
    mask = cv2.inRange(Villainhsv, lower_blue, upper_blue)
    img = cv2.bitwise_and(frame, frame, mask=mask)

    grayscaled = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    retval, thresholdgray = cv2.threshold(
        grayscaled, 30, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(
        thresholdgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    shape_Detected = {}

    for contour in contours:
        # approx the polygon gives coordinates of vecrtices
        approx_points = cv2.approxPolyDP(
            contour, 0.04 * cv2.arcLength(contour, True), True)
        no_of_vertices = len(approx_points)

        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        if(checkVillain((cx, cy), Villainhsv)):

            cx, cy = RemoveErr((cx, cy))

            if(no_of_vertices == 3):
                shape_Detected['triangle'] = ((cx, cy))

            elif(no_of_vertices == 4):
                shape_Detected['square'] = ((cx, cy))

            else:
                shape_Detected['circle'] = ((cx, cy))
                
    print('Shape Detected', shape_Detected)

    villain_shapes_pos = []
    villain_shapes_pos.append(shape_Detected[first_shape])
    villain_shapes_pos.append(shape_Detected[second_shape])
    villain_shapes_pos.append(shape_Detected[third_shape])

    return villain_shapes_pos


######################################################
######################################################
############                                ##########
############            Antidote            ##########
############                                ##########
######################################################
######################################################

# this function is for to check wheather the shape is on pink tile

def checkAntidote(presentPos, Antidotehsv):
    x = presentPos[0] + constDistance/2.3
    y = presentPos[1]

    cx = int(x)
    cy = int(y)

    pixel_center = Antidotehsv[cy, cx]
    h = pixel_center[0]
    s = pixel_center[1]
    v = pixel_center[2]

    if(h >= 1 and s <= 179):
        return 1
    else:
        return 0


def getAntidote(frame):
    Antidotehsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width, _ = frame.shape

    lower_blue = np.array([0, 201, 1])
    upper_blue = np.array([127, 253, 194])
    mask = cv2.inRange(Antidotehsv, lower_blue, upper_blue)
    img = cv2.bitwise_and(frame, frame, mask=mask)

    grayscaled = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    retval, thresholdgray = cv2.threshold(
        grayscaled, 30, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(
        thresholdgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    shape_Detected = {}

    for contour in contours:
        # appprox the polygon gives coordinates of vecrtices
        approx_points = cv2.approxPolyDP(
            contour, 0.04 * cv2.arcLength(contour, True), True)
        no_of_vertices = len(approx_points)

        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        if(checkAntidote((cx, cy), Antidotehsv)):

            cx, cy = RemoveErr((cx, cy))

            if(no_of_vertices == 3):
                shape_Detected['triangle'] = ((cx, cy))

            elif(no_of_vertices == 4):
                shape_Detected['square'] = ((cx, cy))

            else:
                shape_Detected['circle'] = ((cx, cy))

    print('Shape Detected Antidote', shape_Detected)
    
    antidote_shapes_pos = []
    antidote_shapes_pos.append(shape_Detected[first_shape])
    antidote_shapes_pos.append(shape_Detected[second_shape])
    antidote_shapes_pos.append(shape_Detected[third_shape])

    return antidote_shapes_pos


######################################################
######################################################
############                                ##########
############       SPIDERMANS RED           ##########
############                                ##########
######################################################
######################################################

# this function is for to check wheather the tile is red 
# and get same coordinate as determined by contours

def checkRed(presentPos, Redhsv):
    cx = presentPos[0]
    cy = presentPos[1]

    if(cx > width or cy > height or cx < 0 or cy < 0):
        return None

    cx, cy = RemoveErr((cx, cy))

    pixel_center = Redhsv[cy, cx]
    h = pixel_center[0]
    s = pixel_center[1]
    v = pixel_center[2]

    if(h >= 1 and h <= 23):
        return (cx, cy)
    else:
        return None


def getRED(frame):
    Redhsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([1, 0, 0])
    upper_blue = np.array([23, 255, 255])
    mask = cv2.inRange(Redhsv, lower_blue, upper_blue)

    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    spiderMan = []
    for contour in contours:

        # center of hexagon of a particular color
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        if(checkRed((cx, cy), Redhsv) != None):
            cx, cy = checkRed((cx, cy), Redhsv)
            print(cx, cy)
            spiderMan.append((cx, cy))

    return spiderMan


######################################################
######################################################
############                               ###########
############       Image from arena and    ##########
############     finding nodes and edges   ##########
############                               ###########
######################################################
######################################################


def addNodesAndEdge(frame):
    global img
    global height, width
    global colorCenter
    global Center
    global constDistance
    global graph
    global graphKeyERR
    global checkPosERR
    global d
    global hsv
    # global antidose

    img = []
    height = 0
    width = 0
    colorCenter = set()
    Center = []
    constDistance = 0
    graph = {}
    graphKeyERR = set()
    checkPosERR = {}
    d = set()
    # antidose=[]

    img = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width, _ = frame.shape

    lower_blue = np.array([0, 0, 1])
    upper_blue = np.array([255, 255, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    colorCenter = set()
    Center = []

    # for each conotur finding its center

    for contour in contours:

        # center of hexagon of a particular color
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        colorCenter.add((cx, cy))
        Center.append((cx, cy))

    # distance between neighbouring nodes
    constDistance = distance(Center[0], Center[1])
    colorCenter = sorted(colorCenter)

    # adding edges of each nodes
    for point in colorCenter:
        addNeighbour(point)


##################################################
##################################################
##################################################
##################################################
##################################################
##################################################
##################################################
##################################################
##################################################


def getpath(src, end):
    return dijisktra(src, end)


if __name__ == '__main__':
    c = 0
