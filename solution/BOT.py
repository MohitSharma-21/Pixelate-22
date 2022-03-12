# This is just a basic template for solution
import gym
import time
import pixelate_arena
import pybullet as p
import pybullet_data
import cv2
import os
import numpy as np
#import cv2.aruco as aruco
import math
import OpencvAndGraph
from cv2 import aruco

##################################################

# This line is to switch the directories for getting resources.
parent_path = os.path.dirname(os.getcwd())
# you don't need to change anything in here.
os.chdir(parent_path)

env = gym.make("pixelate_arena-v0")    # This loads the arena.

#################################################


def dist(a, b):
    x = (a[0]-b[0])*(a[0]-b[0])
    y = (a[1]-b[1])*(a[1]-b[1])
    return np.sqrt(x+y)


###############################################################
###################                              ##############
###################        ARUCO DETECTION       ##############
###################                              ##############
###############################################################


def aruco_detect(img):

    # Constant parameters used in Aruco methods
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect Aruco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

    corners = np.squeeze(corners)
    ids = np.squeeze(ids)

    return corners, ids


###############################################################
###################                              ##############
###################      POSITION AND ANGLE      ##############
###################                              ##############
###############################################################


prevAngle = 0
prevPos = ()


def getAngle(vector):
    comp_1 = complex(vector[0], vector[1])
    angle = np.angle(comp_1, deg=True)

    if(angle < 0):
        angle = 180 + (180 - (-1*angle))

    return angle


def getPositionAndOrientation():
    p.stepSimulation()

    img = env.camera_feed()
    corners, ids = aruco_detect(img)

    # if aruco is not detected then have taken previous position and angle
    # print(ids)
    if(ids == None):
        return prevPos, prevAngle

    # Position of bot
    x = 0
    y = 0
    for cord in corners:
        x = x+cord[0]
        y = y+cord[1]
    x = int(x/4)
    y = int(y/4)

    # orientation of bot

    a = np.array(corners[0])
    b = np.array(corners[3])
    # angle = np.arctan2(a[1]-b[1],a[0]-b[0])*(180/np.pi)

    # ANGLE 2ND METHOD
    angle = getAngle(a-b)

    # print("angle   ",angle)
    pos = np.array([x, y])

    return pos, angle


###############################################################
###################                              ##############
###################        ORIENTING BOT         ##############
###################                              ##############
###############################################################

rot_vel = 5
rot_iter = 15
angle_err = np.pi+2


def orientAlongPath(required_yaw):
    global prevAngle
    global prevPos

    pos, current_yaw = getPositionAndOrientation()

    if(current_yaw > required_yaw):
        if(current_yaw - required_yaw) <= 180:

            while abs(required_yaw - current_yaw) > angle_err:
                print("1 curr ", current_yaw, " ", "req ", required_yaw)

                prevAngle = current_yaw
                prevPos = np.array([pos[0], pos[1]])

                for i in range(rot_iter):
                    p.stepSimulation()
                    env.move_husky(-1*rot_vel, rot_vel, -1*rot_vel, rot_vel)

                p.stepSimulation()
                env.move_husky(0, 0, 0, 0)

                pos, current_yaw = getPositionAndOrientation()
        else:

            while abs(required_yaw - current_yaw) > angle_err:
                print("2 curr ", current_yaw, " ", "req ", required_yaw)

                prevAngle = current_yaw
                prevPos = np.array([pos[0], pos[1]])

                for i in range(rot_iter):
                    p.stepSimulation()
                    env.move_husky(rot_vel, -1*rot_vel, rot_vel, -1*rot_vel)

                p.stepSimulation()
                env.move_husky(0, 0, 0, 0)

                pos, current_yaw = getPositionAndOrientation()

    elif(required_yaw > current_yaw):

        if(required_yaw - current_yaw) <= 180:

            while abs(required_yaw - current_yaw) > angle_err:
                print("3 curr ", current_yaw, " ", "req ", required_yaw)

                prevAngle = current_yaw
                prevPos = np.array([pos[0], pos[1]])

                for i in range(rot_iter):
                    p.stepSimulation()
                    env.move_husky(rot_vel, -1*rot_vel, rot_vel, -1*rot_vel)

                p.stepSimulation()
                env.move_husky(0, 0, 0, 0)

                pos, current_yaw = getPositionAndOrientation()

        else:

            while abs(required_yaw - current_yaw) > angle_err:
                print("4 curr ", current_yaw, " ", "req ", required_yaw)

                prevAngle = current_yaw
                prevPos = np.array([pos[0], pos[1]])

                for i in range(rot_iter):
                    p.stepSimulation()
                    env.move_husky(-1*rot_vel, rot_vel, -1*rot_vel, rot_vel)

                p.stepSimulation()
                env.move_husky(0, 0, 0, 0)

                pos, current_yaw = getPositionAndOrientation()

    p.stepSimulation()
    env.move_husky(0, 0, 0, 0)

    print("********************aligned**************************")


###############################################################
###################                              ##############
###################        MOVING BOT            ##############
###################                              ##############
###############################################################


tran_vel = 3
tran_iter = 245
dist_err = 10

def moveToPoint(dest_x, dest_y):

    dest = np.array([dest_x, dest_y])

    pos, yaw = getPositionAndOrientation()
    prevDistLeft = dist(pos, (dest_x, dest_y))
    # required_yaw = np.arctan2(dest_y - pos[1], dest_x - pos[0])*(180/np.pi)
    required_yaw = getAngle(dest-pos)

    checkAngle = 0

    orientAlongPath(required_yaw)

    while abs(dest_x - pos[0]) > 0.01 or abs(dest_y - pos[1]) > 0.01:
        p.stepSimulation()
        env.move_husky(0, 0, 0, 0)

        global prevAngle
        global prevPos
        prevAngle = yaw
        prevPos = np.array([pos[0], pos[1]])

        for i in range(tran_iter):
            p.stepSimulation()
            env.move_husky(tran_vel, tran_vel, tran_vel, tran_vel)

        if(checkAngle == 0):
            p.stepSimulation()
            env.move_husky(0, 0, 0, 0)

            pos, yaw = getPositionAndOrientation()
            required_yaw = getAngle(dest-pos)
            orientAlongPath(required_yaw)
            checkAngle = 1

        # p.stepSimulation()
        # env.move_husky(0,0,0, 0)

        pos, yaw = getPositionAndOrientation()
        required_yaw = getAngle(dest-pos)

        distLeft = dist(pos, (dest_x, dest_y))

        print("distance left ", distLeft)

        if dist(pos, (dest_x, dest_y)) < dist_err:
            p.stepSimulation()
            env.move_husky(0, 0, 0, 0)
            break
        
        # to keep a check so that if bot moves forward thannn the required then loop will break
        # that is as soon as the distance between the points increases loop breaks

        if (prevDistLeft < distLeft):
            p.stepSimulation()
            env.move_husky(0, 0, 0, 0)
            print("errrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr")
            # orientAlongPath(required_yaw)
            break

        prevDistLeft = distLeft

    p.stepSimulation()
    env.move_husky(0, 0, 0, 0)


###############################################################
###################                              ##############
###################         MAIN                 ##############
###################                              ##############
###############################################################


# capturing image of arena without bot
time.sleep(0.1)
env.remove_car()
time.sleep(0.2)
img = env.camera_feed()
time.sleep(0.2)
env.respawn_car()
time.sleep(0.2)

OpencvAndGraph.addNodesAndEdge(img)


#############################

points = []
pointCheck = {}

###########################


spiderMans = OpencvAndGraph.getRED(img)


# for getting all spidermans position and starting position
pos, yaw = getPositionAndOrientation()
for spidy in spiderMans:

    pointCheck[(spidy[0], spidy[1])] = 0

    d = dist(pos, spidy)
    if(d <= 5):
        BotAtStart = spidy

points.append(BotAtStart)

pointCheck[(BotAtStart[0], BotAtStart[1])] = 1
print(pointCheck)

for spidy in spiderMans:
    if(pointCheck[(spidy[0], spidy[1])] == 0):
        points.append(spidy)


####################################################
print("red ---------- ", points)
# points =
print("red ---------- ", points)
########################################################

f = 0
flag = 0
j = 0
while True:

    pos, yaw = getPositionAndOrientation()

    start = points[j]
    if(j+1 >= len(points)):
        break
    end = points[j+1]

    path = OpencvAndGraph.getpath(start, end)

    i = 1
    while True:

        p.stepSimulation()
        env.move_husky(0, 0, 0, 0)

        if(i < len(path)):
            p.stepSimulation()
            moveToPoint(path[i][0], path[i][1])
            print("&&&&&&&&& reached &&&&&&&&&")

        p.stepSimulation()
        env.move_husky(0, 0, 0, 0)

        i = i+1
        if(i >= len(path)):
            if(flag >= 1):
                time.sleep(0.1)
                if(flag <= 3):
                    print("ANTIDOTE COLLECTED *********************")
                else:
                    print("VILLAIN CURED *********************")

                flag = flag + 1
            break

    if(f == 0 and j == len(points)-2):
        print("$$$$$$$$$$$$ adding  antidote and villains $$$$$$$$$")

        # for getting antidotes
        time.sleep(0.1)
        env.unlock_antidotes()
        time.sleep(0.1)

        # getting antidotes
        img = env.camera_feed()
        antidotes = OpencvAndGraph.getAntidote(img)
        print("antidote ########### ", antidotes)
        # time.sleep(0.1)

        for antidote in antidotes:
            points.append(antidote)

        print("points   ", points)

        # getting villains
        # time.sleep(0.1)
        frame = env.camera_feed()
        villains = OpencvAndGraph.getVillains(frame)
        print("villain @@@@@@@@@ ", villains)

        for villain in villains:
            points.append(villain)

        print("points   ", points)

        # time.sleep(0.5)

        print("points ######### ", points)

        f = 1
        flag = 1
    else:
        print("$$$$$$$$$$$$ already added $$$$$$$$$")
        # f=1

    j = j+1

    if(j == len(points)-1):
        # IF REACHED DESTINATION

        for i in range(40):
            p.stepSimulation()
            env.move_husky(9, 9, 9, 9)

        p.stepSimulation()
        env.move_husky(0, 0, 0, 0)

        for i in range(100):
            print("******************completed******************")
