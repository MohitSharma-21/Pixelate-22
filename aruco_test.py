import cv2
import numpy as np
import cv2.aruco as aruco


# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)


# Create grid board object we're using in our stream
board = aruco.GridBoard_create(
        markersX=2,
        markersY=2,
        markerLength=0.09,
        markerSeparation=0.01,
        dictionary=ARUCO_DICT)

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

img=cv2.imread('media/sample_image.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#Detect Aruco markers
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

# Make sure all 5 markers were detected before printing them out
if ids is not None:
    # Print corners and ids to the console
	for i, corner in zip(ids, corners):
		print('ID: {}; Corners: {}'.format(i, corner))
        # Outline all of the markers detected in our image

# img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 0, 255) border)
img = cv2.rectangle(img, (int(corners[0][0][0][0]),int(corners[0][0][0][1])), (int(corners[0][0][2][0]),int(corners[0][0][2][1])), (0,255,0), 2)

print(corners[0][0])
cv2.imwrite("media/aruco_detected.png", img)
cv2.imshow("img" , img)
cv2.waitKey(0)