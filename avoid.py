from turtle import forward
import numpy as np
import cv2
import actions_24_10_2022 as actions
from actions_24_10_2022 import find_pose, am_i_close
import robot
import particle
import Self_localization_slow as sls
from time import sleep
from camera import Camera
from particle import move_particle
from Parameters import params
import time
import sys




Dict, camera_matrix, dist_coeffs, markerLength = params()
arlo = robot.Robot()
sleep(0.02)


landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),
    4: (400.0, 300.0)
}


def avoid_drive():

    temp_frame = cam.get_next_frame()
    corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
    forward_m(1)
    if ids not in obj_ids:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        dist = np.linalg.norm(tvec)
        theta = np.arccos(np.dot((tvec/dist),ez))
        signfunc = np.sign(np.dot(tvec,ex))
        ang_deg = signfunc * np.rad2deg(theta)
        turn_degrees(theta, -signfunc)
        forward_m(dist+0.10)
        turn_degrees(theta, signfunc, leftSpeed = 60 , rightSpeed = 60)


avoid_drive()