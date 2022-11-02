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



cam = Camera(0, robottype = 'arlo', useCaptureThread = True)

dict, camera_matrix, dist_coeffs, markerLength = params()
arlo = robot.Robot()
sleep(0.02)


landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),
    4: (400.0, 300.0)
}


def avoid_drive(obj_ids = [1]):
    ex = ([1,0,0])
    ez = ([0,0,1])
    temp_frame = cam.get_next_frame()
    corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
    
    if list(ids) not in obj_ids:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        dist = np.linalg.norm(tvec)
        theta = np.arccos(np.dot((tvec/dist),ez))
        signfunc = np.sign(np.dot(tvec,ex))
        ang_deg = signfunc * np.rad2deg(theta)
        sensa = arlo.read_right_ping_sensor()
        sensb = arlo.read_left_ping_sensor()
        print("Typen på a: ", type(sensa), " \n typen på b: ", type(sensb))
        if 70 < ang_deg < 110 and sensa < dist+10 or sensb < dist+10: 
            actions.turn_degrees(60, signfunc)
            sleep(1)
            actions.forward_mm(dist)
            sleep(1)
            actions.turn_degrees(-60, signfunc)
            sleep(1)
            actions.forward_mm(dist)
        elif sensa < dist+10 or sensb < dist+10:
            print("KAN IKKE KØRE")
        else: 
            actions.turn_degrees(theta*2, -signfunc)
            sleep(1)
            actions.forward_mm(dist)
            sleep(1)
            actions.turn_degrees(theta*2, signfunc)
            sleep(1)
            actions.forward_mm(dist)

avoid_drive()
