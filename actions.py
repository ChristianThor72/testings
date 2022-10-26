import robot
import time
from time import sleep
import numpy as np
import cv2
from camera import Camera
from Parameters import params

Dict, camera_matrix, dist_coeffs, markerLength = params()

arlo = robot.Robot()
sleep(0.02)


def forward_m(m, leftSpeed = 69, rightSpeed = 70):
    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    while True:
        if ((float(time.perf_counter()) - float(start_time)) > 2.235 * float(m) - 2.235 * 0.15 ):
            print(arlo.stop())
            break  

def forward_mm(m, leftSpeed = 69, rightSpeed = 70):
    forward_m(m*0.001)


def turn_degrees(degrees, sign, leftSpeed = 60 , rightSpeed = 60): #it will spin clockwise if turn>
    scalar = degrees/ 90
    
    spin_lw, spin_rw = 1 , 0    #Choose spin direction. sign = 1, turn right, sign=-1 turn left 
    if sign == -1: 
        spin_lw, spin_rw = 0 , 1

    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, spin_lw, spin_rw))
    while True:
        if (time.perf_counter() - start_time > 0.675*scalar):
            print(arlo.stop())
            break


def detector(corners, markerLength, camera_matrix, dist_coeffs):
    ex = ([1,0,0])
    ez = ([0,0,1])
    
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
    dist = np.linalg.norm(rvec-tvec)
    tvec = tvec.reshape((3,))
    dist = np.linalg.norm(tvec)
    theta = np.arccos(np.dot((tvec/dist),ez))
    signfunc = np.sign(np.dot(tvec,ex))
    ang_deg = signfunc * np.rad2deg(theta)
    
    return dist, np.rad2deg(theta), signfunc

def drive_to_object(dist_mm, ang, sign):
    turn_degrees(ang, sign)
    sleep(0.5)
    forward_mm(dist_mm)
    sleep(0.5)
    

def scan_for_object(cam,dict):
    for i in range(18):
        turn_degrees(20, 1) #Turning right
        sleep(0.75) #Sleep time it takes to turn 30 degrees.
        temp_frame = cam.get_next_frame()
        corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
        if corners:
            dist, ang_deg, signfunc = detector(corners, markerLength, camera_matrix, dist_coeffs)
            turn_degrees(ang_deg, signfunc)
            sleep(0.5)
            arlo.stop()
            return i+1
            break
    if not corners:
        return 0 
        arlo.stop()
        
    


        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
