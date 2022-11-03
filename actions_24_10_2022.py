import robot
import time
import particle
from time import sleep
import numpy as np
import cv2
from camera import Camera
from Parameters import params
from particle import move_particle
import Self_localization_slow as sls
import random

dict, camera_matrix, dist_coeffs, markerLength = params()

arlo = robot.Robot()
sleep(1)


def backward_m(m, leftSpeed = 69, rightSpeed = 70):
    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))
    while True:
        if ((float(time.perf_counter()) - float(start_time)) > 2.235 * float(m) - 2.235 * 0.15 ):
            print(arlo.stop())
            break  

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
    print(corners, tvec)
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
    

def scan_for_object(cam,dict, obj_ids):
    for i in range(18):
        turn_degrees(20, 1) #Turning right
        sleep(1.5) #Sleep time it takes to turn 30 degrees.
        temp_frame = cam.get_next_frame()
        corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
        temp_corners = []
        dists = []
        if corners and obj_ids in ids:
            for i in range(len(ids)):
                if ids[i] == obj_ids:
                    temp_corners.append(corners[i])
                    dist, _, _ = detector(corners[i], markerLength, camera_matrix, dist_coeffs)
                    dists.append(dist)
            dists = np.array(dists)
            print("DISTANCER: ", dists)
            index = np.argmin(dists)
            corners = temp_corners[index]
        #if corners and obj_ids in ids:
            dist, ang_deg, signfunc = detector(corners, markerLength, camera_matrix, dist_coeffs)
            turn_degrees(ang_deg, signfunc)
            sleep(0.5)
            arlo.stop()
            return 1
            break
        elif i == 17:
            return 0
        arlo.stop()
        
def get_corners_ids(obj_ids, corners, ids):
    temp_corners = []
    dists=[]
    for i in range(len(ids)):
       if ids[i] == obj_ids:
            temp_corners.append(corners[i])
            dist, _, _ = detector(corners[i], markerLength, camera_matrix, dist_coeffs)
            dists.append(dist)
    dists = np.array(dists)
    print("DISTANCER: ", dists)
    index = np.argmin(dists)
    print("closest dist", index, dists[index])
    corners = temp_corners[index]
    
    return corners, obj_ids
    

def am_i_close(cam, obj_ids):
    temp_frame = cam.get_next_frame()
    corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
    
    temp_corners = []
    dists=[]
    if corners:
        for i in range(len(ids)):
            if ids[i] == obj_ids:
                temp_corners.append(corners[i])
                dist, _, _ = detector(corners[i], markerLength, camera_matrix, dist_coeffs)
                dists.append(dist)
        dists = np.array(dists)
        print("DISTANCER: ", dists)
        index = np.argmin(dists)
        corners = temp_corners[index]
          
        if dists[index] < 1000 and obj_ids in ids:
            return True
        else:
            return False
    else:
        return False

def NOT(a):
    if a != 0 and a != 1:
        return "error"
    binary = 0
    if not a == True:
        binary = 1
    return binary

def panic_mode(safety_dist):
    if arlo.read_front_ping_sensor() >= safety_dist:
        turn_direc = random.randint(0, 1)
        arlo.go_diff(40, 40, turn_direc, NOT(turn_direc))

def drive_random():

    safety_dist = 100
    turn_degrees(random.randint(0, 360),1)
    sleep(1)
    final_dist = random.randint(750,2000)
    time_cap = 2.235 * ( float(final_dist*0.001))
    print("Hvor langt den bør køre: ", time_cap)
    start_time = time.perf_counter()
    arlo.go_diff(70, 70, 1, 1)
    print("Begynder at køre")
    while True:
        if (float(time.perf_counter()) - float(start_time)) > time_cap:
            arlo.stop()
            break

        if not (arlo.read_front_ping_sensor() >= safety_dist 
        and arlo.read_left_ping_sensor() >= safety_dist 
        and arlo.read_right_ping_sensor() >= safety_dist):
            arlo.stop()
            break  
    print("Er færdig med at køre")
def find_pose(particles, cam, obj_ids):
    while True:
        pose = None
        found_id = False
        #get corners and ids 
        frameReference = cam.get_next_frame()
        corners, ids, _ = cv2.aruco.detectMarkers(frameReference, dict)
        cv2.aruco.drawDetectedMarkers(frameReference,corners)
        scan_succes = -1
        #if no box is found or the same box is found
        if not corners or obj_ids not in ids:  
            while scan_succes == -1:
                scan_succes = scan_for_object(cam, dict, obj_ids)
                break
            print("scanned done!!!")
            print(scan_succes)
            

            if scan_succes == 0: #0 is fail
                sleep(1)
                drive_random()
                sleep(5)
               
            
        elif corners and obj_ids in ids:
            if scan_succes ==-1:
                
                temp_corners = []
                dists = []
                for i in range(len(ids)):
                    if ids[i] == obj_ids:
                            temp_corners.append(corners[i])
                            dist, _, _ = detector(corners[i], markerLength, camera_matrix, dist_coeffs)
                            dists.append(dist)
                
                dists = np.array(dists)
                print("DISTANCER: ", dists)
                index = np.argmin(dists)
                corners = temp_corners[index]
                         
            theta, x, y, parties = sls.self_locate(cam, frameReference, particles)  
            particles = parties
            pose = [x, y, theta]
            return pose, particles, dists[index]
        