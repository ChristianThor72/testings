import robot
import time
from time import sleep
import numpy as np
import cv2
from camera import Camera
from Parameters import params
import random

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
    for _ in range(18):
        turn_degrees(20, 1) #Turning right
        sleep(0.75) #Sleep time it takes to turn 30 degrees.
        temp_frame = cam.get_next_frame()
        corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
        if corners:
            dist, ang_deg, signfunc = detector(corners, markerLength, camera_matrix, dist_coeffs)
            turn_degrees(ang_deg, signfunc)
            sleep(0.5)
            arlo.stop()
            break
        arlo.stop()
        
def get_corners_ids(cam, dict, obj_ids):
    temp_frame = cam.get_next_frame()
    corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
    
    temp_corners = []
    for i in range(len(ids)):
       if ids[i] == obj_ids:
           temp_corners.append(corners[i])
    dist, _, _ = detector(temp_corners)
    index = np.argmin(dist)
    corners = temp_corners[index]
        
    return corners, obj_ids
    

def am_i_close(cam, obj_ids):
    temp_frame = cam.get_next_frame()
    corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
    
    temp_corners = []
    for i in range(len(ids)):
       if ids[i] == obj_ids:
           temp_corners.append(corners[i])
    dist, _, _ = detector(temp_corners)
    index = np.argmin(dist)
    corners = temp_corners[index]
    
    if dist[index] < 0.40 and ids == obj_ids:
        return True
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

def find_pose(particles):
    #List of found Id's aka boxes
    id_lst = []
    pose = None
    #Making list for the temperary particle poses. 
    parties_lst = []
    for _ in range(2):
        frameReference = cam.get_next_frame() # Read frame
        corners, ids, _ = cv2.aruco.detectMarkers(frameReference, dict)
        cv2.aruco.drawDetectedMarkers(frameReference,corners)
        
        #Scanning for object
        if not corners or ids in id_lst:
            iterations = actions.scan_for_object(cam, dict)
            move_particle(particles, 0,0, -0.349 * iterations)
            particle.add_uncertainty(particles, 3, 0.05)
            sleep(0.5)
        
        # Checking if any object found
        if corners:
            
            # Checking if the object found is the first obejct found
            if len(id_lst) == 0:
                
                #Making sure we are not just seeing the same object again. If we do, nothing happens, and it will start scanning again
                if ids not in id_lst: 
                    id_lst.append(ids)
                    
                    #Calling selflocate to get some poses ready for when second object is found
                    _, _, _, parties = sls.self_locate(cam, frameReference, particles)
                    # Saving poses
                    #parties_lst.append(parties)
                    particles = parties
                    
            # If the object found is not the first obejct found, then use this.            
            elif len(id_lst) == 1:
                
                #Making sure we are not just seeing the same object again. If we do, nothing happens, and it will start scanning again
                if ids not in id_lst:
                    id_lst.append(ids)
                    
                    #Using poses from when we saw the object before. Now we should have a good pose.
                    theta, x, y, parties = sls.self_locate(cam, frameReference, particles)  
                    pose = [x, y, theta]
    
    # Return the best estimated pose
    return pose, id_lst
        