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
import math

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
    

def scan_for_object(cam, dict, obj_ids):
    for i in range(30):
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
    
""" Kan muligvis slettet!!!
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
"""

    

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

def cut_down_corners(corners, ids, obj_ids):
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
        dist = dists[index]
        return corners, dist
    else:
        return None, 1e10    
    
def find_pose(particles, cam, obj_ids):
    while True:
        pose = None
        found_id = False
        #get corners and ids 
        frameReference = cam.get_next_frame()
        corners_temp, ids, _ = cv2.aruco.detectMarkers(frameReference, dict)
        corners, _ = cut_down_corners(corners_temp, ids, obj_ids)
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
        
        





def am_i_close(cam, obj_ids):
    temp_frame = cam.get_next_frame()
    temp_corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
    corners, dist = cut_down_corners(temp_corners, ids, obj_ids)
    if dist < 1000 and obj_ids in ids:
        return True
    else:
        return False

def correct_angle(cam, obj_ids):
    temp_frame = cam.get_next_frame()
    corners_temp, ids, _ = cv2.aruco.detectMarkers(temp_frame, dict)
    if corners_temp:
        corners, _ = cut_down_corners(corners_temp, ids, obj_ids)
        _, ang_deg, signfunc = detector(corners, markerLength, camera_matrix, dist_coeffs)
        turn_degrees(ang_deg, signfunc)
        sleep(0.5)

def drive_to_current_id(cam, time_cap, safety_dist, current_id):
    while True:
        start_time = time.perf_counter()
        arlo.go_diff(70, 70, 1, 1)
        if (float(time.perf_counter()) - float(start_time)) > time_cap:
            arlo.stop()
            break
        if arlo.read_left_ping_sensor() <= safety_dist:
            print(arlo.read_left_ping_sensor())
            turn_degrees(35, 1)
            sleep(1)
            forward_m(0.5, 40, 40)
            sleep(1)
            turn_degrees(35, -1)
            sleep(1)
            correct_angle(cam, current_id)
            sleep(0.5)

        if arlo.read_right_ping_sensor() <= safety_dist:
            print(arlo.read_right_ping_sensor())
            turn_degrees(35, -1)
            sleep(1)
            forward_m(0.5, 40, 40)
            sleep(1)
            turn_degrees(35, 1)     
            sleep(1)
            correct_angle(cam, current_id)
            sleep(0.5)
            
        if arlo.read_front_ping_sensor() <= safety_dist+100:
            arlo.stop()
            break


def turn_towards_next_box(pose, current_id, landmarks):
    x0, y0, theta0 = pose
    x, y = landmarks[current_id]
    delta_x, delta_y = x0-x, y0-y
    dist_from_particle_to_box = math.sqrt(pow(delta_x,2) + pow(delta_y, 2)) 
    theta_corr = np.arccos(delta_x/dist_from_particle_to_box) 
    if delta_y < 0:
        theta_corr = 2*np.pi - theta_corr
    theta_new = (theta0-theta_corr)*180/np.pi
    return theta_new


def object_in_site(cam, current_id):
    temp_frame = cam.get_next_frame()
    corners, ids, _ = cv2.aruco.detectMarkers(temp_frame, dict)
    corners = cut_down_corners(corners, ids, current_id)
    if corners:
        return True
    else:
        return False


#If we cant see the next box, assuming we have scanned for object. 
def going_in_direction_of_box(cam, current_id, pose, landmarks, safety_dist, time_cap = 5):
    #Scan for object we just have seen as we want to face it again to be sure of our direction. 
    #We havent moved so should not be a problem. 
    _ = scan_for_object(cam, dict, current_id-1)
    #Turn towards the next object.
    turn_towards_next_box(pose, current_id, landmarks)
    
    while True:    
        #Drive towards the next object even though we cant see it. 
        drive_to_current_id(cam, time_cap, safety_dist, current_id)
        
        #Check if object has come into sight when time_cap is up. 
        if object_in_site(cam, current_id):
            break
    
    





















        