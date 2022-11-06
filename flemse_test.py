# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 16:19:39 2022

@author: flemm
"""

from gettext import find
from traceback import print_tb
import numpy as np

import cv2
import actions_final as actions
from actions_final import find_pose, am_i_close
import robot
import particle
import Self_localization_slow as sls
from time import sleep
from camera import Camera
from particle import move_particle
from Parameters import params
import time
import sys

arlo = robot.Robot()
cam = Camera(0, robottype = 'arlo', useCaptureThread = True)
sleep(1)
dict, camera_matrix, dist_coeffs, markerLength = params()


NUM_PARTICLES = 20000
particles = sls.initialize_particles(NUM_PARTICLES)

landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),
    4: (400.0, 300.0)
}
#temp


visited_landmarks =  []
scanned_landmarks = []
landmarks_required = [1,2,3,4]
# scanner indtil den enten har scannet to kasser eller har drejet en hel omgang
status1 = False
status2 = False
status3 = False 
status4 = False
status5 = False
finished = False


est_pose_global = [0]
def driving_to_box(current_id, status, est_pose_global):
    while not status:
        print(f"status {current_id}: ", status)
        particles = sls.initialize_particles(NUM_PARTICLES)
        safety_dist = 0.35    
        print("Object in site", actions.object_in_site(cam, current_id))
        if actions.object_in_site(cam, current_id):
            est_pose, particles, cam_dist = find_pose(particles, cam, current_id)
            print("pose lokal", est_pose)
            delta_x = landmarks[current_id][0]*10 - est_pose[0]*10
            delta_y = landmarks[current_id][1]*10 - est_pose[1]*10
            dist_mm = np.sqrt((delta_x**2) + (delta_y**2))
            theta = 0 #vi forventer at vi kigger op kassen
            #actions.drive_to_object(dist, 0, 1)
            final_dist = max(dist_mm, cam_dist)
            
            time_cap = 2.235 * ( float(final_dist*0.001) - safety_dist)
            print("Dette er distancen: ", final_dist)
            
            #Kører imod obejcted og tjekker hele tiden sensor
            print("DRIVING")
            #arlo.go_diff(70, 70, 1, 1)
            print("can see object!!!", current_id)
            actions.drive_to_current_id(cam, time_cap, 350, current_id)
            #Check if it is close to id
            sleep(1)
            actions.backward_m(0.7)
            sleep(1)
            status = am_i_close(cam, current_id)
            if not status:
                sleep(1)
                actions.backward_m(0.35)
                sleep(1)
                status = am_i_close(cam, current_id)                             
            est_pose_global[0] = est_pose
            
        #Søg efter object
        print("Object in site2", actions.object_in_site(cam, current_id))
        if not actions.object_in_site(cam, current_id):
            print("cannot see object. Start scanning")
            scan_val = actions.scan_for_object(cam, dict, current_id)
            print("Scan Value: ", scan_val)
            if scan_val == 0:
                print("going in direction of box")
                actions.going_in_direction_of_box(cam, current_id, 
                                                  est_pose_global[0], 
                                                  landmarks, safety_dist, 
                                                  time_cap = 5)   
                
        #Når den kan se objektet skal den følgende. 

        #Hvis den ikke kan se objektet, kør i retning af det ud fra pose. 
        
        
 
        
        if status:
            est_pose, particles, cam_dists = find_pose(particles, cam, current_id)
            visited_landmarks.append(current_id)
 


print("global pose1", est_pose_global)           
driving_to_box(1,status1, est_pose_global)
print("global pose2", est_pose_global)
driving_to_box(2,status2, est_pose_global)
print("global pose3", est_pose_global)
driving_to_box(3,status3, est_pose_global)
print("global pose4", est_pose_global)
driving_to_box(4,status4, est_pose_global)
print("global pose5", est_pose_global)
driving_to_box(1,status5, est_pose_global)

print("FINITO")

sys.exit()






























