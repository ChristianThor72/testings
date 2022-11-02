# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 16:19:39 2022

@author: flemm
"""

from gettext import find
from traceback import print_tb
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


def go_to_box(status, current_id):
    while not status:
       print(f"status {current_id}: ", status)
       particles = sls.initialize_particles(NUM_PARTICLES)

       est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
       
       delta_x = landmarks[current_id][0]*10 - est_pose[0]*10
       delta_y = landmarks[current_id][1]*10 - est_pose[1]*10
       dist_mm = np.sqrt((delta_x**2) + (delta_y**2))
       theta = 0 #vi forventer at vi kigger op kassen
    #     actions.drive_to_object(dist, 0, 1)
       safety_dist = 0.4
       start_time = time.perf_counter()
       final_dist = max(dist_mm, dist_cam)
       
       time_cap = 2.235 * ( float(final_dist*0.001) - safety_dist)
       print("Dette er distancen: ", final_dist)
       
       #Kører imod obejcted og tjekker hele tiden sensor
       print("DRIVING")
       #arlo.go_diff(70, 70, 1, 1)
       while True:
          arlo.go_diff(40, 40, 1, 1)
          if (float(time.perf_counter()) - float(start_time)) > time_cap:
             arlo.stop()
             break
          if arlo.read_left_ping_sensor() <= safety_dist*1000:
             print(arlo.read_right_ping_sensor())
             actions.turn_degrees(35, 1)
             sleep(1)
             actions.forward_m(0.5)
             sleep(1)
             actions.turn_degrees(35, -1)
             sleep(1)
             
          if arlo.read_left_ping_sensor() <= safety_dist*1000:
             print(arlo.read_left_ping_sensor())
             actions.turn_degrees(35, -1)
             sleep(1)
             actions.forward_m(0.5)
             sleep(1)
             actions.turn_degrees(35, 1)     
             sleep(1)
          
          if arlo.read_front_ping_sensor() <= safety_dist*1000:
             arlo.stop()
             break

go_to_box(status1, 1)




























