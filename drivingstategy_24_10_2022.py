from gettext import find
import numpy as np
import cv2
import actions
from drive_middle import NUM_PARTICLES
from drive_middle import find_pose
import robot
import particle
import Self_localization_slow as sls
from time import sleep
from camera import Camera
from particle import move_particle
from Parameters import params
import time

arlo = robot.Robot()
cam = Camera(0, robottype = 'arlo', useCaptureThread = True)
Dict, camera_matrix, dist_coeffs, markerLength = params()
sleep(1)


landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (300.0, 0.0),  # Coordinates for landmark 2
    3: (0.0, 300.0),
    4: (300.0, 300.0)
}
#temp
def drive_random():
   return 0
 
landmarks_required = [1,2,3,4]
# scanner indtil den enten har scannet to kasser eller har drejet en hel omgang
status1 = False
status2 = False
status3 = False 
status4 = False
status5 = False
finished = False


while not finished:
   visited_landmarks =  []
   scanned_landmarks = []
   #self localizing
   est_pose = [0,0,0]
   
   #Hvis den ikke har besøgt landmark 1. 
   while not status1:
       #Tager et billede og finder dist til tætteste object med id 1
       corners, ids = actions.get_corners_ids(cam, dict, 1)
       dist_mm, ang, sign = actions.detector(corners)
       actions.turn_degrees(ang, sign) #Drejer hen imod objected
       
       #Gør klar til at køre imod objected
       safety_dist = 0.20
       start_time = time.perf_counter()
       time_cap = 2.235 * ( float(dist_mm*0.001) - safety_dist)
       
       #Kører imod obejcted og tjekker hele tiden sensor
       arlo.go_diff(69, 70, 1, 1)
       while True:
           if (float(time.perf_counter()) - float(start_time)) > time_cap:
               arlo.stop()
               break
           if (arlo.read_front_ping_sensor() >= safety_dist+5 
               and arlo.read_left_ping_sensor >= safety_dist+5 
               and arlo.read_right_ping_sensor >= safety_dist+5):
               arlo.stop()
               break
       
       #Check if it is close to id 1
       status1 = actions.am_i_close(cam, 1)
       
       #If it is not close enough?? Then what? Maybe turn 30 degrees, 
       # drive 15cm and turn 30 degrees back? As if it is not close enough,
       # it must be because a object is close to the side sensors. 


   while not status2:
       pass
   
   while not status3:
       pass
   
   while not status4:
       pass
   
   while not status5:
       pass
       

   if len(visited_landmarks) == 5:
      finished = True
      


"""
      NUM_PARTICLES = 20000
      particles = NUM_PARTICLES
      pose, objects = find_pose()
      if pose:
         status = True
         est_pose = pose
      else: 
         drive_random()
"""


















































