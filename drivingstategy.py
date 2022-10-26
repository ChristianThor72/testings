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

arlo = robot.Robot()
cam = Camera(0, robottype = 'arlo', useCaptureThread = True)
sleep(1)
dict, camera_matrix, dist_coeffs, markerLength = params()


NUM_PARTICLES = 20000

landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (400.0, 0.0),  # Coordinates for landmark 2
    3: (0.0, 300.0),
    4: (400.0, 300.0)
}
#temp
def drive_random():
   return 0

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


while not finished:
   #self localizing
   est_pose = [0,0,0]
   
   #Hvis den ikke har besøgt landmark 1. 
   while not status1:
      sleep(1)
      print("status 1: ", status1)
      #Tager et billede og finder dist til tætteste object med id 1
      temp_frame = cam.get_next_frame()
      corners, ids, rejected = cv2.aruco.detectMarkers(temp_frame, dict)
      print(ids)
      print("HER: ", corners)
      if corners:
         corners, ids = actions.get_corners_ids(1, corners, ids)
         dist_mm, ang, sign = actions.detector(corners, markerLength, camera_matrix, dist_coeffs)
         actions.turn_degrees(ang, sign) #Drejer hen imod objected
         print("DISTDISTDIST",  dist_mm)
         #Gør klar til at køre imod objected
         safety_dist = 0.20
         start_time = time.perf_counter()
         time_cap = 2.235 * ( float(dist_mm*0.001) - safety_dist)
         
         #Kører imod obejcted og tjekker hele tiden sensor
         print("DRIVING")
         arlo.go_diff(69, 70, 1, 1)
         while True:
            if (float(time.perf_counter()) - float(start_time)) > time_cap:
               arlo.stop()
               break
            if not (arlo.read_front_ping_sensor() >= safety_dist+5 
               and arlo.read_left_ping_sensor() >= safety_dist+5 
               and arlo.read_right_ping_sensor() >= safety_dist+5):
               arlo.stop()
               break
         
         #Check if it is close to id 1
         actions.backward_m(0.3)
         status1 = am_i_close(cam, 1)
         
         visited_landmarks.append(1)
         #If it is not close enough?? Then what? Maybe turn 30 degrees, 
         # drive 15cm and turn 30 degrees back? As if it is not close enough,
         # it must be because a object is close to the side sensors. 


   while not status2:
      
      print("status 2: ", status2)
      current_id = 2
      particles = sls.initialize_particles(NUM_PARTICLES)
      est_pose, particles = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0] - est_pose[0]
      delta_y = landmarks[current_id][1] - est_pose[1]
      dist = np.sqrt(delta_x**2 + delta_y**2)
      theta = 0 #vi forventer at vi kigger op kassen
      actions.drive_to_object(dist, 0, 1)
      status2 = am_i_close(cam,current_id)
      visited_landmarks.append(current_id)

   
   while not status3:
      current_id = 3
      particles = sls.initialize_particles(NUM_PARTICLES)
      est_pose, particles = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0] - est_pose[0]
      delta_y = landmarks[current_id][1] - est_pose[1]
      dist = np.sqrt(delta_x**2 + delta_y**2)
      theta = 0 #vi forventer at vi kigger op kassen
      actions.drive_to_object(dist, 0, 1)
      status2 = am_i_close(cam,current_id)
      visited_landmarks.append(current_id)

   
   while not status4:
      current_id = 4
      particles = sls.initialize_particles(NUM_PARTICLES)
      est_pose, particles = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0] - est_pose[0]
      delta_y = landmarks[current_id][1] - est_pose[1]
      dist = np.sqrt(delta_x**2 + delta_y**2)
      theta = 0 #vi forventer at vi kigger op kassen
      actions.drive_to_object(dist, 0, 1)
      status2 = am_i_close(cam,current_id)
      visited_landmarks.append(current_id)

   
   while not status5:
      current_id = 1
      particles = sls.initialize_particles(NUM_PARTICLES)
      est_pose, particles = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0] - est_pose[0]
      delta_y = landmarks[current_id][1] - est_pose[1]
      dist = np.sqrt(delta_x**2 + delta_y**2)
      theta = 0 #vi forventer at vi kigger op kassen
      actions.drive_to_object(dist, 0, 1)
      status2 = am_i_close(cam,current_id)
      visited_landmarks.append(current_id)

       

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


