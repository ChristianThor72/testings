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
particles = sls.initialize_particles(NUM_PARTICLES)

landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),
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
         safety_dist = 0.2
         start_time = time.perf_counter()
         time_cap = 2.235 * ( float(dist_mm*0.001) - safety_dist)
         
         #Kører imod obejcted og tjekker hele tiden sensor
         print("DRIVING")
         arlo.go_diff(69, 70, 1, 1)
         while True:
            if (float(time.perf_counter()) - float(start_time)) > time_cap:
               arlo.stop()
               break
            if not (arlo.read_front_ping_sensor() >= safety_dist*1000+5 
               and arlo.read_left_ping_sensor() >= safety_dist*1000+5 
               and arlo.read_right_ping_sensor() >= safety_dist**1000+5):
               arlo.stop()
               break
         
         #Check if it is close to id 1
         sleep(2.5)
         actions.backward_m(0.7)
         sleep(2)
         status1 = am_i_close(cam, 1)
         current_id = 1
         if status1:
            est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
            visited_landmarks.append(1)
         #If it is not close enough?? Then what? Maybe turn 30 degrees, 
         # drive 15cm and turn 30 degrees back? As if it is not close enough,
         # it must be because a object is close to the side sensors. 


   while not status2:
      
      print("status 2: ", status2)
      current_id = 2
      particles = sls.initialize_particles(NUM_PARTICLES)

      est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0]*10 - est_pose[0]*10
      delta_y = landmarks[current_id][1]*10 - est_pose[1]*10
      dist_mm = np.sqrt((delta_x**2) + (delta_y**2))
      theta = 0 #vi forventer at vi kigger op kassen
 #     actions.drive_to_object(dist, 0, 1)
      safety_dist = 0.2
      start_time = time.perf_counter()
      final_dist = max(dist_mm, dist_cam)
      
      time_cap = 2.235 * ( float(final_dist*0.001) - safety_dist)
      print("Dette er distancen: ", final_dist)
      
      #Kører imod obejcted og tjekker hele tiden sensor
      print("DRIVING")
      arlo.go_diff(69, 70, 1, 1)
      while True:
         if (float(time.perf_counter()) - float(start_time)) > time_cap:
            arlo.stop()
            break
         if not (arlo.read_front_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_left_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_right_ping_sensor() >= safety_dist**1000+5):
            arlo.stop()
            break
      
      #Check if it is close to id 2
      sleep(2.5)
      actions.backward_m(0.7)
      sleep(2)
      status2 = am_i_close(cam, 2)
      if not status2:
          sleep(2.5)
          actions.backward_m(0.25)
          sleep(2)
          status2 = am_i_close(cam, 2)  
      
      current_id = 2
      if status2:
         est_pose, particles, cam_dists = find_pose(particles, cam, current_id)
         visited_landmarks.append(2)


   
   while not status3:
          
      print("status 3: ", status3)
      current_id = 3
      particles = sls.initialize_particles(NUM_PARTICLES)
      est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0]*10 - est_pose[0]*10
      delta_y = landmarks[current_id][1]*10 - est_pose[1]*10
      dist_mm = np.sqrt((delta_x**2) + (delta_y**2))
      theta = 0 #vi forventer at vi kigger op kassen
 #     actions.drive_to_object(dist, 0, 1)
      safety_dist = 0.2
      start_time = time.perf_counter()
      final_dist = max(dist_mm, dist_cam)
      
      time_cap = 2.235 * ( float(final_dist*0.001) - safety_dist)
      print("Dette er distancen: ", final_dist)
      
      #Kører imod obejcted og tjekker hele tiden sensor
      print("DRIVING")
      arlo.go_diff(69, 70, 1, 1)
      while True:
         if (float(time.perf_counter()) - float(start_time)) > time_cap:
            arlo.stop()
            break
         if not (arlo.read_front_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_left_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_right_ping_sensor() >= safety_dist**1000+5):
            arlo.stop()
            break
      
      #Check if it is close to id 2
      sleep(2.5)
      actions.backward_m(0.7)
      sleep(2)
      status3 = am_i_close(cam, 3)
      current_id = 3
      if not status3:
          sleep(2.5)
          actions.backward_m(0.25)
          sleep(2)
          status3 = am_i_close(cam, 3) 
      
      if status3:
         est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
         visited_landmarks.append(3)


   while not status4:
      print("status 4: ", status4)
      current_id = 4
      particles = sls.initialize_particles(NUM_PARTICLES)
      est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0]*10 - est_pose[0]*10
      delta_y = landmarks[current_id][1]*10 - est_pose[1]*10
      dist_mm = np.sqrt((delta_x**2) + (delta_y**2))
      theta = 0 #vi forventer at vi kigger op kassen
 #     actions.drive_to_object(dist, 0, 1)
      safety_dist = 0.2
      start_time = time.perf_counter()
      final_dist = max(dist_mm, dist_cam)
      
      time_cap = 2.235 * ( float(final_dist*0.001) - safety_dist)
      print("Dette er distancen: ", final_dist)
      
      #Kører imod obejcted og tjekker hele tiden sensor
      print("DRIVING")
      arlo.go_diff(69, 70, 1, 1)
      while True:
         if (float(time.perf_counter()) - float(start_time)) > time_cap:
            arlo.stop()
            break
         if not (arlo.read_front_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_left_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_right_ping_sensor() >= safety_dist**1000+5):
            arlo.stop()
            break
      
      #Check if it is close to id 2
      sleep(2.5)
      actions.backward_m(0.7)
      sleep(2)
      status4 = am_i_close(cam, 4)
      current_id = 4
      if not status4:
          sleep(2.5)
          actions.backward_m(0.25)
          sleep(2)
          status4 = am_i_close(cam, 4) 
      if status4:
         est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
         visited_landmarks.append(4)

   
   while not status5:
      print("status 5: ", status5)
      current_id = 1
      particles = sls.initialize_particles(NUM_PARTICLES)
      est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
      
      delta_x = landmarks[current_id][0]*10 - est_pose[0]*10
      delta_y = landmarks[current_id][1]*10 - est_pose[1]*10
      dist_mm = np.sqrt((delta_x**2) + (delta_y**2))
      theta = 0 #vi forventer at vi kigger op kassen
 #     actions.drive_to_object(dist, 0, 1)
      safety_dist = 0.2
      start_time = time.perf_counter()
      final_dist = max(dist_mm, dist_cam)
      
      time_cap = 2.235 * ( float(final_dist*0.001) - safety_dist)
      print("Dette er distancen: ", final_dist)
      
      #Kører imod obejcted og tjekker hele tiden sensor
      print("DRIVING")
      arlo.go_diff(69, 70, 1, 1)
      while True:
         if (float(time.perf_counter()) - float(start_time)) > time_cap:
            arlo.stop()
            break
         if not (arlo.read_front_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_left_ping_sensor() >= safety_dist*1000+5 
            and arlo.read_right_ping_sensor() >= safety_dist**1000+5):
            arlo.stop()
            break
      
      #Check if it is close to id 2
      sleep(2.5)
      actions.backward_m(0.7)
      sleep(2)
      status5 = am_i_close(cam, 1)
      current_id = 1
      if not status5:
          sleep(2.5)
          actions.backward_m(0.25)
          sleep(2)
          status5 = am_i_close(cam, 1) 
          
      if status5:
         est_pose, particles, dist_cam = find_pose(particles, cam, current_id)
         visited_landmarks.append(1)

       

   if len(visited_landmarks) == 5:
      finished = True
      
print("We did it!!")




