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
arlo = robot.Robot()

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
finished = False


while not finished:
   visited_landmarks =  []
   scanned_landmarks = []
   #self localizing
   est_pose = [0,0,0]
   while not status1:
      NUM_PARTICLES = 20000
      particles = NUM_PARTICLES
      pose, objects = find_pose()
      if pose:
         status = True
         est_pose = pose
      else: 
         drive_random()

   while not status2:
      


   if visited_landmarks == landmarks_required:
      finished = True
