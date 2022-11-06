# -*- coding: utf-8 -*-
"""
Created on Mon Oct  3 13:47:43 2022

@author: flemm
"""
import math
from random import gauss, choices
from urllib.robotparser import RobotFileParser
import random
import copy
import cv2
import particle
import numpy as np
import time
from timeit import default_timer as timer
#from Sampling_Importance_Resampling import calcWeight, resample
import sys
from camera import Camera
from time import sleep
num_particles = 20000

# Flags
showGUI = False  # Whether or not to open GUI windows
onRobot = True # Whether or not we are running on the Arlo robot


def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../../../../Arlo/python")


try:
    import robot
    onRobot = True
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False





# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 2, 3, 4, 5,6,7,8,9]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),
    4: (400.0, 0.0) 
}






def p(num_particles):
    return(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)

def init_particles(num_particles):
    arr = np.empty((num_particles,4))

    for i in range(len(arr)):
        arr[i] += p(num_particles)
    
    return arr






def res_parts(particles, weights):
    temp = random.choices(particles, weights, k = len(particles))
    for i in range(len(temp)):
        particles[i] = copy.copy(temp[i])
    return particles 


def add_uncertainty(particles_list, sigma, sigma_theta):
    """Add some noise to each particle in the list. Sigma and sigma_theta is the noise
    variances for position and angle noise."""
    for particle in particles_list:
        particle[0] += rn.randn(0.0,sigma)
        particle[1] += rn.randn(0.0,sigma)
        particle[2] = np.mod(particle[2] + rn.randn(0.0, sigma_theta), 2.0 * np.pi) 



def est_poses(particles_list):
    """Estimate the pose from particles by computing the average position and orientation over all particles. 
    This is not done using the particle weights, but just the sample distribution."""
    x_sum = np.sum([particles_list[:,0]])
    y_sum = np.sum([particles_list[:,1]])
    cos_sum = np.sum([particles_list[:,2]])
    sin_sum = np.sum([particles_list[:,3]])
    arr = np.empty((num_particles,4))
        
    flen = len(particles_list)
    if flen != 0:
        x = x_sum / flen
        y = y_sum / flen
        theta = np.arctan2(sin_sum/flen, cos_sum/flen)
    else:
        x = x_sum
        y = y_sum
        theta = 0.0
    return(np.array([x, y, theta, 0]))


# Main program #
def self_locate(cam, frameReference, current_id, init_poses = []):
    
    try:
        if showGUI:
            # Open windows
            WIN_RF1 = "Robot view"
            cv2.namedWindow(WIN_RF1)
            cv2.moveWindow(WIN_RF1, 50, 50)
    
            WIN_World = "World view"
            cv2.namedWindow(WIN_World)
            cv2.moveWindow(WIN_World, 500, 50)
    
    
        # Initialize particles
        num_particles = 20000
        if len(init_poses) != 0:
            particles = init_poses
        else:
            particles = init_particles(num_particles)
    
        est_pose = est_poses(particles) # The estimate of the robots current pose
    
        # Driving parameters
        velocity = 44.74 # cm/sec
        angular_velocity = 2.33 # radians/sec
    
        # Initialize the robot (XXX: You do this)
        #arlo = robot.Robot()
        
        # Allocate space for world map
        world = np.zeros((500,500,3), dtype=np.uint8)
    
        # Draw map
        #draw_world(est_pose, particles, world)
        
        
        print("Opening and initializing camera")
        """
        if camera.isRunningOnArlo():
            cam = camera.Camera(0, 'arlo', useCaptureThread = True)
        else:
            cam = camera.Camera(0, 'macbookpro', useCaptureThread = True)
        """
        count = 0 
        while count < 3:
            # Move the robot according to user input (only for testing)
            action = cv2.waitKey(10)
            if action == ord('q'): # Quit
                break
        
            if not isRunningOnArlo():
                if action == ord('w'): # Forward
                    velocity += 4.0
                elif action == ord('x'): # Backwards
                    velocity -= 4.0
                elif action == ord('s'): # Stop
                    velocity = 0.0
                    angular_velocity = 0.0
                elif action == ord('a'): # Left
                    angular_velocity += 0.2
                elif action == ord('d'): # Right
                    angular_velocity -= 0.2
  
            if showGUI:
                # Draw map
                draw_world(est_pose, particles, world)
        
                # Show frame
                cv2.imshow(WIN_RF1, frameReference)
    
                # Show world
                cv2.imshow(WIN_World, world)
        
    
    
            
            # Use motor controls to update particles
            # XXX: Make the robot drive
            # XXX: You do this
    
    
            # Fetch next frame
            #colour = cam.get_next_frame()
            
            # Detect objects
            frameReference = cam.get_next_frame()
            objectIDs, dists, angles = cam.detect_aruco_objects(frameReference)
            sleep(0.2)            
            if not isinstance(objectIDs, type(None)):
                count += 1
                # List detected objects
                #for i in range(len(objectIDs)):
                    #print("Object ID = ", objectIDs[i], i, ", Distance = ", dists[i], ", angle = ", angles[i])
                    # XXX: Do something for each detected object - remember, the same ID may appear several times
    
                # Compute particle weights
                # XXX: You do this
                
                print("ny omgang ")
                sigma = 10
                sigma_theta = 0.15
                sum_of_weights = 0
                #print(objectIDs[0])
                box_x = landmarks[current_id][0] #x koordinat for kassen der er observeret
                box_y = landmarks[current_id][1]#y koordinat for kassen der er observeret
                dist = dists[0] #distance kassen er observeret fra
                #box_theta = angles[0]

                for elm in (particles):
                    delta_x, delta_y =  box_x - elm[0], box_y - elm[1] #forskellen på partikel og koordinat for den observerede kasse   
                    dist_from_particle_to_box = math.sqrt(pow(delta_x,2) + pow(delta_y, 2)) #distancen fra partiklen til kassen 
                    potens = ((pow( (dist_from_particle_to_box - dist), 2 ))/(2 * (pow(sigma, 2))))
                    weight_dist = np.exp(-potens) #regner vægten

                    
                    theta_corr = np.arccos(delta_x/dist_from_particle_to_box) 
                    if delta_y < 0:
                        theta_corr = 2*np.pi - theta_corr 
                    
                    #print(elm.getTheta(), box_theta)
                    delta_theta =  elm[3] - theta_corr 
    
    
                    potens_theta = ((pow( (delta_theta), 2 ))/(2 * (pow(sigma_theta, 2))))
                    weight_theta = np.exp(-potens_theta)
                    
                        
                    #print(delta_theta)
                    
                    weight = weight_dist * weight_theta
                    
                    elm[3] = weight
                    #elm.setWeight(weight)
                    sum_of_weights += weight

                probabilities = []    
                
                for elm in particles:
                    probabilities.append(elm[3]/sum_of_weights)
                    elm[3] = (elm[3]/sum_of_weights)
                    #elm.setWeight(elm.getWeight()/sum_of_weights)
                    #print("probability: ", elm.getWeight())
                cam.draw_aruco_objects(frameReference)
                
                # Resampling
                # XXX: You do this
                print("resampling....")
                particles = res_parts(particles, probabilities)
                time.sleep(0.5)

                
                #adding noise
                add_uncertainty(particles, 1.5, 0.1)

                # Draw detected objects
            else:
                # No observation - reset weights to uniform distribution
                for p in particles:
                    p[3] = (1.0/num_particles)
            
            add_uncertainty(particles, 0.1, 0.01)
        
            est_pose = est_poses(particles) # The estimate of the robots current pose
    
            if showGUI:
                # Draw map
                draw_world(est_poses, particles, world)
        
                # Show frame
                cv2.imshow(WIN_RF1, frameReference)
    
                # Show world
                cv2.imshow(WIN_World, world)
        
            

        return est_poses[2],est_poses[0], est_poses[1], particles
        
    
        #est_pose.getTheta(), est_pose.getX(), est_pose.getY(), particles
        
        
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()
    
        # Clean-up capture thread
        #cam.terminateCaptureThread()

   
#theta, x, y, parti = self_locate()
#print(theta, x, y)



