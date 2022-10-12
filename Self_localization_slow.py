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


# Flags
showGUI = True  # Whether or not to open GUI windows
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




# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 2, 3, 4, 5]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (300.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks





def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarks)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
                                 ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)



def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles

def resample_particles(particles, weights):
    temp = random.choices(particles, weights, k = len(particles))
    for i in range(len(temp)):
        particles[i] = copy.copy(temp[i])
    return particles

def draw_aruco_objects(ids, img, corners, intrinsic_matrix, distortion_coeffs, rvecs, tvecs, arucoMarkerLength):
    """Draws detected objects and their orientations on the image given in img."""
    if not isinstance(ids, type(None)):
        outimg = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        for i in range(ids.shape[0]):
            outimg = cv2.drawFrameAxes(outimg, intrinsic_matrix, distortion_coeffs,
                                       rvecs[i], tvecs[i], arucoMarkerLength)
    else:
        outimg = img

    return outimg


# Main program #
def self_locate(cam, frameReference, init_poses = []):
    
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
            particles = initialize_particles(num_particles)
    
        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
    
        # Driving parameters
        velocity = 44.74 # cm/sec
        angular_velocity = 2.33 # radians/sec
    
        # Initialize the robot (XXX: You do this)
        #arlo = robot.Robot()
        
        # Allocate space for world map
        world = np.zeros((500,500,3), dtype=np.uint8)
    
        # Draw map
        draw_world(est_pose, particles, world)
        
        
        print("Opening and initializing camera")
        """
        if camera.isRunningOnArlo():
            cam = camera.Camera(0, 'arlo', useCaptureThread = True)
        else:
            cam = camera.Camera(0, 'macbookpro', useCaptureThread = True)
        """
        count = 0 
        while count < 10:
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
                sigma = 5
                sigma_theta = 0.05
                sum_of_weights = 0
                #print(objectIDs[0])
                box_x = landmarks[int(objectIDs[0])][0] #x koordinat for kassen der er observeret
                box_y = landmarks[int(objectIDs[0])][1]#y koordinat for kassen der er observeret
                dist = dists[0] #distance kassen er observeret fra
                box_theta = angles[0]

                for elm in (particles):
                    delta_x, delta_y =  box_x - elm.getX(), box_y - elm.getY() #forskellen på partikel og koordinat for den observerede kasse   
                    dist_from_particle_to_box = math.sqrt(pow(delta_x,2) + pow(delta_y, 2)) #distancen fra partiklen til kassen 
                    potens = ((pow( (dist_from_particle_to_box - dist), 2 ))/(2 * (pow(sigma, 2))))
                    weight_dist = np.exp(-potens) #regner vægten

                    
                    theta_corr = np.arccos(delta_x/dist_from_particle_to_box) 
                    if delta_y < 0:
                        theta_corr = 2*np.pi - theta_corr 
                    
                    #print(elm.getTheta(), box_theta)
                    delta_theta =  elm.getTheta() - theta_corr 
    
    
                    potens_theta = ((pow( (delta_theta), 2 ))/(2 * (pow(sigma_theta, 2))))
                    weight_theta = np.exp(-potens_theta)
                    
                        
                    #print(delta_theta)
                    
                    weight = weight_dist * weight_theta
                    
                    elm.setWeight(weight)
                    sum_of_weights += weight
                probabilities = []    
                
                for elm in particles:
                    probabilities.append(elm.getWeight()/sum_of_weights)
                    elm.setWeight(elm.getWeight()/sum_of_weights)
                    #print("probability: ", elm.getWeight())
                cam.draw_aruco_objects(frameReference)
                
                # Resampling
                # XXX: You do this
                print("resampling....")
                particles = resample_particles(particles, probabilities)
                time.sleep(0.5)

                
                #adding noise
                #particle.add_uncertainty(particles, 0.1, 0.1)

                # Draw detected objects
            else:
                # No observation - reset weights to uniform distribution
                for p in particles:
                    p.setWeight(1.0/num_particles)
            
            particle.add_uncertainty(particles, 0.1, 0.01)
        
            est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
    
            if showGUI:
                # Draw map
                draw_world(est_pose, particles, world)
        
                # Show frame
                cv2.imshow(WIN_RF1, frameReference)
    
                # Show world
                cv2.imshow(WIN_World, world)
        
            

        return est_pose.getTheta(), est_pose.getX(), est_pose.getY(), particles
        
        
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()
    
        # Clean-up capture thread
        #cam.terminateCaptureThread()

   
#theta, x, y, parti = self_locate()
#print(theta, x, y)



