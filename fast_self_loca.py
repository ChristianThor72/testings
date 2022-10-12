# -*- coding: utf-8 -*-
"""
Created on Wed Oct  5 19:14:27 2022

@author: flemm
"""

import numpy as np
import cv2
import camera
from camera import Camera
import random
from time import sleep

showGUI = True  # Whether or not to open GUI windows
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
landmarkIDs = [1, 2]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (300.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks



# k is number of particles
def create_particles(k):
    particles = np.random.uniform(low  = (-100,-250,0,0), 
                                  high = (500, 350, 2*np.pi, 1/k), 
                                  size = (k, 4))
    return particles

def estimate_pose(particles):
    estimated_pose = np.array( [ np.mean(particles[:,0]) , np.mean(particles[:,1]), np.mean(particles[:,2]), np.mean(particles[:,3]) ] )    
    return estimated_pose

def resample_particles(particles, weights):
    return random.choices(particles, weights, k = len(particles))

def add_uncertainty_np(particles_list, sigma, sigma_theta):
    particles_list = np.array(particles_list)
    particles_list[:,0] += np.random.uniform(-sigma, sigma, len(particles_list))
    particles_list[:,1] += np.random.uniform(-sigma, sigma, len(particles_list))
    particles_list[:,2] += np.random.uniform(-sigma_theta, sigma_theta, len(particles_list))
    return particles_list

def jet(x):
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)
    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    X, Y, theta, weight = est_pose
    particles = np.array(particles)
    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = max(particles[:,3])

    # Draw particles
    for particle in particles:
        x = int(particle[0] + offsetX)
        y = ymax - (int(particle[1] + offsetY))
        colour = jet(particle[3] / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle[0] + 15.0*np.cos(particle[2]))+offsetX, 
             ymax - (int(particle[1] + 15.0*np.sin(particle[2]))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)


    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(X)+offsetX, ymax-(int(Y)+offsetY))
    b = (int(X + 15.0*np.cos(theta))+offsetX, 
                                 ymax-(int(Y + 15.0*np.sin(theta))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)


# Main program #
def self_locate(init_poses = []):
    
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
        num_particles = 5000
        particles = create_particles(num_particles)
        
    
        est_pose = estimate_pose(particles) # The estimate of the robots current pose
    
        # Allocate space for world map
        world = np.zeros((500,500,3), dtype=np.uint8)
    
        # Draw map
        draw_world(est_pose, particles, world)
    
    
        print("Opening and initializing camera")
        cam = camera.Camera(0, 'macbookpro', useCaptureThread = True)
        count = 0 
        print(count)
        while True:
            # Move the robot according to user input (only for testing)
            action = cv2.waitKey(10)
            if action == ord('q'): # Quit
                break
        
            # Fetch next frame
            colour = cam.get_next_frame()
            
            # Detect objects
            objectIDs, dists, angles = cam.detect_aruco_objects(colour)
            if not isinstance(objectIDs, type(None)):
                print(count)
                count += 1
                # List detected objects
                for i in range(len(objectIDs)):
                    print("Object ID = ", objectIDs[i], i, ", Distance = ", dists[i], ", angle = ", angles[i])
                    # XXX: Do something for each detected object - remember, the same ID may appear several times
    
                # Compute particle weights
                # XXX: You do this
                
                print("ny omgang ")
                sigma = 5
                sigma_theta = 0.3
                sum_of_weights = 0
                #print(objectIDs[0])
                box_x = landmarks[objectIDs[0]][0] #x koordinat for kassen der er observeret
                box_y = landmarks[objectIDs[0]][1]#y koordinat for kassen der er observeret
                dist = dists[0] #distance kassen er observeret fra
                box_theta = angles[0]

                particles = np.array(particles)
                delta_x, delta_y =  box_x - particles[:,0], box_y - particles[:,1] #forskellen på partikel og koordinat for den observerede kasse   
                dist_from_particle_to_box = np.sqrt(pow(delta_x,2) + pow(delta_y, 2)) #distancen fra partiklen til kassen 
                potens = ((pow( (dist_from_particle_to_box - dist), 2 ))/(2 * (pow(sigma, 2))))
                weight_dist = np.exp(-potens) #regner vægten

                

                theta_corr = np.arccos(delta_x/dist_from_particle_to_box) 
                theta_corr[delta_y < 0] = 2*np.pi - theta_corr[delta_y < 0]             
                                      
                delta_theta =  particles[:,2] - theta_corr 
    
    
                potens_theta = ((pow( (delta_theta), 2 ))/(2 * (pow(sigma_theta, 2))))
                weight_theta = np.exp(-potens_theta)
                    
                        
                    #print(delta_theta)
                    
                weight = weight_dist * weight_theta
                particles[:,3] = weight
                
                sum_of_weights = sum(particles[:,3])
                probabilities = particles[:,3]/sum_of_weights
                particles[:,3] = particles[:,3]/sum_of_weights
                

                cam.draw_aruco_objects(colour)
                
                # Resampling
                # XXX: You do thisQQQQQQ
                print("resampling....")
                particles = resample_particles(particles, probabilities)


                
                #adding noise
                #particles = particle.add_uncertainty(particles, 0.1, 0.1)

                # Draw detected objects
            else:
                # No observation - reset weights to uniform distribution
                particles = np.array(particles)
                particles[:, 3] = 1.0/len(particles)

            
            #particle.add_uncertainty(particles, 0.1, 0.01)
            particles = add_uncertainty_np(particles, 0.1, 0.001)
            particles = np.array(particles)
            #est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
    
            if showGUI:
                # Draw map
                draw_world(est_pose, particles, world)
        
                # Show frame
                cv2.imshow(WIN_RF1, colour)
    
                # Show world
                cv2.imshow(WIN_World, world)
        
            

        #return est_pose.getTheta(), est_pose.getX(), est_pose.getY(), particles
        
        
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()
    
        # Clean-up capture thread
        cam.terminateCaptureThread()


self_locate()
        

        
