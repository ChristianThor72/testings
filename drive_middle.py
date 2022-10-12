import numpy as np
import cv2
import actions
import robot
import particle
import Self_localization_slow as sls
from time import sleep
from camera import Camera
from particle import move_particle
arlo = robot.Robot()

#Initilize pipline for cam and spawn cam in seperat thread
#cam_imp.gstreamer_pipeline()
cam = Camera(0, robottype = 'arlo', useCaptureThread = True)
sleep(1)

NUM_PARTICLES = 5000
particles = sls.initialize_particles(NUM_PARTICLES)

#Defining the aruco dict
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)


#Finding the theta needed to turn direction toward middle of boxes and the dist to drive. 
def driving_strat(middle, pose):
    delta_y = middle[1]-pose[1]
    delta_x = middle[0]-pose[0]
    dist = np.sqrt(delta_x**2 + delta_y**2)
    theta = np.arccos(delta_y/dist) 
    theta_new = (pose[2]-theta)*180/np.pi
    return theta_new, dist


def find_pose(particles):
    #List of found Id's aka boxes
    id_lst = []
    
    #Making list for the temperary particle poses. 
    parties_lst = []
    while len(id_lst) < 2:
        frameReference = cam.get_next_frame() # Read frame
        corners, ids, _ = cv2.aruco.detectMarkers(frameReference, dict)
        cv2.aruco.drawDetectedMarkers(frameReference,corners)
        
        #Scanning for object
        if not corners or ids in id_lst:
            actions.scan_for_object(cam, dict)
            move_particle(particles, 0,0, -0.349)
            particle.add_uncertainty(particles, 0.0, 0.05)
            sleep(0.5)
        
        # Checking if any object found
        if corners:
            
            # Checking if the object found is the first obejct found
            if len(id_lst) == 0:
                
                #Making sure we are not just seeing the same object again. If we do, nothing happens, and it will start scanning again
                if ids not in id_lst: 
                    id_lst.append(ids)
                    
                    #Calling selflocate to get some poses ready for when second object is found
                    _, _, _, parties = sls.self_locate(cam, frameReference, particles)
                    # Saving poses
                    #parties_lst.append(parties)
                    particles = parties
                    
            # If the object found is not the first obejct found, then use this.            
            elif len(id_lst) == 1:
                
                #Making sure we are not just seeing the same object again. If we do, nothing happens, and it will start scanning again
                if ids not in id_lst:
                    id_lst.append(ids)
                    
                    #Using poses from when we saw the object before. Now we should have a good pose.
                    theta, x, y, parties = sls.self_locate(cam, frameReference, particles)  
                    pose = [x, y, theta]
    
    # Return the best estimated pose
    return pose

#Find pose
pose = find_pose(particles)

# Finding theta to correct turn direction and dist to drive
theta_corr, dist = driving_strat([150,0], pose)

#Making theta ready for actions.turn as it need an angle and a sign.
sign, theta = np.sign(theta_corr), np.abs(theta_corr)

#Make the robot turn towards middle
print(theta_corr)
actions.turn_degrees(theta, sign)
move_particle(particles, 0,0, theta_corr)
particle.add_uncertainty(particles, 0.0, 0.05)#eventuel fortegnsfejl
#Make the robot drive into the middle. The unit for dist should be in mm. Think the output for driving strat might be in meters.. :/
print(dist*10)
actions.forward_mm(dist*10)

#Make sure to stop the robot.
sleep(1)
arlo.stop()



























