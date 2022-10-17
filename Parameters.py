
import cv2
from time import sleep, perf_counter
from threading import Thread
import time
import numpy as np

#Pipline
def gstreamer_pipeline(capture_width=1280, capture_height=720, framerate=30):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc !"
        "videobox autocrop=true !"
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (
            capture_width,
            capture_height,
            framerate,
        )
    )
	
def params():
#	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
	Dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	camera_matrix = np.array([ 1120, 0.0,1280/2, 0, 1120, 720/2, 0, 0, 1]).reshape(3,3)
	dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape(5,1)
	markerLength = 150
	return Dict, camera_matrix, dist_coeffs, markerLength	
