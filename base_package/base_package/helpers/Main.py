import cv2
import numpy as np
import pandas as pd
from Aruco import *


if __name__ == "__main__":

    CV_CAP_PROP_FRAME_WIDTH = 3
    CV_CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_FPS = 5

    cap = cv2.VideoCapture(0)                     #Change
    cap.set(CV_CAP_PROP_FRAME_WIDTH,3840)
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2160)
    cap.set(CAP_PROP_FPS, 60)
    while cap.isOpened():
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        robot_id = 1
        puck_id = 2 
        ids = [robot_id,puck_id]
        pose = get_pose(frame, ids)
        print("The pose of the robot is : ", pose[1])
    
    cap.release()
    cv2.destroyAllWindows()




