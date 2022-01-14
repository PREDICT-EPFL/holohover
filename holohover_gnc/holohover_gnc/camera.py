# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from holohover_msgs.msg import Pose
import os
import cv2
import pandas as pd
import numpy as np
from holohover_gnc.helpers.Aruco import get_pose, four_point_transform
import time


class Camera(Node):

    def __init__(self):
        #path = '/install/holohover_gnc/lib/python3.8/site-packages/holohover_gnc/'
        FILENAME = os.path.join(os.path.dirname(__file__),'refPt.csv')
        super().__init__('Camera')
        self.publisher_1 = self.create_publisher(Pose, '/camera/robot_pose', 10)
        timer_period = 1 / 60 # seconds
        self.timer = self.create_timer(timer_period, self.pose_callback)
        self.publisher_2 = self.create_publisher(Pose, '/camera/puck_pose', 10)
        
        
        self.get_logger().info('Launching video capture')
        self.holohover_ID = 1
        self.puck_ID = 2
        self.cap = cv2.VideoCapture(2)                     #Change if needed
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))               #Change if needed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.ref_Pt = np.array(pd.read_csv(FILENAME, header=None))

    def __del__(self):
        self.cap.release()

    def pose_callback(self):
        if self.cap.isOpened():
            t0 = time.time()
            ret, frame = self.cap.read()
            #print('Processing time is {}'.format(time.time()-t0))
            # if frame is read correctly ret is True
            if not ret:
                self.get_logger().error('Frame capture failed')
                return
            
            #[TODO] Add image cropping
            M, cropped = four_point_transform(frame, self.ref_Pt)
            

            ids = [self.holohover_ID, self.puck_ID]
            pose = get_pose(cropped, ids)
            holohover_idx = np.where(pose == self.holohover_ID)[0][0]
            puck_idx = np.where(pose == self.puck_ID)[0][0]

            if pose[holohover_idx][1] != -999:
                msg = Pose()
                msg.x = float(pose[holohover_idx][1])
                msg.y = float(pose[holohover_idx][2])
                msg.yaw = float(pose[holohover_idx][3])
                self.publisher_1.publish(msg)

            if pose[puck_idx][1] != -999:
                msg = Pose()
                msg.x = float(pose[puck_idx][1])
                msg.y = float(pose[puck_idx][2])
                msg.yaw = float(pose[puck_idx][3])
                self.publisher_2.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    camera = Camera()

    rclpy.spin(camera)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
