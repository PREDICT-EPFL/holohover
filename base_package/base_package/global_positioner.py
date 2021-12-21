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
from std_msgs.msg import String
from holohover_msgs.msg import Positions
import os
import cv2
import pandas as pd
import numpy as np
from .helpers.Aruco import get_pose, four_point_transform
import time


class GlobalPositioner(Node):

    def __init__(self):
        #path = '/install/base_package/lib/python3.8/site-packages/base_package/'
        FILENAME = os.path.join(os.path.dirname(__file__),'refPt.csv')
        super().__init__('tracking_system')
        self.publisher_ = self.create_publisher(Positions, 'global_position', 10)
        timer_period = 1 / 50 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        print('[INFO] Launching video capture')
        CV_CAP_PROP_FRAME_WIDTH = 3
        CV_CAP_PROP_FRAME_HEIGHT = 4
        CAP_PROP_FPS = 5
        global holohover_ID, puck_ID
        holohover_ID = 1
        puck_ID = 2
        global cap
        cap = cv2.VideoCapture(0)                     #Change if needed
        cap.set(CAP_PROP_FPS, 60)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))               #Change if needed
        cap.set(CV_CAP_PROP_FRAME_WIDTH,1920)
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080)
        self.ref_Pt = np.array(pd.read_csv(FILENAME, header=None))

    def __del__(self):
        cap.release()

    def timer_callback(self):
        if cap.isOpened():
            t0 = time.time()
            ret, frame = cap.read()
            #print('Processing time is {}'.format(time.time()-t0))
            # if frame is read correctly ret is True
            if not ret:
                print("[FAIL] Frame captuer failed")
                return
            
            #[TODO] Add image cropping
            M, cropped = four_point_transform(frame, self.ref_Pt)
            

            ids = [holohover_ID,puck_ID]
            pose = get_pose(cropped, ids)
            holohover_idx = np.where(pose == holohover_ID)[0][0]
            puck_idx = np.where(pose == puck_ID)[0][0]

            msg = Positions()
            msg.holohover.x = float(pose[holohover_idx][1])
            msg.holohover.y = float(pose[holohover_idx][2])
            msg.holohover.yaw = float(pose[holohover_idx][3])
            msg.puck.x = float(pose[puck_idx][1])
            msg.puck.y = float(pose[puck_idx][2])
            msg.puck.yaw = float(pose[puck_idx][3])
            self.publisher_.publish(msg)
            self.i += 1


def main(args=None):
    rclpy.init(args=args)

    tracking_system = GlobalPositioner()

    rclpy.spin(tracking_system)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracking_system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
