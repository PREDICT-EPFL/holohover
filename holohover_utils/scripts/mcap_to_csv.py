# Copyright (c) 2025 EPFL
#
# This source code is licensed under the BSD 2-Clause License found in the
# LICENSE file in the root directory of this source tree.

# pip3 install pandas mcap-ros2-support
import os
import sys
import math
import pandas as pd
from collections import defaultdict
from mcap_ros2.reader import read_ros2_messages

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def read_mcap_file(path, topics=None):
    def get_attributes(obj):
        return filter(lambda a: not a.startswith('_'), dir(obj))

    def msg_to_dic(msg):
        dic = {'log_time_ns': msg.log_time_ns}
        
        def recursive_traversal(prefix, obj, depth):
            if depth > 5: return
            for attr in get_attributes(obj):

                field = getattr(obj, attr)
                
                if prefix == '':
                    name = attr
                else:
                    name = f'{prefix}.{attr}'

                if type(field).__name__ == 'Quaternion':
                    roll, pitch, yaw = euler_from_quaternion(field.x, field.y, field.z, field.w)
                    dic[f'{name}.roll'] = roll
                    dic[f'{name}.pitch'] = pitch
                    dic[f'{name}.yaw'] = yaw

                if type(field) in [bool, int, float, str]:
                    dic[name] = field
                elif type(field) not in [list]:
                    recursive_traversal(name, field, depth + 1)
                    
        recursive_traversal('', msg.ros_msg, 0)
        return dic

    topic_msgs = defaultdict(list)
    for msg in read_ros2_messages(path, topics):
        topic_msgs[msg.channel.topic].append(msg_to_dic(msg))

    return topic_msgs

def main():

    # maps topic to csv file
    topic_mapping = None
    # topic_mapping = {
    #     '/car/state': 'state',
    #     '/car/set/control': 'control',
    # }

    topic_msgs = read_mcap_file(sys.argv[1], topic_mapping.keys() if topic_mapping is not None else None)

    if topic_mapping is None:
        class TopicToFile:
            def __getitem__(self, topic):
                return topic.strip('/').replace('/', '_')
        topic_mapping = TopicToFile()

    dir = os.path.dirname(sys.argv[1])

    for topic, msgs in topic_msgs.items():
        df = pd.DataFrame(msgs)
        df.to_csv(os.path.join(dir, f'{topic_mapping[topic]}.csv'), index=False)

if __name__ == '__main__':
    main()
