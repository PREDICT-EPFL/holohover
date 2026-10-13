from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import pandas as pd
import math
import os

def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()    

        if topic == "/optitrack/drone/pose":           
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
        if topic == "/drone/control":           
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
        if topic == "/drone/imu":           
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
        if topic == "/bus0/ft_sensor0/ft_sensor_readings/wrench":           
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
    del reader

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

def main():
    serie = "holohover_20230404"
    exp = "rosbag2_2023_04_03-13_10_11"
    file = exp + "_0.mcap"

    # loop through all files of all subfolders of the defined experiment series
    for root, dirs, files in os.walk(serie):
        for f in files:

            # skip file if it is not a .mcap file
            if f.find(".mcap") == -1:
                continue
            print(f"Converting {os.path.join(root, f)}")

            df_optitrack = pd.DataFrame({ "time":pd.Series(dtype='int'), "x":pd.Series(dtype='float'), "y":pd.Series(dtype='float'), 
                                            "theta":pd.Series(dtype='float')})
            df_control = pd.DataFrame({ "time":pd.Series(dtype='int'), "motor_a_1":pd.Series(dtype='float'), "motor_a_2":pd.Series(dtype='float'), 
                                        "motor_b_1":pd.Series(dtype='float'), "motor_b_2":pd.Series(dtype='float'), "motor_c_1":pd.Series(dtype='float'), 
                                        "motor_c_2":pd.Series(dtype='float') })
            df_imu = pd.DataFrame({ "time":pd.Series(dtype='int'), "ddx":pd.Series(dtype='float'), "ddy":pd.Series(dtype='float'), 
                                    "ddtheta":pd.Series(dtype='float') })
            df_thrust = pd.DataFrame({ "time":pd.Series(dtype='int'), "Fx":pd.Series(dtype='float'), "Fy":pd.Series(dtype='float'), 
                                        "Fz":pd.Series(dtype='float'), "Mx":pd.Series(dtype='float'), "My":pd.Series(dtype='float'),
                                        "Mz":pd.Series(dtype='float') })
            
            #counter = 0

            for topic, msg, timestamp in read_messages(os.path.join(root, f)):
                #print(f"{topic} ({type(msg).__name__}) [{timestamp}]: '{msg}'")
                
                if topic == "/optitrack/drone/pose":
                    roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
                    df_optitrack.loc[len(df_optitrack)] = [timestamp, msg.pose.position.x, msg.pose.position.y, yaw]
                elif topic == "/drone/control":
                    df_control.loc[len(df_control)] = [timestamp, msg.motor_a_1, msg.motor_a_2, msg.motor_b_1, msg.motor_b_2, msg.motor_c_1, msg.motor_c_2]
                elif topic == "/drone/imu":
                    df_imu.loc[len(df_imu)] = [timestamp, msg.acc.x, msg.acc.y, msg.gyro.z]
                elif topic == "/bus0/ft_sensor0/ft_sensor_readings/wrench": 
                    df_thrust.loc[len(df_thrust)] = [timestamp, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x,
                                                        msg.wrench.torque.y, msg.wrench.torque.z]

                """counter += 1
                if counter >= 200:
                    break"""

            print(df_thrust.head(5))
            print(df_control.head(5))
            df_optitrack.to_csv(os.path.join(root, f[:-7]+"_optitrack.csv"))
            df_control.to_csv(os.path.join(root, f[:-7]+"_control.csv"))
            df_imu.to_csv(os.path.join(root, f[:-7]+"_imu.csv"))
            df_thrust.to_csv(os.path.join(root, f[:-7]+"_thrust.csv"))

if __name__ == "__main__":
    main()
