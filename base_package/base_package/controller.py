from numpy.core.fromnumeric import shape
import rclpy
from rclpy.node import Node, NodeNameNonExistentError
from holohover_msgs.msg import MotorControl, DroneMeasurement, Pose, DroneState, MotorControl
from std_msgs.msg import String
import numpy as np
from .helpers.Holohover import Holohover
from .helpers.Sensor import Sensor
from .helpers.Helpers import *
from .helpers.PID import PID
import scipy
from numpy import DataSource, array, dot
from qpsolvers import solve_qp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt
import time

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')
        # Subscriber
        self.subscription = self.create_subscription(DroneState,'estimator/state',self.control_callback,10)
        self.subscription  # prevent unused variable warning
        
        # Publisher
        self.publisher = self.create_publisher(MotorControl, '/drone/motor_control', 10)

        # Controllers
        self.PID_vx    = PID(Kp=2, Kd=0, Ki=0, ref=2, limit=0.1)
        self.PID_vy    = PID(Kp=2, Kd=0, Ki=0, ref=-1, limit=0.1)
        self.PID_yaw_d = PID(Kp=2, Kd=0, Ki=0, ref=0.2, limit=5)
        self.u = None
        self.x = None
        self.signal = np.empty(shape=(6,1))
        self.thrust = np.empty(shape=(6,1))

        # Robot Configuration
        self.D = 0.02                           # half the distance between the propellers
        self.R = 0.1                            # radial distance from the center of the robot to the midpoint between the two propellers
        self.phi = 2*np.pi/3                    # seperation angle between sets of propellers        
        self.flipped = False                    # change to True if thrust force are directed inwards
        self.mass = 90*10**(-3)                 # in kilograms
        self.J = 0.5*self.mass**2*(self.R+0.02) # inertia of the robot in the z-direction
        self.MAX_THRUST = 60*10**-3             # in newtons 


    def control_callback(self, msg):
        v_x_0   = msg.v_x
        v_y_0   = msg.v_y
        yaw_d_0 = msg.yaw_d

        self.x  = np.empty(shape=(6,1))
        self.x[0] = msg.yaw
        self.x[1] = msg.yaw_d
        self.x[2] = msg.v_x
        self.x[3] = msg.v_y
        self.x[4] = msg.x
        self.x[5] = msg.y

        acc_x  = self.PID_vx.computeOutput(v_x_0)
        acc_y  = self.PID_vy.computeOutput(v_y_0)
        yaw_dd = self.PID_yaw_d.computeOutput(yaw_d_0)
        self.u = np.array([acc_x,acc_y,yaw_dd])
        #self.get_logger().info('Controller Output: {}'.format(self.u))
        self.thrust = getThrust(self)
        self.signal = convertToSignal(self)

        # Cap
        #self.signal[self.signal > 1] = 1

        msg = MotorControl()
        msg.motor_a_1 = float(self.signal[0])
        msg.motor_a_2 = float(self.signal[1])
        msg.motor_b_1 = float(self.signal[2])
        msg.motor_b_2 = float(self.signal[3])
        msg.motor_c_1 = float(self.signal[4])
        msg.motor_c_2 = float(self.signal[5])
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
 
    controller = Controller()

    rclpy.spin(controller)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



