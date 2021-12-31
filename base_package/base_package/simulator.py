import rclpy
from rclpy.node import Node
from holohover_msgs.msg import MotorControl, DroneMeasurement, Pose
from std_msgs.msg import String
import numpy as np
from .helpers.Holohover import Holohover
import scipy
from numpy import DataSource, array, dot
from qpsolvers import solve_qp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt
import time

class Simulator(Node):

    def __init__(self, x0, u0, input = 'acceleration'):
        super().__init__('Simulator')
        self.subscription_1 = self.create_subscription(MotorControl,'drone/motor_control',self.listener_callback,10)
        self.subscription_1  # prevent unused variable warning

        self.publisher_1 = self.create_publisher(DroneMeasurement, '/drone/measurement', 10)
        timer_period = 0.005  # 200Hz
        self.timer_1 = self.create_timer(timer_period, self.measurement_callback)
        self.publisher_2 = self.create_publisher(Pose, '/camera/robot_pose', 10)
        self.timer_2 = self.create_timer(timer_period, self.pose_callback)

        # Robot State Space
        self.A = np.array(([0,1,0,0,0,0],
                          [0,0,0,0,0,0],
                          [0,0,0,0,0,0],
                          [0,0,0,0,0,0],
                          [0,0,1,0,0,0],
                          [0,0,0,1,0,0]))
        self.B = np.array(([0,0,0],
                           [0,0,1],
                           [1,0,0],
                           [0,1,0],
                           [0,0,0],
                           [0,0,0]))

        self.C = np.eye(6,6)
        self.D = np.zeros(shape=(6,3))
        self.x = x0

        if input == 'acceleration':
            self.u = u0
        else:
            self.u = self.__getAcceleration(u0)
        
        self.dt = 1/100
        self.A_d,self.B_d,self.C_d,self.D_d,_ = self.__getDiscrete()
        self.x_prev = x0
        self.u_prev = u0   
        self.thrust = None

        # Robot Configuration
        self.D = 0.02                           # half the distance between the propellers
        self.R = 0.1                            # radial distance from the center of the robot to the midpoint between the two propellers
        self.phi = 2*np.pi/3                    # seperation angle between sets of propellers        
        self.flipped = False                    # change to True if thrust force are directed inwards
        self.mass = 90*10**(-3)                 # in kilograms
        self.J = 0.5*self.mass**2*(self.R+0.02) # inertia of the robot in the z-direction
        self.MAX_THRUST = 100                   # in newtons
        
        # Sensors
        self.camera = self.x[4:]
        self.acc = self.u[0:2]
        self.gyro = self.x[1] 
        self.camera_noise = np.zeros(shape=(2,1)) 
        self.gyro_noise = np.array([0,0,0])
        self.acc_noise = np.zeros(shape=(2,1))

        # Mappings
        self.P = None
        self.G = None

        # Initial Time
        self.previousTime = time.time()
        self.Time = None

    # Body to World Force Mapping Matrix
    def __getForceMapping(self):
        I = np.eye(3,3)
        tmp = np.concatenate((I,I,I),axis=1)
        P = self.__getBodytoWorld() @ np.concatenate((tmp,tmp), axis=1)
        return P


    # Body to World Moment Mapping Matrix
    def __getMomentMapping(self):  
        R = self.__getRadialVector()
        G = np.empty(shape=(3,18))
        for idx,r in enumerate(R):
            cross = np.array([[0    ,-r[2],r[1]],
                             [r[2] ,0    ,-r[0]],
                             [-r[1],r[0] ,   0]])
            G[:,idx*3:idx*3+3] = cross
        return G

    def __getDiscrete(self):
        # Convert the continuous system to discrete taking AD/DA conversion
        # Returns [A_d,B_d,C_d,D_d] discrete state-space
        return cont2discrete(system  = [self.A,self.B,self.C,self.D], dt = self.dt, method='zoh') 

    def initializeThrust(self):
        self.thrust = self.__getThrust()

    def predict(self, input, input_type = 'acceleration'):
        # Takes an input vector u and precticts the state at iteration i+1
        self.x_prev = self.x
        self.u_prev = self.u
        if input_type == 'acceleration':
            self.u = input
        elif input_type == 'signal':
            thrust = self.__convertSignal(input)
            self.u = self.__getAcceleration(np.transpose(thrust))
        self.x = np.add(self.A_d @ self.x, np.reshape((self.B_d @ self.u), newshape=(6,)))
        self.updateThrust()

    def __convertSignal(self, input):
        b = 1000
        a = 1000
        input = np.add(1000,np.multiply(input, np.ones(shape=(1,6))*1000))
        for idx, i in enumerate(input):
            input[idx] = self.fromSignaltoThrust(i)*10**-3
        return input

    def fromSignaltoThrust(self, x):
        return (1.5447*10**-7)*x**3 - 0.000565*x**2 + 0.70302*x - 292.4456

    def __getBodytoWorld(self):
        # Returns the transformation matrix from body to world frame
        T = np.array(([np.cos(self.x[0]),-np.sin(self.x[0]),0],
                     [np.sin(self.x[0]), np.cos(self.x[0]),0],
                     [0                , 0                ,1]))
        return T

    def __getWorldtoBody(self):
        # Returns the transformation matrix from world to body frame
        T = np.array(([np.cos(self.x[0]),-np.sin(self.x[0]),0],
                     [np.sin(self.x[0]), np.cos(self.x[0]),0],
                     [0                , 0                ,1]))
        return T.transpose()

    def __getDirectionVector(self):
        """
        Returns the direction vector of the propellers in local frame 

        :returns: (6,3) numpy array including the different unit vectors
        """

        e1 = np.array([np.cos(self.phi*0),np.sin(self.phi*0),0])
        e2 = -1*e1
        e3 = np.array([np.cos(self.phi*1),np.sin(self.phi*1),0])
        e4 = -1*e3
        e5 = np.array([np.cos(self.phi*2),np.sin(self.phi*2),0])
        e6 = -1*e5
        if self.flipped:
            return -np.stack([e1,e2,e3,e4,e5,e6], axis=0)
        else:
            return np.stack([e1,e2,e3,e4,e5,e6], axis=0)

    def __getRadialVector(self):
        """
        Returns the position vector of the propellers in local frame 

        :returns: (6,3) numpy array including the different position vectors
        """

        alpha = np.arctan(self.D/self.R)
        r = np.sqrt(self.R**2+self.D**2)
        r1 = [r*np.sin(self.phi*0-alpha),  r*np.cos(self.phi*0-alpha), 0]
        r2 = [r*np.sin(self.phi*0+alpha),  r*np.cos(self.phi*0+alpha), 0]
        r3 = [r*np.sin(self.phi*1-alpha),  r*np.cos(self.phi*1-alpha), 0]
        r4 = [r*np.sin(self.phi*1+alpha),  r*np.cos(self.phi*1+alpha), 0]
        r5 = [r*np.sin(self.phi*2-alpha),  r*np.cos(self.phi*2-alpha), 0]
        r6 = [r*np.sin(self.phi*2+alpha),  r*np.cos(self.phi*2+alpha), 0]
        return np.array([r1,r2,r3,r4,r5,r6])
    

    def updateThrust(self):
        self.thrust = self.__getThrust()
    
    def __getDirectionMatrix(self):
        """
        Returns a direction matrix which maps the thrusts in vector format (18,1) to thrusts in magnitude as (6,1)
        
        :returns: (18,6) numpy array including the different direction vectors
        """

        E = np.empty(shape=(18,6))
        e = self.__getDirectionVector()
        for idx, vec in enumerate(e):
            tmp = np.zeros(shape=(3,6))
            tmp[:,idx] = vec
            E[idx*3:idx*3+3,:] = tmp
        return E


    def __getWorldForces(self):
        Fx = self.u[0]*self.mass
        Fy = self.u[1]*self.mass
        Fz = 0
        return np.array([Fx,Fy,Fz], dtype='object')

    def __fromTranslation(self):
        """
        Computes Newton's law equations in Translation
        
        :returns: F: (3,1) numpy array including the world forces (Fx,Fy,Fz)
                  P: (3,18) numpy array including relating the world forces to the local force vector (18,1)  
        """
        P = self.__getForceMapping()
        F = self.__getWorldForces()
        return F,P

    def __fromRotation(self):
        """
        Computes Newton's law equations in Rotation
        
        :returns: M: (3,1) numpy array including the world moments (Mx,My,Mz)
                  G: (3,18) numpy array including relating the world moments to the local force vector (18,1)  
        """
        M = np.array([0,0,self.J*self.u[2]], dtype='object')
        R = self.__getRadialVector()
        G = self.__getMomentMapping()
        return M,G 

    def getCameraReading(self, noisy=False):
        """
        Simulates a camera reading. Returns global positions
        
        :returns: pose: (3,1) numpy array including the pose of the robot in global frame (X,Y,Yaw)
        """
        
        self.camera = np.array([self.x[4],self.x[5],self.x[0]])
        
        # Simulate gaussian noise
        if noisy:
            N = len(self.camera)
            sigma_pos = 0.002 # cm
            sigma_yaw = 0.2 * np.pi / 180 # rad == 0.2 deg
            self.camera_noise = np.array([sigma_pos*np.random.randn(),sigma_pos*np.random.randn(),sigma_yaw*np.random.randn()])
            self.camera = np.add(self.camera,self.camera_noise)
        return self.camera

    def getIMUReading(self, noisy=False):
        """
        Simulates an IMU reading. Returns linear accelerations and angular velocity readings

        :returns: acc:  (3,1) numpy array including the linear acceleration of the robot
                  gyro: (3,1) numpy array including the angular velocities of the robot
        """

        self.acc = np.array([self.u[0], self.u[1],0] , dtype='object') #[TODO]
        self.gyro = np.array([0,0,self.x[1]]) #[TODO]

        if noisy:
            self.gyro_noise = np.array([0,0,self.gyro_noise[2] + (np.random.randn() + 1)*0.0001])
            self.gyro = np.add(self.gyro,self.gyro_noise)
            sigma = 0.02
            self.acc_noise = np.array([sigma*np.random.randn(),sigma*np.random.randn(),0])
            self.acc = np.add(self.acc, self.acc_noise)
        
        return self.gyro, self.acc

    def __getAcceleration(self, U):
        P = self.__getForceMapping()
        G = self.__getMomentMapping()
        H = np.concatenate((P,G), axis=0)
        E = self.__getDirectionMatrix()

        A = H @ E @ U

        a_x = A[0]/self.mass
        a_y = A[1]/self.mass
        gamma_dd = A[5]/self.J

        return np.array([a_x,a_y,gamma_dd]) 

    def __getThrust(self):
        """
        Returns the thrust vector of the propellers taking the inputs as self.u

        :returns: (6,1) numpy array including the different thrusts in magnitude
        """

        F,P = self.__fromTranslation()
        M,G = self.__fromRotation()
        E = self.__getDirectionMatrix()

        # Concatenate
        H = np.concatenate((P,G), axis=0)
        A = H @ E
        b = np.concatenate((F,M), axis=0)

        # QP formualtion
        P = np.eye(6, dtype=float)
        q = np.zeros(shape=(6,), dtype=float)
        G = np.eye(6, dtype=float)
        h = np.ones(shape=(6,), dtype=float)*self.MAX_THRUST
        A = A.astype(float)
        b = b.astype(float)
        lb = np.zeros(shape=(6,))
        T = solve_qp(P,q,G,h,A,b,lb)
        return 

    def listener_callback(self, msg):
        #Update the thrust input
        #self.get_logger().info('Updating the thrust force')
        Thrust = np.array([msg.motor_a_1, msg.motor_a_2, msg.motor_b_1, msg.motor_b_2, msg.motor_c_1, msg.motor_c_2])
        self.predict(input=Thrust, input_type='signal')

    def measurement_callback(self):
        msg = DroneMeasurement()

        # Attitude
        msg.atti.roll = float(0)
        msg.atti.pitch = float(0)
        msg.atti.yaw = float(self.x[0])

        # Accelerometer and Gyro
        gyro, acc  = self.getIMUReading(noisy=True)
        msg.gyro.x = float(gyro[0])
        msg.gyro.y = float(gyro[1])
        msg.gyro.z = float(gyro[2])

        msg.acc.x = float(acc[0])
        msg.acc.y = float(acc[1])
        msg.acc.z = float(acc[2])

        self.publisher_1.publish(msg)

    def pose_callback(self):
        msg = Pose()

        pose = self.getCameraReading(noisy=True)
        msg.x = pose[0]
        msg.y = pose[1]
        msg.yaw = pose[2]

        self.publisher_2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
 
    # Initialize simulator
    x0 = np.zeros(shape=(6,1))
    u0 = np.zeros(shape=(3,1))
    simulator = Simulator(x0[:,0],u0[:,0])
    simulator.initializeThrust() # Initialize the corresponding thrust based on initital inputs

    rclpy.spin(simulator)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()