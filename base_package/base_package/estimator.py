import rclpy
from rclpy.node import Node, NodeNameNonExistentError
from holohover_msgs.msg import MotorControl, DroneMeasurement, Pose, DroneState, MotorControl
from std_msgs.msg import String
import numpy as np
from .helpers.Holohover import Holohover
from .helpers.Sensor import Sensor
import scipy
from numpy import DataSource, array, dot
from qpsolvers import solve_qp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt
import time

class Estimator(Node):

    def __init__(self, x0, u0, input='acceleration'):
        super().__init__('Estimator')
        
        self.dt = 1/100 #Update rate
        
        # Subscribers
        self.subscription_1 = self.create_subscription(Pose,'camera/robot_pose',self.updateFromCamera_callback,10)
        self.subscription_1  # prevent unused variable warning
        self.subscription_2 = self.create_subscription(DroneMeasurement,'drone/measurement',self.updateFromIMU_callback,10)
        self.subscription_2  # prevent unused variable warning
        self.subscription_3 = self.create_subscription(MotorControl,'drone/motor_control',self.motor_command_callback,10)
        self.subscription_3  # prevent unused variable warning
        
        # Publishers
        self.publisher = self.create_publisher(DroneState, '/estimator/state', 10)
        self.timer = self.create_timer(self.dt, self.predict_callback)
        
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
        
        
        self.A_d,self.B_d,self.C_d,self.D_d,_ = self.__getDiscrete()
        self.x_previous = x0
        self.u_previous = u0   
        self.thrust = None

        # Robot Configuration
        self.D = 0.02                           # half the distance between the propellers
        self.R = 0.1                            # radial distance from the center of the robot to the midpoint between the two propellers
        self.phi = 2*np.pi/3                    # seperation angle between sets of propellers        
        self.flipped = False                    # change to True if thrust force are directed inwards
        self.mass = 90*10**(-3)                 # in kilograms
        self.J = 0.5*self.mass**2*(self.R+0.02) # inertia of the robot in the z-direction
        self.MAX_THRUST = 100                   # in newtons
        self.YAW_OFFSET = 0 #[TODO]
        # Mappings
        self.P = None
        self.G = None

        # Initialize Estimator Parameters
        self.IMU_previousTime = time.time()
        self.Camera_previousTime = time.time()
        self.v_x = self.x[2]
        self.v_y = self.x[3]
        self.x_prev = self.x[4]
        self.y_prev = self.x[5]
        self.yaw_prev = self.x[0]

        # Kalman Filter Parameters
        self.IMU = Sensor()
        self.IMU.R = np.array([[0.01,0  ,0      ,0],
                               [0   ,3  ,0      ,0],
                               [0   ,0  ,0.05   ,0],
                               [0   ,0  ,0   ,0.05]])

        self.IMU.H = np.array([[1,0,0,0,0,0],
                               [0,1,0,0,0,0],
                               [0,0,1,0,0,0],
                               [0,0,0,1,0,0]])
        self.IMU.x = self.x
        self.IMU.z = None
        self.IMU.K = None

        self.P_KF = np.array([[0.1,0,0,0,0,0],
                              [0,0.1,0,0,0,0],
                              [0,0,0.1,0,0,0],
                              [0,0,0,0.1,0,0],
                              [0,0,0,0,0.1,0],
                              [0,0,0,0,0,0.1]])

        self.Q_KF = np.array([[0.01,0,0,0,0,0],
                              [0,0.01,0,0,0,0],
                              [0,0,0.1,0,0,0],
                              [0,0,0,0.1,0,0],
                              [0,0,0,0,0.01,0],
                              [0,0,0,0,0,0.01]])

        self.Camera = Sensor()
        self.Camera.R = np.array([[0.01,0   ,0      ,0,   0, 0],
                                  [0   ,0.1 ,0      ,0,   0, 0],
                                  [0   ,0   ,0.01   ,0,   0, 0],
                                  [0   ,0   ,0   ,0.01 ,   0, 0],
                                  [0   ,0   ,0   ,0   , 0.01, 0],
                                  [0   ,0   ,0      ,0,   0,  0.01]])

        self.Camera.H = np.array([[1,0,0,0,0,0],
                                  [0,1,0,0,0,0],
                                  [0,0,1,0,0,0],
                                  [0,0,0,1,0,0],
                                  [0,0,0,0,1,0],
                                  [0,0,0,0,0,1]])

        self.Camera.x = self.x
        self.Camera.z = None
        self.Camera.K = None


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
        self.x_previous = self.x
        self.u_previous = self.u
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
        return T

    def motor_command_callback(self, msg):
        # Update the thrust input
        #self.get_logger().info('Updating the thrust force')
        signal = np.array([msg.motor_a_1, msg.motor_a_2, msg.motor_b_1, msg.motor_b_2, msg.motor_c_1, msg.motor_c_2])
        thrust = self.__convertSignal(signal)
        self.u = self.__getAcceleration(np.transpose(thrust))
        self.updateThrust()

    def predict_callback(self):
        self.predict(input=self.u, input_type='acceleration')
        self.P_KF = self.A_d @ self.P_KF @ np.transpose(self.A_d) + self.Q_KF
        #print('Predictor: {}'.format(self.x))

        # Publish
        msg = DroneState()
        msg.yaw    = self.x[0]
        msg.yaw_d  = self.x[1]
        msg.v_x    = self.x[2]
        msg.v_y    = self.x[3]
        msg.x      = self.x[4]
        msg.y      = self.x[5]

        self.publisher.publish(msg)

    def updateFromIMU_callback(self, msg):
        acc_x = msg.acc.x
        acc_y = msg.acc.y
        gyro_z = msg.gyro.z

        IMU_Time = time.time()
        Delta = IMU_Time - self.IMU_previousTime
        self.v_x = self.v_x + acc_x*Delta
        self.v_y = self.v_y + acc_y*Delta
        yaw = msg.atti.yaw + self.YAW_OFFSET
        yaw_d = gyro_z
        self.IMU_Prev_Time = IMU_Time

        # Update Kalman Gain
        self.IMU.z = np.array([yaw,yaw_d,self.v_x,self.v_y])
        S = self.IMU.H @ self.P_KF @ np.transpose(self.IMU.H) + self.IMU.R
        self.IMU.K = self.P_KF @ np.transpose(self.IMU.H) @ np.linalg.inv(S)

        # Update State
        self.IMU.x = self.x + self.IMU.K @ (self.IMU.z - self.IMU.H @ self.x)
        self.x = self.IMU.x
        #print('IMU: {}'.format(self.x))

        # Update State Error Covariance
        self.P_KF = self.P_KF - (self.IMU.K @ self.IMU.H @ self.P_KF)
    

    def updateFromCamera_callback(self, msg):
        x = msg.x
        y = msg.y
        yaw = msg.yaw

        Camera_Time =  time.time()
        Delta = Camera_Time - self.Camera_previousTime
        v_x = (self.x_prev-x)/Delta
        v_y = (self.y_prev-y)/Delta
        yaw_d = (self.yaw_prev - yaw)/Delta
        self.Camera_previousTime = Camera_Time

        # Update Kalman Gain
        self.Camera.z = np.array([yaw,yaw_d,v_x,v_y,x,y])
        S = self.Camera.H @ self.P_KF @ np.transpose(self.Camera.H) + self.Camera.R
        self.Camera.K = self.P_KF @ np.transpose(self.Camera.H) @ np.linalg.inv(S)

        # Update State
        self.Camera.x = self.x + self.Camera.K @ (self.Camera.z - self.Camera.H @ self.x)
        self.x = self.Camera.x
        self.x_prev = x
        self.y_prev = y
        self.yaw_prev = yaw
        #print('Camera: {}'.format(self.x))

        # Update State Error Covariance
        self.P_KF = self.P_KF - (self.Camera.K @ self.Camera.H @ self.P_KF)


def main(args=None):
    rclpy.init(args=args)
 
    # Initialize simulator
    x0 = np.zeros(shape=(6,1))
    u0 = np.zeros(shape=(3,1))
    estimator = Estimator(x0[:,0],u0[:,0])
    estimator.initializeThrust() # Initialize the corresponding thrust based on initital inputs

    rclpy.spin(estimator)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
