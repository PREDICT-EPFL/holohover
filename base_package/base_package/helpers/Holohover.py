import numpy as np
import scipy
from numpy import array, dot
from qpsolvers import solve_qp
class Holohover:
    def __init__(self, x0, u0):
        self.A = np.array([0,1,0,0,0,0],
                          [0,0,0,0,0,0],
                          [0,0,0,0,0,0],
                          [0,0,0,0,0,0],
                          [0,0,1,0,0,0],
                          [0,0,0,1,0,0])
        self.B = np.array([0,0,0],
                          [0,0,1],
                          [1,0,0],
                          [0,1,0],
                          [0,0,0],
                          [0,0,0],)

        self.C = np.eye(6,6)
        self.D = np.zeros(6,3)
        self.x = x0
        self.u = u0
        self.A_d,self.B_d,self.C_d,self.D_d = self.__getDiscrete()
        self.x_prev = x0
        self.u_prev = u0
        self.D = None #[TODO]                    # half the distance between the propellers
        self.R = None #[TODO]                    # radial distance from the center of the robot to the midpoint between the two propellers
        self.phi = 2*np.pi/3                # seperation angle between sets of propellers
        self.flipped = False                # change to True if thrust force are directed inwards
        self.thrust = self.__getThrust()
        self.mass = None #[TODO]
        self.J = None #[TODO]                    # inertia of the robot in the z-direction


    def __getDiscrete(self):
        # Convert the continuous system to discrete taking AD/DA conversion
        # Returns [A_d,B_d,C_d,D_d] discrete state-space
        return scipy.signal.cont2discrete(system  = [self.A,self.B,self.C,self.D], dt = 1/100, method='zoh') 


    def predict(self, u):
        # Takes an input vector u and precticts the state at iteration i+1
        self.x_prev = self.x
        self.u_prev = self.u
        self.u = u
        self.x = self.A_d*self.x + self.B_d*self.u 

    def __getBodytoWorld(self):
        # Returns the transformation matrix from body to world frame
        T = np.array([np.cos(self.x[0]),-np.sin(self.x[0]),0],
                     [np.sin(self.x[0]), np.cos(self.x[0]),0],
                     [0                , 0                ,1],)
        return T

    def __getWorldtoBody(self):
        # Returns the transformation matrix from world to body frame
        T = np.array([np.cos(self.x[0]),-np.sin(self.x[0]),0],
                     [np.sin(self.x[0]), np.cos(self.x[0]),0],
                     [0                , 0                ,1],)
        return T.transpose()

    def __getDirectionVector(self):
        """
        Returns the direction vector of the propellers in local frame 

        :returns: (6,3) numpy array including the different unit vectors
        """

        e1 = [np.cos(self.phi*0),np.sin(self.phi*0),0]
        e2 = -e1
        e3 = [np.cos(self.phi*1),np.sin(self.phi*1),0]
        e4 = -e3
        e5 = [np.cos(self.phi*2),np.sin(self.phi*2),0]
        e6 = -e5
        if self.flipped:
            return -np.array([e1,e2,e3,e4,e5,e6])
        else:
            return np.array([e1,e2,e3,e4,e5,e6])

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
    
    def __getThrust(self):
        """
        Returns the thrust vector of the propellers taking the inputs as self.u

        :returns: (6,1) numpy array including the different thrusts in magnitude
        """

        F,P = self.__fromTranslation()
        M,G = self.__fromRotation()
        E = self.__getDirectionMatrix()

        # concatenate the systems of equations
        H = np.concatenate((P,G), axis=0)
        A = np.concatenate((H,E), axis=1)
        b = np.concatenate((F,M), axis=0)

        # build QP
        P = np.ones(shape=(6,6))
        thrust = solve_qp(P,  np.zeros(shape=(6,1)), np.zeros(shape=(6,6)), np.zeros(shape=(6,1)),A,b)
        return thrust

    def __updateThrust(self):
        self.thrust = self.__getThrust()
    
    def __getDirectionMatrix(self):
        """
        Returns a direction matrix which maps the thrusts in vector format (18,1) to thrusts in magnitude as (6,1)
        
        :returns: (18,6) numpy array including the different direction vectors
        """

        E = np.empty(shape=(18,6))
        e = self.__getDirectionVector()
        for idx, vec in e:
            tmp = np.zeros(shape=(3,6))
            tmp[:,idx] = vec.transpose()
            E[idx*3:idx*3+3,:] = tmp
        return E


    def __getWorldForces(self):
        Fx = self.u[0]*self.mass
        Fy = self.u[1]*self.mass
        Fz = 0
        return [Fx,Fy,Fz]

    def __fromTranslation(self):
        """
        Computes Newton's law equations in Translation
        
        :returns: F: (3,1) numpy array including the world forces (Fx,Fy,Fz)
                  P: (3,18) numpy array including relating the world forces to the local force vector (18,1)  
        """
        I = np.eye(3,3)
        tmp = np.concatenate((I,I,I),axis=1)
        P = self.__getBodytoWorld() * np.concatenate((tmp,tmp), axis=1)
        F = self.__getWorldForces()
        return F,P

    def __fromRotation(self):
        """
        Computes Newton's law equations in Rotation
        
        :returns: M: (3,1) numpy array including the world moments (Mx,My,Mz)
                  G: (3,18) numpy array including relating the world moments to the local force vector (18,1)  
        """
        M = [0,0,self.J*self.u[2]]
        R = self.__getRadialVector()
        G = np.empty(shape=(3,18))
        for idx,r in enumerate(R):
            cross = np.array([[0    ,-r[2],r[1]],
                             [r[2] ,0    ,-r[0]],
                             [-r[1],r[0] ,   0]])
            G[:,idx*3:idx*3+3] = cross
        return M,G 

    def __getCameraReading(noisy=False):
        """
        Simulates a camera reading. Returns global positions
        
        :returns: pose: (3,1) numpy array including the pose of the robot in global frame (X,Y,Yaw)
        """
        
        pose = None #[TODO]
        return pose

    def __getIMUReading(noisy=False):
        """
        Simulates an IMU reading. Returns linear accelerations and angular velocity readings

        :returns: acc:  (3,1) numpy array including the linear acceleration of the robot
                  gyro: (3,1) numpy array including the angular velocities of the robot
        """

        acc = None #[TODO]
        gyro = None #[TODO]
        return


    