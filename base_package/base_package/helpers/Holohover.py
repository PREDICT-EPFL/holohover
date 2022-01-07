import numpy as np
import scipy
from numpy import DataSource, array, dot
from qpsolvers import solve_qp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt
import os 






class Holohover:
    def __init__(self, x0, u0):

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
        self.u = u0
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
        self.gyro_noise = 0
        self.acc_noise = np.zeros(shape=(2,1))

    def __getDiscrete(self):
        # Convert the continuous system to discrete taking AD/DA conversion
        # Returns [A_d,B_d,C_d,D_d] discrete state-space
        return cont2discrete(system  = [self.A,self.B,self.C,self.D], dt = self.dt, method='zoh') 

    def initializeThrust(self):
        self.thrust = self.__getThrust()

    def predict(self, u):
        # Takes an input vector u and precticts the state at iteration i+1
        self.x_prev = self.x
        self.u_prev = self.u
        self.u = u
        self.x = np.add(self.A_d @ self.x, np.reshape((self.B_d @ self.u), newshape=(6,)))
        self.updateThrust()

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
        return np.array([Fx,Fy,Fz])

    def __fromTranslation(self):
        """
        Computes Newton's law equations in Translation
        
        :returns: F: (3,1) numpy array including the world forces (Fx,Fy,Fz)
                  P: (3,18) numpy array including relating the world forces to the local force vector (18,1)  
        """
        I = np.eye(3,3)
        tmp = np.concatenate((I,I,I),axis=1)
        P = self.__getBodytoWorld() @ np.concatenate((tmp,tmp), axis=1)
        F = self.__getWorldForces()
        return F,P

    def __fromRotation(self):
        """
        Computes Newton's law equations in Rotation
        
        :returns: M: (3,1) numpy array including the world moments (Mx,My,Mz)
                  G: (3,18) numpy array including relating the world moments to the local force vector (18,1)  
        """
        M = np.array([0,0,self.J*self.u[2]])
        R = self.__getRadialVector()
        G = np.empty(shape=(3,18))
        for idx,r in enumerate(R):
            cross = np.array([[0    ,-r[2],r[1]],
                             [r[2] ,0    ,-r[0]],
                             [-r[1],r[0] ,   0]])
            G[:,idx*3:idx*3+3] = cross
        return M,G 

    def getCameraReading(self, noisy=False):
        """
        Simulates a camera reading. Returns global positions
        
        :returns: pose: (3,1) numpy array including the pose of the robot in global frame (X,Y,Yaw)
        """
        
        self.camera = self.x[4:]
        
        # Simulate gaussian noise
        if noisy:
            N = len(self.camera)
            sigma = 0.02
            self.camera_noise = sigma*np.random.randn(int(N),1)
            self.camera = np.add(self.camera,self.camera_noise[:,0])
        return self.camera

    def getIMUReading(self, noisy=False):
        """
        Simulates an IMU reading. Returns linear accelerations and angular velocity readings

        :returns: acc:  (3,1) numpy array including the linear acceleration of the robot
                  gyro: (3,1) numpy array including the angular velocities of the robot
        """

        self.acc = self.u[0:2] #[TODO]
        self.gyro = self.x[1] #[TODO]

        if noisy:
            self.gyro_noise = self.gyro_noise + (np.random.randn() + 1)*0.01
            self.gyro = np.add(self.gyro,self.gyro_noise)
            sigma = 0.02
            self.acc_noise = sigma*np.random.randn(2,1)
            self.acc = np.add(self.acc, self.acc_noise[:,0])
        
        return self.gyro, self.acc

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


if __name__ == "__main__":
    x0 = np.zeros(shape=(6,1))
    u0 = np.zeros(shape=(3,1))
    Robot = Holohover(x0[:,0],u0[:,0])

    SIMULATION_TIME = 10 #In seconds
    SIMULATION_LENGTH = int(SIMULATION_TIME/Robot.dt)
    
    # States
    U = np.zeros(shape=(SIMULATION_LENGTH+1,3))
    X = np.empty(shape=(SIMULATION_LENGTH+1,6))
    F = np.empty(shape=(SIMULATION_LENGTH+1,6))
    
    # Sensors
    C = np.empty(shape=(SIMULATION_LENGTH+1,2))
    A = np.empty(shape=(SIMULATION_LENGTH+1,2))
    G = np.empty(shape=(SIMULATION_LENGTH+1,1))

    # Initialize results
    U[:,:] = [0.1,0,0]
    X[0,:] = x0[:,0]
    U[0,:] = u0[:,0]
    Robot.initializeThrust()
    F[0,:] = Robot.thrust


    # Run Simulation
    for i in range(SIMULATION_LENGTH): 
        Robot.predict(U[i,:])
        
        X[i+1,:] = Robot.x
        F[i+1,:] = Robot.thrust
        C[i+1,:] = Robot.getCameraReading(noisy=True)
        G[i+1,:], A[i+1,:] = Robot.getIMUReading(noisy=True)


    # Write Data 
    dir_path = os.path.dirname(os.path.realpath(__file__))+'/results/model_validation'
    t = np.arange(0,(SIMULATION_LENGTH+1)*Robot.dt,Robot.dt)
    np.savetxt(dir_path + "/t.csv", t, delimiter=",")
    np.savetxt(dir_path + "/F.csv", F, delimiter=",")
    np.savetxt(dir_path + "/U.csv", U, delimiter=",")
    np.savetxt(dir_path + "/X.csv", X, delimiter=",")

    # Plots

    f, ax = plt.subplots(2, 2, sharey=False)

    f.suptitle('State Space Readings')
    
    ax[0,0].plot(t,X[:,4])
    ax[0,0].set_title('Position')

    ax[1,0].plot(t,X[:,2])
    ax[1,0].set_title('Velocity')

    ax[0,1].plot(t,U[:,0])
    ax[0,1].set_title('Acceleration')

    ax[1,1].plot(t,F[:,0])
    ax[1,1].plot(t,F[:,1])
    ax[1,1].plot(t,F[:,2])
    ax[1,1].plot(t,F[:,3])
    ax[1,1].plot(t,F[:,4])
    ax[1,1].plot(t,F[:,5])
    ax[1,1].set_title('Propellers')


    f2, ax2 = plt.subplots(3, sharey=False)

    f2.suptitle('Noisy Sensor Readings')

    ax2[0].plot(t,C[:,0])
    ax2[0].set_title('Position')

    ax2[1].plot(t,A[:,0])
    ax2[1].set_title('Acceleration')

    ax2[2].plot(t,G[:,0])
    ax2[2].set_title('Angular Velocity')


    f3, ax3 = plt.subplots(2, 2, sharey=False)

    f3.suptitle('State Space Readings')
    
    ax3[0,0].plot(t,X[:,0])
    ax3[0,0].set_title('Yaw')

    ax3[1,0].plot(t,X[:,1])
    ax3[1,0].set_title('Yaw_d')

    ax3[0,1].plot(t,U[:,2])
    ax3[0,1].set_title('Yaw_dd')

    f.show()
    f2.show()
    f3.show()

    input("<Hit Enter To Close>")
    print('Simulation complete')








