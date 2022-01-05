import numpy as np
import scipy
from numpy import DataSource, array, dot
from qpsolvers import solve_qp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt
import time

# Body to World Force Mapping Matrix
def __getForceMapping(self):
    I = np.eye(3,3)
    tmp = np.concatenate((I,I,I),axis=1)
    P = __getBodytoWorld(self) @ np.concatenate((tmp,tmp), axis=1)
    return P

# Body to World Moment Mapping Matrix
def __getMomentMapping(self):  
    R = __getRadialVector(self)
    G = np.empty(shape=(3,18))
    for idx,r in enumerate(R):
        cross = np.array([[0    ,-r[2],r[1]],
                            [r[2] ,0    ,-r[0]],
                            [-r[1],r[0] ,   0]])
        G[:,idx*3:idx*3+3] = cross
    return G

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


def __getDirectionMatrix(self):
    """
    Returns a direction matrix which maps the thrusts in vector format (18,1) to thrusts in magnitude as (6,1)
    
    :returns: (18,6) numpy array including the different direction vectors
    """

    E = np.empty(shape=(18,6))
    e = __getDirectionVector(self)
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
    P = __getForceMapping(self)
    F = __getWorldForces(self)
    return F,P

def __fromRotation(self):
    """
    Computes Newton's law equations in Rotation
    
    :returns: M: (3,1) numpy array including the world moments (Mx,My,Mz)
                G: (3,18) numpy array including relating the world moments to the local force vector (18,1)  
    """
    M = np.array([0,0,self.J*self.u[2]], dtype='object')
    R = __getRadialVector(self)
    G = __getMomentMapping(self)
    return M,G 

def getThrust(self):
    """
    Returns the thrust vector of the propellers taking the inputs as self.u

    :returns: (6,1) numpy array including the different thrusts in magnitude
    """

    F,P = __fromTranslation(self)
    M,G = __fromRotation(self)
    E = __getDirectionMatrix(self)

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


def convertToSignal(self):
    # Input a polynomial function that maps the force in N to a signal
    # between 1000 2000
    if self.thrust is not None:
        for idx,i in enumerate(self.thrust):
            self.signal[idx] = fromThrusttoSignal(i) # Returns a signal between [1000-2000]
            self.signal[idx] = 0.001*self.signal[idx] - 1 # Returns a signal between [0-1]
    else:
        print('Thrust is None')
        self.signal = np.zeros(shape=(6,1))
    return self.signal

def fromThrusttoSignal(x):
    x = x*10**3 #mN
    return 0.0143*x**3 - 1.6098*x**2 + 60.891*x + 1048.5
