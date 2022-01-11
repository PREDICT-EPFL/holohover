import numpy as np
from qpsolvers import solve_qp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt
import os
import math


class Holohover:
    def __init__(self, x0, u0, dt=1/100):

        # Robot State Space
        self.A = np.array(([0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0]))
        self.B = np.array(([0, 0, 0],
                           [0, 0, 1],
                           [1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 0],
                           [0, 0, 0]))

        self.C = np.eye(6, 6)
        self.D = np.zeros((6, 3))
        self.x = x0
        self.u = u0
        self.dt = dt
        self.A_d, self.B_d, self.C_d, self.D_d, _ = self.__getDiscrete()

        # Robot Configuration
        self.D = 0.015  # half the distance between the propellers
        self.R = 0.043  # radial distance from the center of the robot to the midpoint between the two propellers
        self.phi = 2 * np.pi / 3  # separation angle between sets of propellers
        self.phi_offset = np.pi / 3  # angle of first set of propellers
        self.flipped = True  # True, if thrust force are directed inwards
        self.mass = 90 * 10 ** (-3)  # in kilograms
        self.J = 0.5 * self.mass ** 2 * (self.R + 0.02)  # inertia of the robot in the z-direction
        self.MAX_THRUST = 60.2 * 1e-3  # in newtons

        # Sensors
        self.camera = np.array([self.x[4], self.x[5], self.x[0]])
        self.acc = np.array([self.u[0], self.u[1], 0])
        self.gyro = np.array([0, 0, self.x[1]])
        self.camera_noise = np.zeros(3)
        self.gyro_noise = np.zeros(3)
        self.acc_noise = np.zeros(3)

    def __getDiscrete(self):
        # Convert the continuous system to discrete taking AD/DA conversion
        # Returns [A_d,B_d,C_d,D_d] discrete state-space
        return cont2discrete(system=[self.A, self.B, self.C, self.D], dt=self.dt, method='zoh')

    def predict(self, u):
        # Takes an input vector u and practices the state at iteration i+1
        self.u = u
        self.x = self.A_d @ self.x + self.B_d @ self.u
        # Map yaw between 0 and 2pi
        if self.x[0] < 0:
            self.x[0] += math.ceil(-self.x[0] / (2 * math.pi)) * 2 * math.pi
        elif self.x[0] > 2 * math.pi:
            self.x[0] -= math.floor(self.x[0] / (2 * math.pi)) * 2 * math.pi

    def __getBodytoWorld(self):
        # Returns the transformation matrix from body to world frame
        T = np.array(([np.cos(self.x[0]), -np.sin(self.x[0]), 0],
                      [np.sin(self.x[0]), np.cos(self.x[0]), 0],
                      [0, 0, 1]))
        return T

    def __getWorldtoBody(self):
        # Returns the transformation matrix from world to body frame
        T = np.array(([np.cos(self.x[0]), -np.sin(self.x[0]), 0],
                      [np.sin(self.x[0]), np.cos(self.x[0]), 0],
                      [0, 0, 1]))
        return T.transpose()

    def __getDirectionVector(self):
        """
        Returns the direction vector of the propellers in local frame 

        :returns: (6,3) numpy array including the different unit vectors
        """

        e1 = np.array([np.sin(self.phi_offset + self.phi * 0), -np.cos(self.phi_offset + self.phi * 0), 0])
        e2 = -1 * e1
        e3 = np.array([np.sin(self.phi_offset + self.phi * 1), -np.cos(self.phi_offset + self.phi * 1), 0])
        e4 = -1 * e3
        e5 = np.array([np.sin(self.phi_offset + self.phi * 2), -np.cos(self.phi_offset + self.phi * 2), 0])
        e6 = -1 * e5
        if self.flipped:
            return -np.stack([e1, e2, e3, e4, e5, e6], axis=0)
        else:
            return np.stack([e1, e2, e3, e4, e5, e6], axis=0)

    def __getRadialVector(self):
        """
        Returns the position vector of the propellers in local frame 

        :returns: (6,3) numpy array including the different position vectors
        """

        alpha = np.arctan(self.D / self.R)
        r = np.sqrt(self.R ** 2 + self.D ** 2)
        r1 = [r * np.cos(self.phi_offset + self.phi * 0 - alpha), r * np.sin(self.phi_offset + self.phi * 0 - alpha), 0]
        r2 = [r * np.cos(self.phi_offset + self.phi * 0 + alpha), r * np.sin(self.phi_offset + self.phi * 0 + alpha), 0]
        r3 = [r * np.cos(self.phi_offset + self.phi * 1 - alpha), r * np.sin(self.phi_offset + self.phi * 1 - alpha), 0]
        r4 = [r * np.cos(self.phi_offset + self.phi * 1 + alpha), r * np.sin(self.phi_offset + self.phi * 1 + alpha), 0]
        r5 = [r * np.cos(self.phi_offset + self.phi * 2 - alpha), r * np.sin(self.phi_offset + self.phi * 2 - alpha), 0]
        r6 = [r * np.cos(self.phi_offset + self.phi * 2 + alpha), r * np.sin(self.phi_offset + self.phi * 2 + alpha), 0]
        return np.array([r1, r2, r3, r4, r5, r6])

    def __getDirectionMatrix(self):
        """
        Returns a direction matrix which maps the thrusts in magnitude (6,) to thrusts in vector format (18,) to
        
        :returns: (18,6) numpy array including the different direction vectors
        """

        E = np.zeros((18, 6))
        e = self.__getDirectionVector()
        for idx, vec in enumerate(e):
            E[idx * 3:idx * 3 + 3, idx] = vec
        return E

    def __getWorldForces(self):
        Fx = self.u[0] * self.mass
        Fy = self.u[1] * self.mass
        Fz = 0
        return np.array([Fx, Fy, Fz])

    # Body to World Force Mapping Matrix
    def __getForceMapping(self):
        I = np.eye(3, 3)
        tmp = np.concatenate((I, I, I), axis=1)
        P = self.__getBodytoWorld() @ np.concatenate((tmp, tmp), axis=1)
        return P

    def __fromTranslation(self):
        """
        Computes Newton's law equations in Translation
        
        :returns: F: (3,) numpy array including the world forces (Fx,Fy,Fz)
                  P: (3,18) numpy array including relating the local force vectors (18,) to world forces
        """
        F = self.__getWorldForces()
        P = self.__getForceMapping()
        return F, P

    # Body to World Moment Mapping Matrix
    def __getMomentMapping(self):
        R = self.__getRadialVector()
        G = np.empty((3, 18))
        for idx, r in enumerate(R):
            cross = np.array([[0, -r[2], r[1]],
                              [r[2], 0, -r[0]],
                              [-r[1], r[0], 0]])
            G[:, idx * 3:idx * 3 + 3] = cross
        return G

    def __fromRotation(self):
        """
        Computes Newton's law equations in Rotation
        
        :returns: M: (3,) numpy array including the world moments (Mx,My,Mz)
                  G: (3,18) numpy array including relating the world moments to the local force vector (18,1)  
        """
        M = np.array([0, 0, self.J * self.u[2]])
        G = self.__getMomentMapping()
        return M, G

    def getCameraReading(self, noisy=False):
        """
        Simulates a camera reading. Returns global positions
        
        :returns: pose: (3,) numpy array including the pose of the robot in global frame (X,Y,Yaw)
        """

        self.camera = np.array([self.x[4], self.x[5], self.x[0]])

        # Simulate gaussian noise
        if noisy:
            sigma_pos = 0.002  # cm
            sigma_yaw = 0.2 * np.pi / 180  # rad == 0.2 deg
            self.camera_noise = np.array([
                sigma_pos * np.random.randn(),
                sigma_pos * np.random.randn(),
                sigma_yaw * np.random.randn()
            ])
            self.camera += self.camera_noise
        return self.camera

    def getIMUReading(self, noisy=False):
        """
        Simulates an IMU reading. Returns linear accelerations and angular velocity readings

        :returns: acc:  (3,) numpy array including the linear acceleration of the robot
                  gyro: (3,) numpy array including the angular velocities of the robot
        """

        self.acc = np.array([self.u[0], self.u[1], 0])
        self.gyro = np.array([0, 0, self.x[1]])

        if noisy:
            sigma_yaw_d = 0.001
            sigma_acc = 0.02
            self.gyro_noise = np.array([0, 0, sigma_yaw_d])
            self.gyro += self.gyro_noise
            self.acc_noise = np.array([sigma_acc * np.random.randn(), sigma_acc * np.random.randn(), 0])
            self.acc += self.acc_noise

        return self.gyro, self.acc

    def getThrust(self):
        """
        Returns the thrust vector of the propellers taking the inputs as self.u

        :returns: (6,) numpy array including the different thrusts in magnitude
        """

        F, P = self.__fromTranslation()
        M, G = self.__fromRotation()
        E = self.__getDirectionMatrix()

        # Concatenate
        H = np.concatenate((P, G), axis=0)
        A = H @ E
        b = np.concatenate((F, M), axis=0)

        # QP formualtion
        #
        # min   ||F_i||^2_2 + mu * ||eps||^2_2
        # s.t.  (Fx,Fy,Fz,Mx,My,Mz) = A @ (F_1,...,F_6) + eps
        #       0 <= F_i <= F_max
        #
        # translated into standard form
        # min   0.5 * x'Px + q'x
        # s.t.  Gx <= h
        #       Ax = b
        #       lb <= x <= ub
        #
        # with x = (F_i, eps)

        mu = 1e6
        P = np.diag(np.concatenate((np.ones(6), mu * np.ones(6))))
        q = np.zeros(12)
        G = None
        h = None
        A = np.hstack((A, np.eye(6)))
        b = b
        lb = np.concatenate((np.zeros(6), -np.inf * np.ones(6)))
        ub = np.concatenate((self.MAX_THRUST * np.ones(6), np.inf * np.ones(6)))
        T = solve_qp(P, q, G, h, A, b, lb, ub)

        return T[:6]

    def getAcceleration(self, thrust):
        """
        Returns the acceleration input vector taking the motor thrusts

        :returns: (3,) numpy array including the input acceleration (ax,ay,yaw_d_d)
        """

        P = self.__getForceMapping()
        G = self.__getMomentMapping()
        H = np.concatenate((P, G), axis=0)
        E = self.__getDirectionMatrix()

        A = H @ E @ thrust

        a_x = A[0] / self.mass
        a_y = A[1] / self.mass
        gamma_dd = A[5] / self.J

        return np.array([a_x, a_y, gamma_dd])

    def convertFromSignal(self, motor_input):
        motor_input = 1000 + 1000 * motor_input
        return self.__fromSignalToThrust(motor_input)

    def __fromSignalToThrust(self, x):
        y = 1.5447e-7 * x ** 3 - 0.000565 * x ** 2 + 0.70302 * x - 292.4456
        return y * 1e-3  # mN -> N

    def convertToSignal(self, thrust):
        # Input a polynomial function that maps the force in N to a signal
        # between 1000 2000
        signal = self.__fromThrustToSignal(thrust)  # returns a signal between [1000-2000]
        signal = 0.001 * signal - 1  # normalize to [0-1]
        return signal

    def __fromThrustToSignal(self, x):
        x = x * 1e3  # N -> mN
        return 0.0143 * x ** 3 - 1.6098 * x ** 2 + 60.891 * x + 1048.5


if __name__ == "__main__":
    x0 = np.zeros(6)
    u0 = np.zeros(3)
    Robot = Holohover(x0, u0)

    SIMULATION_TIME = 10  # In seconds
    SIMULATION_LENGTH = int(SIMULATION_TIME / Robot.dt)

    # States
    U = np.zeros((SIMULATION_LENGTH + 1, 3))
    X = np.empty((SIMULATION_LENGTH + 1, 6))
    F = np.empty((SIMULATION_LENGTH + 1, 6))

    # Sensors
    C = np.empty((SIMULATION_LENGTH + 1, 3))
    A = np.empty((SIMULATION_LENGTH + 1, 3))
    G = np.empty((SIMULATION_LENGTH + 1, 3))

    # Initialize results
    U[:, :] = [0.1, 0, 0]
    X[0, :] = x0
    U[0, :] = u0
    F[0, :] = Robot.getThrust()

    # Run Simulation
    for i in range(SIMULATION_LENGTH):
        Robot.predict(U[i, :])

        X[i + 1, :] = Robot.x
        F[i + 1, :] = Robot.getThrust()
        print(Robot.convertToSignal(Robot.getThrust()))
        C[i + 1, :] = Robot.getCameraReading(noisy=True)
        G[i + 1, :], A[i + 1, :] = Robot.getIMUReading(noisy=True)

    # Write Data
    dir_path = os.path.dirname(os.path.realpath(__file__))
    np.savetxt(dir_path + "/F.csv", F, delimiter=",")
    np.savetxt(dir_path + "/U.csv", U, delimiter=",")
    np.savetxt(dir_path + "/X.csv", X, delimiter=",")

    # Plots
    t = np.arange(0, (SIMULATION_LENGTH + 1) * Robot.dt, Robot.dt)

    f, ax = plt.subplots(2, 2, sharey=False)

    f.suptitle('State Space Readings')

    ax[0, 0].plot(t, X[:, 4])
    ax[0, 0].set_title('Position')

    ax[1, 0].plot(t, X[:, 2])
    ax[1, 0].set_title('Velocity')

    ax[0, 1].plot(t, U[:, 0])
    ax[0, 1].set_title('Acceleration')

    ax[1, 1].plot(t, F[:, 0])
    ax[1, 1].plot(t, F[:, 1])
    ax[1, 1].plot(t, F[:, 2])
    ax[1, 1].plot(t, F[:, 3])
    ax[1, 1].plot(t, F[:, 4])
    ax[1, 1].plot(t, F[:, 5])
    ax[1, 1].set_title('Propellers')

    f2, ax2 = plt.subplots(3, sharey=False)

    f2.suptitle('Noisy Sensor Readings')

    ax2[0].plot(t, C[:, 0])
    ax2[0].set_title('Position')

    ax2[1].plot(t, A[:, 0])
    ax2[1].set_title('Acceleration')

    ax2[2].plot(t, G[:, 2])
    ax2[2].set_title('Angular Velocity')

    f3, ax3 = plt.subplots(2, 2, sharey=False)

    f3.suptitle('State Space Readings')

    ax3[0, 0].plot(t, X[:, 0])
    ax3[0, 0].set_title('Yaw')

    ax3[1, 0].plot(t, X[:, 1])
    ax3[1, 0].set_title('Yaw_d')

    ax3[0, 1].plot(t, U[:, 2])
    ax3[0, 1].set_title('Yaw_dd')

    f.show()
    f2.show()
    f3.show()

    input("<Hit Enter To Close>")
    print('Simulation complete')
