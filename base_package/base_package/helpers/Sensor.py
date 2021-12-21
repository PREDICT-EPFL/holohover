import numpy as np
from numpy.linalg import inv


class Sensor:
    def __init__(self,R,P):
        self.Z = np.ones(shape=(6,1))
        self.R = R
        self.H = np.ones(shape=(6,6))
        self.P = P
        self.S = self.H.dot(self.P).dot(self.H.T) + self.R
        self.K = self.P.dot(self.H).dot(inv(self.S))

    def update_states(self, P, X, Z):
        # Compute kalman gain
        self.P = P #Update P value
        self.Z = Z #Update Z value
        self.S = self.H.dot(self.P).dot(self.H.T) + self.R
        self.K = self.P.dot(self.H).dot(inv(self.S))

        # Reshape the new data into the measurement space.
        Y = self.H.dot(self.Z).reshape(6, 1)

        # Update the State Matrix
        # Combination of the predicted state, measured values, covariance matrix and Kalman Gain
        X = X + self.K.dot(Y - self.H.dot(X))

        # Update Process Covariance Matrix
        self.P = (np.identity(len(self.K)) - self.K.dot(self.H)).dot(self.P)

        return X, self.P 