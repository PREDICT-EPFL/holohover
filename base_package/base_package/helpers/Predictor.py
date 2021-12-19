import numpy as np

class Predictor:
    def __init__(self, P, X):
        self.A = np.ones(shape=(6,6))
        self.B = np.ones(shape=(3,1))
        self.Q = np.zeros(shape=(6,6)) 
        self.P = P
        self.X = np.zeros(shape=(6,1))
   
    def covariance_predict(self):
        return np.matmul(self.A,np.matmul(self.P,np.transpose(self.A))) + self.Q

    def state_predict(self,X,P,U,t):
        self.A = np.array([[1, 0, 0, t, 0, 0],
                    [0, 1, 0, 0, t, 0],
                    [0, 0, 1, 0, 0, t],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]])
        
        
        self.B = np.array([[0.5*t**2, 0 , 0],
                    [0, 0.5*t**2, 0],
                    [0, 0 , 0.5*t**2],
                    [t, 0 , 0],
                    [0, t, 0],
                    [0, 0, t]])
        
        # Update State Matrix
        X_prime = self.A.dot(X) + self.B.dot(U)

        # Update Process Covariance Matrix
        P = self.covariance_predict(self.A,P,self.Q)

        return X_prime, P
