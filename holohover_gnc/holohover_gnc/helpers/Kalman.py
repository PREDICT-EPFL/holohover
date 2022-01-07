import numpy as np
from numpy.linalg import inv
from Predictor import Predictor
from Sensor import Sensor

# Initialize Error Matrix
R1 = np.diag([20,5,5,5,5,5])
R2 = np.diag([20,5,5,5,5,5])
P = np.diag([20,5,5,5,5,5])

# Initialize Camera
camera_sensor = Sensor(R1,P)

# Initialize IMU
imu_sensor = Sensor(R2,P)

# Initialize Predictor
state_predictor = Predictor(P)

# Initial State Matrix 
state_predictor.X[0] = 0                        # [TODO] Change position
state_predictor.X[1] = 0                        # [TODO] Change position
state_predictor.X[2] = 0                        # [TODO] Change orientation


