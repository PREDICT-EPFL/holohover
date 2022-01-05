import numpy as np
from numpy.linalg import inv


class Sensor:
    def __init__(self):
        self.R = None
        self.H = None
        self.K = None
        self.z = None
        self.x = None
