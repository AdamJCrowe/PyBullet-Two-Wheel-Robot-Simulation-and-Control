import numpy as np


class Observer:
    """
    TBC - Kalman filter
    """
        
    def __init__(self):
        self.state = np.array(([0.0],[0.0],[0.0],[0.0]))
