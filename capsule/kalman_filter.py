import numpy as np
from scipy.linalg import norm, inv

from config import Config

class KalmanFilter:

    def __init__(self, a=None, b=None, c=None, fwf=None, v=None) -> None:
        a = Config.A if a is None else a
        b = Config.B if b is None else b
        c = Config.C if c is None else c
        fwf = Config.FWF if fwf is None else fwf
        v = Config.V if v is None else v
        p = Config.P

        self.a, self.b, self.c = a, b, c
        self.fwf, self.v = fwf, v
        self.set_dare(p)
        self.filt_state = None
    
    def sys_get(self):
        return self.a, self.b, self.c, self.fwf, self.v, self.p

    def state_get(self):
        return self.filt_state

    def set_dare(self, p):
        self.p = p
        self.gain = self.__compute_gain()

    def asym_init(self, init_cond):
        if init_cond.shape[0] != self.a.shape[0] or init_cond.shape[1] != 1:
            raise Exception("Shape does not Comply @ Initial Condition")
        self.filt_state = init_cond

    def asym_update(self, measure, ctrl):
        a, b, c = self.a, self.b, self.c
        gain, filt_state = self.gain, self.filt_state

        filt_state = a @ filt_state + b @ ctrl + gain @ (measure - c @ filt_state)
        self.filt_state = filt_state

    def __compute_gain(self):
        a, c, v, p = self.a, self.c, self.v, self.p
        return a @ p @ c.T @ inv(v + c @ p @ c.T)

# Example Use Case
# kf = KalmanFilter()
# kf.asym_init(init_cond)
# kf.asym_update(x_measure,u)
