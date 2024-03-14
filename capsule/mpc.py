import numpy as np
from numpy.linalg import inv

from config import Config

class MPC:

    def __init__(self, a=None, b=None, q=None, r=None) -> None:
        a = Config.AMPC if a is None else a
        b = Config.BMPC if b is None else b
        q = Config.Q if q is None else q
        r = Config.R if r is None else r
        n = Config.N

        constraint_bias = lambda bias: bias*np.ones([n,1])
        constraint_A = np.hstack([-1*np.eye(n) + np.hstack([np.vstack([np.zeros([1,n-1]),a*np.eye(n-1)]),np.zeros([n,1])]), b*np.eye(n)])
        constraint_QR = np.block([[q*np.eye(n),np.zeros([n,n])],[np.zeros([n,n]),r*np.eye(n)]])
        constraint_block = np.block([[constraint_A, np.zeros([n,n])],[2*constraint_QR.T, constraint_A.T]])

        self.a = a
        self.constr_A = constraint_A
        self.constr_bias = constraint_bias
        self.constr_QR = constraint_QR
        self.constr_block = constraint_block

        # predictive controller horizon
        self.n = n
    
    def update(self, current_state, bias):
        constr_b = self.constr_bias(-bias)
        constr_b[0,0] -= self.a * current_state
        constr_block = self.constr_block
        xopt = inv(constr_block) @ np.vstack([constr_b, np.zeros([2*self.n,1])])
        return xopt[self.n,0]
