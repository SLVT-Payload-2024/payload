import numpy as np

class Config:

    DT = 0.02 # estimated constant software loop time
    MOI = 1.5625e-04 # moment of inertia on rotation axis

    ##### Kalman Filter #####
    A = np.array([[1,DT],[0,1]])
    B = np.array([[DT/MOI],[0]])
    C = np.array([[1,0]])
    FWF = np.array([[0.1,0],[0,1]])
    V = np.array([[1]])

    # use pre-computed solution for Discrete time Algebric Riccati Equation
    P = np.array([[0.7293,1.3150],[1.3150,6.5457]])
    #########################

    ##### MPC #####
    N = 15
    AMPC = np.array([[1]])
    BMPC = np.array([[DT/MOI]])
    Q = np.array([1])
    R = np.array([0.1])

    # motor output constraints
    MOTOR_TOC_LOW = 0.0010
    MOTOR_TOC_HIGH = 0.0032
    #########################

    itr_tol = 1e-6
