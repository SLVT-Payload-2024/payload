import numpy as np
import random
import time
import matplotlib.pyplot as plt

from kalman_filter import KalmanFilter
from mpc import MPC
from config import Config

init_cond = np.array([[0],[0]])
init_ctrl = 0
x_filter = init_cond
x = x_filter[0,0]
u = 0

x_hist, u_hist = list(), list()
xf_hist, bias_hist = list(), list()
t_hist = list()

t = 0
dt = Config.DT
tf = 10

kf = KalmanFilter()
kf.asym_init(init_cond)
mpc = MPC()

while t <= tf:
    start_time = time.time()
    x_hist.append(x)
    u_hist.append(u)
    t_hist.append(t)
    
    kf.asym_update(x,np.array([[u]]))
    filt_state = kf.state_get()
    
    xf_hist.append(filt_state[0,0])
    bias_hist.append(filt_state[1,0])

    u = mpc.update(current_state=filt_state[0,0], bias=dt*filt_state[1,0])

    # dynamic update
    torque_ext = 0.002*(1 + 0.2*random.random())
    dt = Config.DT*(1 + 0.2*random.random())
    x = x + dt/Config.MOI*(torque_ext + u)

    t = t + dt

plt.figure(figsize=(10, 8))
plt.subplot(3, 1, 1)
plt.plot(t_hist, x_hist, label="true state")
plt.plot(t_hist, xf_hist, label="filtered state")
plt.xlabel("Simulated Time, s")
plt.ylabel("Roll Speed, rad/s")
plt.legend()
plt.subplot(3, 1, 2)
plt.plot(t_hist, bias_hist)
plt.xlabel("Simulated Time, s")
plt.ylabel("Bias Estimate, 1/s^2")
plt.subplot(3, 1, 3)
plt.plot(t_hist, u_hist)
plt.xlabel("Simulated Time, s")
plt.ylabel("Control Torque, Nm")
plt.show()
