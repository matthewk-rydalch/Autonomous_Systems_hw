#ekf_slam
import numpy as np
import scipy
import scipy.signal
import random
import math
import matplotlib.pyplot as plt
import scipy.io as sio
from IPython.core.debugger import set_trace
from importlib import reload, import_module
rob_2wh = reload(import_module("rob_2wh"))
from rob_2wh import Rob2Wh
visuals = reload(import_module("visuals"))
from visuals import Visualizer
slam_ekf = reload(import_module("slam_ekf"))
from slam_ekf import Slam

############################################
####given parameters
alpha = np.array([0.1, 0.01, 0.01, 0.1, 0.01, 0.01]) #velocity noise model characteristics
M = np.array([[6.0,4.0], [-7.0,8.0], [6.0,-4.0]]) #landmark locations
sig_r = 0.1 #sensor noise standard deviation
sig_phi = 0.05 #rad
dt = 0.1
tf = 20
x0 = -5 #initial states
y0 = -3
th0 = math.pi/2 #rad
xgrid = [-10, 10] #map size
ygrid = [-10, 10]

# if given data is being used
# given = sio.loadmat('hw3_1_soln_data.mat') #1 marker
# # given = sio.loadmat('hw3_4_soln_data.mat') #3 markers
# self.z_btr = given['bearing'] #only for 1 marker
# self.z_rtr = given['range'] #only for 1 marker
# self.wtr = given['om']
# self.ttr = given['t']
# self.thtr = given['th']
# self.vtr = given['v']
# self.xtr = given['x']
# self.ytr = given['y']

###instatiate objects
rob = Rob2Wh(dt, alpha, M, sig_r, sig_phi)
viz = Visualizer()
slam = Slam(rob.vel_motion_model. rob.model_sensor, sig_r, sig_phi, M, dt)

###initial values
Mup = np.array([[x0], [y0], [th0]])
Sig_p = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0,0.0,0.1]])
time_steps = int(tf/dt)

###go through algorithm for each time step
for i in range(0,time_steps+1):

    t = i*rob.dt
    Ut = rob.generate_command(t)
    Zt = rob.model_sensor(Mup)

    Mu, Sig, K = slam.ekf(Mup,Sig_p,Ut,Zt)

    # mu_hist.append(Mu)
    # sig_hist.append(Sig)
    # k_hist.append(K)

    # Mup = Mu
    # Sig_p = Sig




# size = len(mu)
# x_hat = []
# y_hat =[]
# th_hat = []
# sig_x = []
# sig_y = []
# sig_th = []
# for i in range(size):
#     x_hat.append(mu[i][0])
#     y_hat.append(mu[i][1])
#     th_hat.append(mu[i][2])
#     sig_x.append(Sig[i][0][0])
#     sig_y.append(Sig[i][1][1])
#     sig_th.append(Sig[i][2][2])
# viz.animator(x, y, th, x_hat,y_hat,th_hat, elements)

# viz.plotting(x_hat, x, y_hat, y, th_hat, th, vc, v, wc, w,\
#     t, xe, ye, the, ve, we, K, sig_x, sig_y, sig_th)
