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
####set parameters
alpha = np.array([0.1, 0.01, 0.01, 0.1, 0.01, 0.01]) #velocity noise model characteristict
Mtr = np.array([[6.0,4.0], [-7.0,-8.0], [6.0,-4.0], [7.0,-8.0], [-1.0,-1.0],\
                [-6.0,-4.0], [7.0,8.0], [-6.0,4.0], [-7.0,8.0], [1.0,1.0],\
                [3.0,-9.0], [-2.0,-8.0], [6.0,4.0], [7.0,-3.0], [-7.0,-1.0],\
                [6.0,5.0], [5.0,-8.0], [-6.0,-4.0], [7.0,-2.0], [9.0,-1.0]]) #actual landmark locations
fov = 90 #field of view deg
sig_r = 0.1 #sensor noise standard deviation
sig_phi = 0.05 #rad
dt = 0.1
tf = 50
x0 = 0.0 #initial states
y0 = 0.0
th0 = 0.0 #rad
xgrid = [-15, 15] #map size
ygrid = [-15, 15]

###storage variables for plotting
mu_hist = []
xhat_hist = []
sig_hist = []
xtr_hist = []
t_hist = []
z_hist = []
marker_hist = []
###

###initial values
N = len(Mtr) #number of landmarks
M = np.zeros((N,2))

##getting initial Sig_p
mat1 = np.zeros((2*N+3,3))
mat2 = np.zeros((3,2*N))
mat3 = np.diag(1000000*np.ones(2*N))
mat4 = np.concatenate((mat2,mat3),axis=0)
Sig_p = np.concatenate((mat1.T,mat4.T),axis=0)
##

time_steps = int(tf/dt)
Xtru = np.concatenate((np.array([[x0,y0,th0]]).T,np.reshape(Mtr, (N*2,1))), axis=0) #combining position states to markers.  markers are ordered mx1 my1 mx2 my2 etc.
Mup = np.concatenate((np.array([[x0,y0,th0]]).T,np.reshape(M, (N*2,1))), axis=0) #combining position states to markers.  markers are ordered mx1 my1 mx2 my2 etc.
Fx = np.concatenate((np.eye(3,3).T,np.zeros((3,2*N)).T), axis=0).T
###

###instatiate objects
rob = Rob2Wh(dt, alpha, Mtr, sig_r, sig_phi)
viz = Visualizer(Mtr, xgrid, ygrid)
slam = Slam(rob.vel_motion_model, sig_r, sig_phi, alpha, dt, N, Fx, Mtr)

fov = fov*np.pi/180 #convert field of view to radians

###go through algorithm for each time step
for i in range(0,time_steps+1):

    t = i*dt
    Ut = rob.generate_command(t)
    Xtru = rob.vel_motion_model(Ut, Xtru, Fx, noise=1)
    Zt, ct = rob.model_sensor(Xtru, fov, noise=1)

    Mu, Sig = slam.ekf(Mup, Sig_p, Ut, Zt, ct)
    xhat_hist.append(Mu[0:3])
    marker_hist.append(Mu[3:2*N+3])
    sig_hist.append(Sig)
    xtr_hist.append(Xtru[0:3])
    t_hist.append(t)
    z_hist.append(Zt)

    Mup = Mu
    Sig_p = Sig

viz.plotting(xhat_hist, sig_hist, xtr_hist, marker_hist, t_hist)
viz.animator(xtr_hist, xhat_hist, sig_hist, marker_hist, time_steps)