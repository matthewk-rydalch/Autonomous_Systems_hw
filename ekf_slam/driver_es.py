#ekf_slam
import numpy as np
import scipy
import scipy.signal
import random
import math
import matplotlib.pyplot as plt
import scipy.io as sio
from IPython.core.debugger import set_trace
rob_2wh = reload(import_module("rob_2wh"))
from rob_2wh import rob_2wh
visualiztion = reload(import_module("visualization"))
from visualization import visualizer
slam_ekf = reload(import_module("slam_ekf"))

############################################

rob = rob_2wh()
viz = visualizer()
slam = slam_ekf()
# given = sio.loadmat('hw2_soln_data.mat')

t = []
vc = []
wc = []
x = []
y = []
th = []
v = []
w = []
z = []
mu = []
Sig = []
K = []
xe = []
ye = []
the = []
ve = []
we = []
Mup = np.array([[rob.x0], [rob.y0], [rob.th0]])
Sig_p = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0,0.0,0.1]])
elements = int(rob.tf/rob.dt)

#if given data is use these
# t = rob.ttr[0]
# w = rob.wtr[0]
# th =rob.thtr[0]
# v = rob.vtr[0]
# x = rob.xtr[0]
# y = rob.ytr[0]
# set_trace()

for i in range(0,elements+1):

    t = i*rob.dt
    Ut = rob.generate_command(t)

    Xt = rob.vel_motion_model(Ut, Mup)
    Zt = rob.simulate_sensor(Xt)
    Mu, Sig, K = slam.slam(Mu,Sig,Ut,Zt)

    z.append(rob.simulate_sensor(x[i], y[i], th[i]))

    
    mu.append(mu_new)
    Sig.append(Sig_new)
    K.append(K_new)

    xe.append(mu_new[0]-x[i])
    ye.append(mu_new[1]-y[i])
    the.append(mu_new[2]-th[i])
    ve.append(vc_new-v[i])
    we.append(wc_new-w[i])


    mu_prev = np.array(mu_new)
    Sig_prev = np.array(Sig_new)



size = len(mu)
x_hat = []
y_hat =[]
th_hat = []
sig_x = []
sig_y = []
sig_th = []
for i in range(size):
    x_hat.append(mu[i][0])
    y_hat.append(mu[i][1])
    th_hat.append(mu[i][2])
    sig_x.append(Sig[i][0][0])
    sig_y.append(Sig[i][1][1])
    sig_th.append(Sig[i][2][2])
animate.animator(x, y, th, x_hat,y_hat,th_hat, elements)

rob.plotting(x_hat, x, y_hat, y, th_hat, th, vc, v, wc, w,\
    t, xe, ye, the, ve, we, K, sig_x, sig_y, sig_th)

return(x, y, th, z, mu, Sig)