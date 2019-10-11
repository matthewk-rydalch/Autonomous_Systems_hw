#Monte Carlo Simulation

from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np
import scipy
import scipy.signal
import random
import math
import matplotlib.pyplot as plt

import rob_2wh
import animator
import monte_carlo
reload(rob_2wh)
reload(animator)
reload(monte_carlo)
from rob_2wh import Rob2Wh
from animator import Animator
from monte_carlo import Monte_Carlo
import utils
reload(utils)
from utils import wrap


def main():

    markers = 1
    given = 0 #set to one if data is given

    rob = Rob2Wh()
    animate = Animator()
    mc = Monte_Carlo()

    t = []
    vc = []
    wc = []
    xt = []
    yt = []
    tht = []
    zt = []
    mu = []
    Sig = []
    xe = []
    ye = []
    the = []
    Xk = []

    states_new = np.array([[rob.x0], [rob.y0], [rob.th0]])
    elements = int(rob.tf/rob.dt)
    M = rob.particles

    if given != 0:
        t_given = rob.ttr
        x_given = rob.xtr
        y_given = rob.ytr
        th_given = rob.thtr
        v_given = rob.vtr
        w_given = rob.wtr
        z_given = np.squeeze(np.array([[rob.z_rtr],[rob.z_btr]]))

    
    for i in range(0,elements+1):
        #extract truth
        if given == 0:
            t.append(i*rob.dt)
            vc_new, wc_new = rob.generate_command(t[i])
            ut = np.array([[vc_new],[wc_new]])
            states_new = rob.vel_motion_model(ut, states_new)
            z_new = rob.simulate_sensor(states_new)
        else:
            t.append(t_given[0][i])
            vc_new, wc_new = rob.generate_command(t[i])
            ut = np.array([[vc_new],[wc_new]])
            states_new = np.array([x_given[0][i], y_given[0][i], th_given[0][i]])
            if markers == 1:
                z_new = np.array([[z_given[0,i], 0, 0, z_given[1,i], 0, 0]]).T
            else:
                z_new = rob.simulate_sensor(states_new) #may need to change depending on the data given


        #filter implimentation
        Xk_prev = mc.uniform_point_cloud(rob.xgrid, rob.ygrid, M)
        for j in range(markers):
            marker = j
            Xkt = mc.monte_carlo(Xk_prev, ut, z_new, M, rob)
            Xk_prev = Xkt
            x_new = np.mean(Xkt[:,0])
            y_new = np.mean(Xkt[:,1])
            th_new = np.mean(Xkt[:,2])
            mu_new = np.array([x_new, y_new, th_new])
            Sig_new = np.cov(Xkt)

        mu.append(mu_new)
        Sig.append(Sig_new)
        xt.append(states_new[0])
        yt.append(states_new[1])
        tht.append(states_new[2])
        xe.append(mu_new[0]-xt[i])
        ye.append(mu_new[1]-yt[i])
        the.append(mu_new[2]-tht[i])
        Xk.append(Xk_prev)

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
    # set_trace()
    animate.animator(xt, yt, tht, x_hat,y_hat,th_hat, elements, rob, Xk)

    plotting(x_hat, xt, y_hat, yt, th_hat, tht,\
        t, xe, ye, the, sig_x, sig_y, sig_th)

    return(xt, yt, tht, zt, mu, Sig)

def plotting(x_hat, xt, y_hat, yt, th_hat, tht, t, xe, ye, the, sig_x, sig_y, sig_th):

    sigx_hi = 2*np.sqrt(sig_x)
    sigx_lo = -2*np.sqrt(sig_x)
    sigy_hi = 2*np.sqrt(sig_y)
    sigy_lo = -2*np.sqrt(sig_y)
    sigth_hi = 2*np.sqrt(sig_th)
    sigth_lo = -2*np.sqrt(sig_th)

    fig1, aXk = plt.subplots(3)
    fig1.suptitle("x, y, and theta vs. Time")
    aXk[0].plot(t, x_hat, label = "x [m]")
    aXk[0].plot(t, xt, label = "true x [m]")
    aXk[0].legend(loc = "upper right")
    aXk[1].plot(t, y_hat, label="y [m]")
    aXk[1].plot(t, yt, label="true y [m]")
    aXk[1].legend(loc = "upper right")
    aXk[2].plot(t, th_hat, label = "theta [rad]")
    aXk[2].plot(t, tht, label = "true theta [rad]")
    aXk[2].legend(loc = "upper right")
    aXk[2].set_xlabel('time [s]')
    fig1.show()

    fig2, aXk = plt.subplots(3)
    fig2.suptitle("Covariance & Error vs. Time")
    aXk[0].plot(t,xe, label="x error [m]")
    aXk[0].plot(t,sigx_hi, label="upper covariance")
    aXk[0].plot(t,sigx_lo, label="lower covariance")
    aXk[0].legend(loc = "upper right")
    aXk[1].plot(t,ye, label="y error [m]")
    aXk[1].plot(t,sigy_hi, label="upper covariance")
    aXk[1].plot(t,sigy_lo, label="lower covariance")
    aXk[1].legend(loc = "upper right")
    aXk[2].plot(t,the, label="theta error [rad]")
    aXk[2].plot(t,sigth_hi, label="upper covariance")
    aXk[2].plot(t,sigth_lo, label="lower covariance")
    aXk[2].legend(loc = "upper right")
    aXk[2].set_xlabel('time [s]')
    fig2.show()
#

if __name__ == '__main__':
	 [x, y, th, z, mu, Sig] = main()
