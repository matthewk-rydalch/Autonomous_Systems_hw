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


def main():

    markers = 3
    given = 0
    rob = Rob2Wh()
    animate = Animator()
    mc = Monte_Carlo()

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
    x_new = rob.x0
    y_new = rob.y0
    th_new = rob.th0
    mu_prev = np.array([[x_new], [y_new], [th_new]])
    Sig_prev = np.array([[0.1, 0.0, 0.0],[0.0, 0.1, 0.0],[0.0,0.0,0.1]])
    elements = int(rob.tf/rob.dt)
    M = rob.particles

    t_given = rob.ttr
    x_given = rob.xtr
    y_given = rob.ytr
    th_given = rob.thtr
    v_given = rob.vtr
    w_given = rob.wtr
    z_given = np.squeeze(np.array([[rob.z_rtr],[rob.z_btr]]))


    for i in range(0,elements+1):
        if given == 0:
            t.append(i*rob.dt)
            vc_new, wc_new = rob.generate_command(t[i])
            ut = np.array([[vc_new],[wc_new]])
            states_new = np.array([[x_new],[y_new],[th_new]])
            (x_new, y_new, th_new, v_new, w_new) = rob.vel_motion_model(ut, states_new)
            u_new = np.array([v_new,w_new])
            z_new = rob.simulate_sensor(states_new)
        else:
            t.append(t_given[0][i])
            vc_new, wc_new = rob.generate_command(t[i])
            x_new = x_given[0][i]
            y_new = y_given[0][i]
            th_new = th_given[0][i]
            u_new = np.array([v_given[0][i],w_given[0][i]])
            if markers == 1:
                z_new = np.array([[z_given[0,i], 0, 0, z_given[1,i], 0, 0]]).T
            else:
                states = np.array([[x_new],[y_new],[z_new]])
                z_new = rob.simulate_sensor(states_new)

        for j in range(markers):
            marker = j
            mu_new, Sig_new, K_new = mc.monte_carlo(mu_prev,Sig_prev,u_new,z_new, M)
            mu_prev = mu_new
            Sig_prev = Sig_new

        mu.append(mu_new)
        Sig.append(Sig_new)
        K.append(K_new)
        x.append(x_new)
        y.append(y_new)
        th.append(th_new)
        v.append(u_new[0])
        w.append(u_new[0])
        vc.append(vc_new)
        wc.append(wc_new)
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

    plotting(x_hat, x, y_hat, y, th_hat, th, vc, v, wc, w,\
        t, xe, ye, the, ve, we, K, sig_x, sig_y, sig_th)

    return(x, y, th, z, mu, Sig)

def plotting(x_hat, x, y_hat, y, th_hat, th, vc, v, wc, w, t, xe, ye, the, ve, we, K, sig_x, sig_y, sig_th):

    k1r = []
    k1b = []
    k2r = []
    k2b = []
    k3r = []
    k3b = []

    for i in range(len(K)):
        k1r.append(K[i][0][0])
        k1b.append(K[i][0][1])
        k2r.append(K[i][1][0])
        k2b.append(K[i][1][1])
        k3r.append(K[i][2][0])
        k3b.append(K[i][2][1])

    sigx_hi = 2*np.sqrt(sig_x)
    sigx_lo = -2*np.sqrt(sig_x)
    sigy_hi = 2*np.sqrt(sig_y)
    sigy_lo = -2*np.sqrt(sig_y)
    sigth_hi = 2*np.sqrt(sig_th)
    sigth_lo = -2*np.sqrt(sig_th)

    fig1, aXk = plt.subplots(3)
    fig1.suptitle("x, y, and theta vs. Time")
    aXk[0].plot(t, x_hat, label = "x [m]")
    aXk[0].plot(t, x, label = "true x [m]")
    aXk[0].legend(loc = "upper right")
    aXk[1].plot(t, y_hat, label="y [m]")
    aXk[1].plot(t, y, label="true y [m]")
    aXk[1].legend(loc = "upper right")
    aXk[2].plot(t, th_hat, label = "theta [rad]")
    aXk[2].plot(t, th, label = "true theta [rad]")
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

    fig3 = plt.figure(5)
    fig3.suptitle("Kalman Gains vs. Time")
    plt.plot(t, k1r, label="Marker 1 range Kalman gain")
    plt.plot(t, k1b, label="Marker 1 bearing Kalman gain")
    plt.plot(t, k2r, label="Marker 2 range Kalman gain")
    plt.plot(t, k2b, label="Marker 2 bearing Kalman gain")
    plt.plot(t, k3r, label="Marker 3 range Kalman gain")
    plt.plot(t, k3b, label="Marker 3 bearing Kalman gain")
    plt.legend(loc = "upper right")

    fig3.show()
#

if __name__ == '__main__':
	 [x, y, th, z, mu, Sig] = main()
