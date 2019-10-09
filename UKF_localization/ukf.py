# everything important
from IPython.core.debugger import set_trace
from importlib import reload

#kf_localization
import numpy as np
import scipy
import scipy.signal
# import control
import random
import math
import matplotlib.pyplot as plt

import rob_2wh
import animator
reload(rob_2wh)
reload(animator)
from rob_2wh import Rob2Wh
from animator import Animator


def main():

    markers = 1
    given = 0
    rob = Rob2Wh()
    animate = Animator()

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
            vc_new, wc_new = generate_command(t[i])
            (x_new, y_new, th_new, v_new, w_new) = rob.vel_motion_model(vc_new, wc_new, x_new, y_new, th_new)
            u_new = np.array([vc_new,wc_new])
            z_new = rob.simulate_sensor(x_new, y_new, th_new)
        else:
            t.append(t_given[0][i])
            vc_new, wc_new = generate_command(t[i])
            x_new = x_given[0][i]
            y_new = y_given[0][i]
            th_new = th_given[0][i]
            u_new = np.array([v_given[0][i],w_given[0][i]])
            if markers == 1:
                z_new = np.array([[z_given[0,i], 0, 0, z_given[1,i], 0, 0]]).T
            else:
                z_new = rob.simulate_sensor(x_new, y_new, th_new)

        for j in range(markers):
            marker = j
            mu_new, Sig_new, K_new = rob.UKF(mu_prev,Sig_prev,u_new,z_new, marker)
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

    rob.plotting(x_hat, x, y_hat, y, th_hat, th, vc, v, wc, w,\
        t, xe, ye, the, ve, we, K, sig_x, sig_y, sig_th)

    return(x, y, th, z, mu, Sig)

def generate_command(t):
    vc_new = 1+0.5*math.cos(2*math.pi*0.2*t)
    wc_new = -0.2+2*math.cos(2*math.pi*0.6*t)

    return vc_new, wc_new

if __name__ == '__main__':
	 [x, y, th, z, mu, Sig] = main()
