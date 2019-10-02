#kf_localization
import numpy as np
import scipy
import scipy.signal
# import control
import random
import math
import matplotlib.pyplot as plt
from IPython.core.debugger import set_trace
from rob_2wh import rob_2wh
from animator import animator

def main():

    rob = rob_2wh()
    animate = animator()

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
    mu_prev = np.array([[rob.x0], [rob.y0], [rob.th0]])
    Sig_prev = np.array([[1.0, 0.0, 0.0],[0.0, 0.1, 0.0],[0.0,0.0,0.1]])
    elements = int(rob.tf/rob.dt)

    for i in range(0,elements+1):

        t.append(i*rob.dt)
        vc_new = 1+0.5*math.cos(2*math.pi*0.2*t[i])
        vc.append(vc_new)
        wc_new = -0.2+2*math.cos(2*math.pi*0.6*t[i])
        wc.append(wc_new)

        if i == 0:
            (x_new, y_new, th_new, v_new, w_new) = rob.vel_motion_model(vc[i], wc[i], rob.x0, rob.y0, rob.th0)
            x.append(rob.x0)
            y.append(rob.y0)
            th.append(rob.th0)
            v.append(v_new)
            w.append(w_new)
        else:
            (x_new, y_new, th_new, v_new, w_new) = rob.vel_motion_model(vc[i], wc[i], x[i-1], y[i-1], th[i-1])
            x.append(x_new)
            y.append(y_new)
            th.append(th_new)
            v.append(v_new)
            w.append(w_new)

        z.append(rob.simulate_sensor(x[i], y[i], th[i]))
        z_now = np.array(z[i])
        u_now = np.array([vc[i],wc[i]])

        mu_new, Sig_new, K_new = rob.UKF(mu_prev,Sig_prev,u_now,z_now)
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

if __name__ == '__main__':
	 [x, y, th, z, mu, Sig] = main()
