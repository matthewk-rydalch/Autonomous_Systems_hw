#kf_localization
import numpy as np
import scipy
import scipy.signal
# import control
import random
import math
import matplotlib.pyplot as plt
import scipy.io as sio
from IPython.core.debugger import set_trace
from rob_2wh import rob_2wh
from animator2 import animator


def main():

    rob = rob_2wh()
    animate = animator()
    # given = sio.loadmat('hw2_soln_data.mat')
    # set_trace()

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
    Sig_prev = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0,0.0,0.1]])
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

        t.append(i*rob.dt) #coment out if t is given
        vc_new = 1+0.5*math.cos(2*math.pi*0.2*t[i])
        # vc_new = 2+.8*math.cos(2*math.pi*0.2*t[i])
        vc.append(vc_new)
        wc_new = -0.2+2*math.cos(2*math.pi*0.6*t[i])
        # wc_new = 1+2*math.cos(2*math.pi*0.6*t[i])
        wc.append(wc_new)

        # # if data needs to be generated
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
        # set_trace()
        # z_now = rob.simulate_sensor(x[i], y[i], th[i])
        # set_trace()
        # z.append(z_now)

        mu_new, Sig_new, K_new = rob.EKF(mu_prev,Sig_prev,u_now,z_now)
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




    # tf = 50.0
    # t0 = 0
    # elements = int(tf/uuv.dt)
    # u_hist = []
    # z_hist = []
    # t_hist = []
    # truth = np.array([[0], [0]])
    # mu = np.array([[2], [2]])
    # muv_hist = []
    # mux_hist = []
    # Sig = np.array([[1, 0],[0,.1]])
    # Sig1_hi_hist = []
    # Sig2_hi_hist = []
    # Sig1_lo_hist = []
    # Sig2_lo_hist = []
    # Kv_hist = []
    # Kx_hist = []
    # erx_hist = [] #error in x
    # erv_hist = [] #error in v
    # trux_hist = []
    # truv_hist = []
    #
    # for i in range(0, elements):
    #     t = i*uuv.dt
    #     if(t<5):
    #         u = 50
    #     elif(t<25):
    #         u = 0
    #     elif(t<30):
    #         u = -50
    #     else:
    #         u = 0
    #     # if(t<25):
    #     #     u = 50
    #     # elif(t<30):
    #     #     u = -300
    #     # elif(t<45):
    #     #     u = -50
    #     # else:
    #     #     u = 0
    #
    #     # if sensory information is given you can use the following values
    #     # set_trace()
    #     z = uuv.z[0][i]
    #     truth = [[uuv.vtr[0][i]],[uuv.xtr[0][i]]]
    #     # if no sensory information is given you have to simulate it below.
    #     # (z,truth) = uuv.simulate_sensor(u, truth)
    #
    #     (mu,Sig,K) = uuv.kalman_filter(mu, Sig, u, z)
    #     er = (mu-truth)
    #
    #     mu = mu.tolist()
    #     er = er.tolist()
    #     K = K.tolist()
    #     # tru = truth.tolist()
    #     tru = truth
    #     muv_hist.append(mu[0])
    #     mux_hist.append(mu[1])
    #     Sig1_hi_hist.append(2*np.sqrt(Sig[0,0]))
    #     Sig2_hi_hist.append(2*np.sqrt(Sig[1,1]))
    #     Sig1_lo_hist.append(-2*np.sqrt(Sig[0,0]))
    #     Sig2_lo_hist.append(-2*np.sqrt(Sig[1,1]))
    #     z_hist.append(z)
    #     erx_hist.append(er[1])
    #     erv_hist.append(er[0])
    #     trux_hist.append(tru[1])
    #     truv_hist.append(tru[0])
    #     Kx_hist.append(K[1])
    #     Kv_hist.append(K[0])
    #     t_hist.append(t)
    #
    # #
    #
    # uuv.plotting(mux_hist, muv_hist, Sig1_hi_hist, Sig2_hi_hist, Sig1_lo_hist, Sig2_lo_hist, erx_hist, erv_hist, trux_hist, truv_hist, Kx_hist, Kv_hist, t_hist)
    #
    # return [muv_hist, mux_hist], t_hist
#

if __name__ == '__main__':
	 [x, y, th, z, mu, Sig] = main()
