#kf_localization
import numpy as np
import scipy
import scipy.signal
import control
import random
import math
import matplotlib.pyplot as plt
from IPython.core.debugger import set_trace
from rob_2wh import rob_2wh

def main():

    rob = rob_2wh()

    t = []
    vc = []
    wc = []
    x = []
    y = []
    th = []
    elements = int(rob.tf/rob.dt)

    for i in range(0,elements+1):

        t.append(i*rob.dt)
        vc_new = 1+0.5*math.cos(2*math.pi*0.2*t[i])
        vc.append(vc_new)
        wc_new = 0.2+2*math.cos(2*math.pi*0.6*t[i])
        wc.append(wc_new)

        if i == 0:
            x.append(rob.x0)
            y.append(rob.y0)
            th.append(rob.th0)
        else:
            (x_new, y_new, th_new) = rob.vel_motion_model(vc[i], wc[i], x[i-1], y[i-1], th[i-1])
            x.append(x_new)
            y.append(y_new)
            th.append(th_new)

    


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
	 main()