import matplotlib.pyplot as plt
import numpy as np
from IPython.core.debugger import set_trace

class Plotter:
    def plotting(self, x_hat, xt, y_hat, yt, th_hat, tht, t, xe, ye, the, sig_x, sig_y, sig_th):

        #hats are predicted states
        #xt yt tht are curent truth states
        #xe, ye, the are curent state Error

        #calculate upper and lower bounds for covariance plot
        sigx_hi = 2*np.sqrt(sig_x)
        sigx_lo = -2*np.sqrt(sig_x)
        sigy_hi = 2*np.sqrt(sig_y)
        sigy_lo = -2*np.sqrt(sig_y)
        sigth_hi = 2*np.sqrt(sig_th)
        sigth_lo = -2*np.sqrt(sig_th)

        #subplots for states vs. time
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

    def test(self, Xk):
        fig1 = plt.figure()
        plt.scatter(Xk[0,:],Xk[1,:])
        fig1.show
