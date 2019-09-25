#uuv
import numpy as np
import scipy
import scipy.signal
import control
import random
import math
import matplotlib.pyplot as plt
# import scipy.io as sio
from IPython.core.debugger import set_trace
# from sympy import Symbol, Derivative

class rob_2wh:
    def __init__(self):
        self.a1 = 0.1
        self.a2 = 0.01
        self.a3 = 0.01
        self.a4 = 0.1

        self.landmark1 = [[6], [4]]
        self.landmark2 = [[-7], [8]]
        self.landmark3 = [[6], [-4]]

        #sensor noise
        self.sig_r = 0.1
        self.sig_phi = 0.05 #rad

        self.dt = 0.1
        self.tf = 20

        self.x0 = -5
        self.y0 = -3
        self.th0 = math.pi/2 #rad

    def vel_motion_model(self, vc, wc, x, y, th):

        np.random.normal(0, np.sqrt(self.a1*vc**2+self.a2*wc**2))
        v_hat = vc + np.random.normal(0, np.sqrt(self.a1*vc**2+self.a2*wc**2))
        w_hat = wc + np.random.normal(0, np.sqrt(self.a3*vc**2+self.a4*wc**2))


        x1 = x - v_hat/w_hat*math.sin(th)+v_hat/w_hat*math.sin(th+w_hat*self.dt)
        y1 = y + v_hat/w_hat*math.cos(th)-v_hat/w_hat*math.cos(th+w_hat*self.dt)
        th1 = th + w_hat*self.dt

        xt_new = (x1,y1,th1)
        return xt_new

    def simulate_sensor(self, vc, wc, mu):
        # if no sensory data is given you have to simulate it below.
        model = self.A@mu+np.array(self.B*u)
        process_variance = np.random.multivariate_normal([0,0], self.R).T
        truth = model + [[process_variance[0]], [process_variance[1]]]
        z = truth[1] + np.random.normal(0, np.sqrt(self.Q))

        return(z,truth)

    def EKF(self,mu_prev, Sig_prev, u, z):

        mu_bar = g(u,mu_prev)
        Sig_bar = G*Sig_prev*G.T+R
        K = Sig_bar*H.T*inv(H*Sig_bar*H.T+Q)
        mu = mu_bar + K*(z - h(mu_bar))
        Sig = (np.eye(2) - K*H)*Sig_bar

        return(mu, Sig)
    #

    def propagate(self,mu, Sig, u):
        mu_bar = self.A@mu+self.B*u
        # process_variance = np.random.multivariate_normal([0,0], self.R).T
        # mu_w_noise = mu_bar + [[process_variance[0]], [process_variance[1]]]
        Sig_bar = self.A@Sig@self.A.T+self.R #R is the covariance of uncertainty Sig

        return(mu_bar, Sig_bar)
        # return(mu_w_noise, Sig_bar)
    #

    def plotting(self,x,v,Sig1_hi,Sig2_hi,Sig1_lo,Sig2_lo,erx,erv,trux,truv,Kx,Kv,t):


        fig1, axs = plt.subplots(2)
        fig1.suptitle("Position and Velocity vs. Time")
        axs[0].plot(t, x, label = "position [m]")
        axs[0].plot(t, trux, label = "true position")
        axs[0].legend(loc = "upper right")
        axs[1].plot(t, v, label="velocity [m/s]")
        axs[1].plot(t,truv, label="true velocity")
        axs[1].legend(loc = "upper right")
        axs[1].set_xlabel('time [s]')
        fig1.show()

        fig2, axs = plt.subplots(2)
        fig2.suptitle("Covariance & Error vs. Time")
        axs[0].plot(t,erx, label="position error [m]")
        axs[0].plot(t,Sig2_hi, label="upper covariance")
        axs[0].plot(t,Sig2_lo, label="lower covariance")
        axs[0].legend(loc = "upper right")
        axs[1].plot(t,erv, label="velocity error [m/s]")
        axs[1].plot(t,Sig1_hi, label="upper covariance")
        axs[1].plot(t,Sig1_lo, label="lower covariance")
        axs[1].legend(loc = "upper right")
        axs[1].set_xlabel('time [s]')
        fig2.show()

        fig3 = plt.figure(3)
        fig3.suptitle("Kalman Gains vs. Time")
        plt.plot(t, Kx, label="Position Kalman Gain")
        plt.plot(t, Kv, label="Velocity Kalman Gain")
        plt.legend(loc = "upper right")

        fig3.show()
    #
