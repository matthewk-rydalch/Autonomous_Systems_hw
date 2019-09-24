#uuv
import numpy as np
import scipy
import scipy.signal
import control
import random
import math
import matplotlib.pyplot as plt
import scipy.io as sio
from IPython.core.debugger import set_trace

class UUV:
    def __init__(self):
        m = 100.0
        b = 20.0
        meas_noise = .001
        self.vel_noise = 0.01
        self.pos_noise = 0.0001
        # meas_noise = 1
        # self.vel_noise = 0.1
        # self.pos_noise = 1

        self.Q = meas_noise
        self.R = np.diag([self.vel_noise,self.pos_noise])

        a = np.array([[-b/m, 0],[1, 0]])
        b = np.array([[1/m], [0]])
        c = np.array([0, 1])
        d = np.array([0])
        self.dt = 0.05

        sysc = control.StateSpace(a, b, c, d)
        sysd = control.c2d(sysc, self.dt)

        self.A = np.array(sysd.A)
        self.B = np.array(sysd.B)
        self.C = np.array(sysd.C)

        given = sio.loadmat('hw1_soln_data.mat')
        self.z = given['z']
        self.xtr = given['xtr']
        self.vtr = given['vtr']


    def simulate_sensor(self, u, mu):
        # if no sensory data is given you have to simulate it below.
        model = self.A@mu+np.array(self.B*u)
        process_variance = np.random.multivariate_normal([0,0], self.R).T
        truth = model + [[process_variance[0]], [process_variance[1]]]
        z = truth[1] + np.random.normal(0, np.sqrt(self.Q))

        return(z,truth)

    def kalman_filter(self,mu_prev, Sig_prev, u, z):
        (mu_bar, Sig_bar) = self.propagate(mu_prev, Sig_prev, u)

        K_inv_term = 1/(self.C@Sig_bar@self.C.T+self.Q)
        K = Sig_bar@self.C.T@K_inv_term
        mu = mu_bar+K@(z-self.C@mu_bar)
        Sig = (np.eye(2)-K@self.C)@Sig_bar
        return(mu, Sig, K)
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
