"""
Matplotlib Animation Example

author: Jake Vanderplas
email: vanderplas@astro.washington.edu
website: http://jakevdp.github.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from IPython.core.debugger import set_trace
# from rob_2wh import rob_2wh

class Visualizer:
    def __init__(self, M):
        self.M = M
    def animator(self, Xtru, Mu, elements, Zt):
        Zt = np.array(Zt)
        self.xtru = np.array(Xtru)[:,0]
        self.ytru = np.array(Xtru)[:,1]
        self.thtru = np.array(Xtru)[:,2]
        self.xhat = np.array(Mu)[:,0]
        self.yhat = np.array(Mu)[:,1]
        self.thhat = np.array(Mu)[:,2]
        zr1hat = Zt[:,0,0]
        zr2hat = Zt[:,0,1]
        zr3hat = Zt[:,0,2]
        zb1hat = Zt[:,1,0]
        zb2hat = Zt[:,1,1]
        zb3hat = Zt[:,1,2]
        self.xz1 = zr1hat*np.sin(zb1hat)
        self.xz2 = zr2hat*np.sin(zb2hat)
        self.xz3 = zr3hat*np.sin(zb2hat)
        self.yz1 = zr1hat*np.cos(zb1hat)
        self.yz2 = zr2hat*np.cos(zb2hat)
        self.yz3 = zr3hat*np.cos(zb3hat)
        

        mx1, my1, mx2, my2, mx3, my3 = self.M[0][0], self.M[0][1],\
         self.M[1][0], self.M[1][1], self.M[2][0], self.M[2][1]

        fig, ax = plt.subplots()
        xtru, ytru, xdata, ydata, thdata, xhist, yhist, xpoint, ypoint = [], [], [], [], [], [], [], [], []
        mx1_data, my1_data, mx2_data, my2_data, mx3_data, my3_data = [], [], [], [], [], []
        plt.axes(xlim=(-10, 10), ylim=(-10, 10))
        plt.plot(mx1,my1,'g^')
        plt.plot(mx2,my2,'g^')
        plt.plot(mx3,my3,'g^')
        plt.plot(self.xhat,self.yhat, 'r')
        robot, = plt.plot([], [], 'ro', markersize=12, animated=True)
        line_hat, = plt.plot([], [], 'y', animated=True)
        line_tru, = plt.plot([], [], 'b', animated=True)
        arrow, = plt.plot([], [], 'k*', markersize=3, animated=True)
        # marker1, = plt.plot([], [], 'r^', markersize=3, animated=True)
        # marker2, = plt.plot([], [], 'r^', markersize=3, animated=True)
        # marker3, = plt.plot([], [], 'r^', markersize=3, animated=True)

        f = np.linspace(-3, 3, 200)

        # initialization function: plot the background of each frame
        def init():
            # ax.set_xlim(-10, 10)
            # ax.set_ylim(-10, 10)
            line_tru.set_data(xtru,ytru)
            line_hat.set_data(xhist,yhist)
            robot.set_data(xdata,ydata)
            arrow.set_data(xpoint, ypoint)
            # marker1.set_data(mx1_data,my1_data)
            # marker2.set_data(mx2_data,my2_data)
            # marker3.set_data(mx3_data,my3_data)
            return robot, line_hat, line_tru, arrow#, marker1, marker2, marker3

        def update(frame):
            i = int(frame)
            xdata = self.xhat[i]
            ydata = self.yhat[i]
            thdata = self.thhat[i]
            r = 1
            xpoint = xdata + r*np.cos(thdata)
            ypoint = ydata + r*np.sin(thdata)
            xhist.append(self.xhat[i])
            yhist.append(self.yhat[i])
            xtru.append(self.xtru[i])
            ytru.append(self.ytru[i])
            robot.set_data(xdata, ydata)
            line_tru.set_data(xtru, ytru)
            line_hat.set_data(xdata, ydata)
            arrow.set_data(xpoint, ypoint)
            # mx1_data = self.xz1[i]
            # my1_data = self.yz1[i]
            # mx2_data = self.xz2[i]
            # my2_data = self.yz2[i]
            # mx3_data = self.xz3[i]
            # my3_data = self.yz3[i]
            # marker1.set_data(mx1_data,my1_data)
            # marker2.set_data(mx2_data,my2_data)
            # marker3.set_data(mx3_data,my3_data)
            return robot, line_hat, line_tru, arrow#, marker1, marker2, marker3

        ani = animation.FuncAnimation(fig, update,
                            init_func=init, frames = 201, interval = 20, blit=True)
        plt.show()

    def plotting(self, Mu, Sig, K, Xtru, t):

        #unpack variables
        x_hat = np.array(Mu)[:,0]
        y_hat = np.array(Mu)[:,1]
        th_hat = np.array(Mu)[:,2]
        sig_x = np.array(Sig)[:,0,0]
        sig_y = np.array(Sig)[:,1,1]
        sig_th = np.array(Sig)[:,2,2]
        xt = np.array(Xtru)[:,0]
        yt = np.array(Xtru)[:,1]
        tht = np.array(Xtru)[:,2]

        #get error
        xe = x_hat - xt
        ye = y_hat - yt
        the = th_hat - tht
        

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

