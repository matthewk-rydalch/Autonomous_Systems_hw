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

class Visualizer:
    def __init__(self, Mtr, xgrid, ygrid):
        self.Mtr = Mtr
        self.xgrid = xgrid
        self.ygrid = ygrid
    def animator(self, Xtru, Mu, sig_hist, m_hist, elements):

        elipse_points = 100
        self.time_len = len(m_hist)
        self.marker_len = int(len(m_hist[0]))

        self.xtru = np.array(Xtru)[:,0]
        self.ytru = np.array(Xtru)[:,1]
        self.thtru = np.array(Xtru)[:,2]
        self.xhat = np.array(Mu)[:,0]
        self.yhat = np.array(Mu)[:,1]
        self.thhat = np.array(Mu)[:,2]

        #plotting covariance info
        #time, marker, elipse points
        self.xcov = np.zeros((self.time_len,self.marker_len,elipse_points))
        self.ycov = np.zeros((self.time_len,self.marker_len, elipse_points))
        for k in range(self.time_len):
            # sig_end = sig_hist[k]
            # sig_mark = sig_end[0:len(sig_end)]
            # sig_mark = sig_mark[0:len(sig_mark)]
            sig_mark = sig_hist[k]
            for i in range(self.marker_len):
                cov_mark = sig_mark[i]
                U, S, vh = np.linalg.svd(cov_mark) #don't need vh
                S = np.diag(S)
                C = U@np.sqrt(S)
                theta = np.linspace(0, 2*np.pi, elipse_points)
                circle =np.array([[np.cos(theta)],[np.sin(theta)]])
                elps = C@np.squeeze(circle)
                self.xcov[k][i] = elps[0,:]+m_hist[k][i][0][0]
                self.ycov[k][i] = elps[1,:]+m_hist[k][i][0][1]



        fig = plt.figure()
        plt.axes(xlim=(self.xgrid), ylim=(self.ygrid))

        ###static plots###
        plt.plot(self.xhat,self.yhat, 'y')
        plt.plot(self.xtru, self.ytru, 'b')
        for i in range(len(self.Mtr)):
            plt.plot(self.Mtr[i][0],self.Mtr[i][1],'g^')

        ### animation ###
        plt.plot(self.xhat,self.yhat, 'r')
        robot, = plt.plot([], [], 'ro', markersize=12, animated=True)
        arrow, = plt.plot([], [], 'k*', markersize=3, animated=True)
        cov = []
        for i in range(self.marker_len):
            next_cov, = plt.plot([],[], 'c', animated=True)
            cov.append(next_cov)

        # initialization function: plot the background of each frame
        def init():
            robot.set_data([],[])
            arrow.set_data([], [])
            for i in range(self.marker_len):
                cov[i].set_data([], [])
            return (robot, arrow) + tuple(cov)

        def update(frame):
            i = int(frame)
            r = 1
            xpoint = self.xhat[i] + r*np.cos(self.thhat[i])
            ypoint = self.yhat[i] + r*np.sin(self.thhat[i])
            robot.set_data(self.xhat[i], self.yhat[i])
            arrow.set_data(xpoint, ypoint)
            for j in range(self.marker_len):
                cov[j].set_data(self.xcov[i][j],self.ycov[i][j])
            return (robot, arrow) + tuple(cov)

        ani = animation.FuncAnimation(fig, update,
                            init_func=init, frames = self.time_len, interval = 20, blit=True)
        plt.show()

    def plotting(self, Mu, Sig, Xtru, m_hist, t):

        #unpack variables
        x_hat = np.array(Mu)[:,0]
        y_hat = np.array(Mu)[:,1]
        th_hat = np.array(Mu)[:,2]
        sig_x = np.array(Sig)[:,0,0]
        sig_y = np.array(Sig)[:,1,1]
        # sig_th = np.array(Sig)[:,2,2]
        xt = np.array(Xtru)[:,0]
        yt = np.array(Xtru)[:,1]
        tht = np.array(Xtru)[:,2]
        m_hist = np.reshape(np.squeeze(np.array(m_hist)),(len(m_hist),len(self.Mtr),2))


        #get error
        xe = x_hat - xt
        ye = y_hat - yt
        the = th_hat - tht


        # #calculate upper and lower bounds for covariance plot
        # sigx_hi = 2*np.sqrt(sig_x)
        # sigx_lo = -2*np.sqrt(sig_x)
        # sigy_hi = 2*np.sqrt(sig_y)
        # sigy_lo = -2*np.sqrt(sig_y)
        # sigth_hi = 2*np.sqrt(sig_th)
        # sigth_lo = -2*np.sqrt(sig_th)

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

        # mtr = self.Mtr
        # size = len(m_hist[0])
        # fig2, aXk = plt.subplots(size)
        # fig2.suptitle("marker locations")

        # for i in range(size):
        #     aXk[i].plot(t, m_hist[:,i,0], label = "xhat")
        #     aXk[i].plot(t, m_hist[:,i,1], label = "yhat")
        #     aXk[i].plot([t[0],t[len(m_hist)-1]], [mtr[i][0],mtr[i][0]], label = "x")
        #     aXk[i].plot([t[0],t[len(m_hist)-1]], [mtr[i][1],mtr[i][1]], label = "y")
        # aXk[i].legend(loc = "upper right")
        # aXk[i].set_xlabel('time(s)')
        # fig2.show()

        # fig2, aXk = plt.subplots(3)
        # fig2.suptitle("Covariance & Error vs. Time")
        # aXk[0].plot(t,xe, label="x error [m]")
        # aXk[0].plot(t,sigx_hi, label="upper covariance")
        # aXk[0].plot(t,sigx_lo, label="lower covariance")
        # aXk[0].legend(loc = "upper right")
        # aXk[1].plot(t,ye, label="y error [m]")
        # aXk[1].plot(t,sigy_hi, label="upper covariance")
        # aXk[1].plot(t,sigy_lo, label="lower covariance")
        # aXk[1].legend(loc = "upper right")
        # aXk[2].plot(t,the, label="theta error [rad]")
        # aXk[2].plot(t,sigth_hi, label="upper covariance")
        # aXk[2].plot(t,sigth_lo, label="lower covariance")
        # aXk[2].legend(loc = "upper right")
        # aXk[2].set_xlabel('time [s]')
        # fig2.show()
    #
