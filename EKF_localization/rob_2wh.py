import numpy as np
import scipy
import scipy.signal
import random
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import scipy.io as sio
from IPython.core.debugger import set_trace
from numpy.linalg import inv

class rob_2wh:
    def __init__(self):
        self.a1 = 0.1
        self.a2 = 0.01
        self.a3 = 0.01
        self.a4 = 0.1

        self.landmark1 = np.array([[6], [4]])
        self.landmark2 = np.array([[-7], [8]])
        self.landmark3 = np.array([[6], [-4]])

        # self.landmark1 = np.array([[-8], [4]])
        # self.landmark2 = np.array([[-7], [8]])
        # self.landmark3 = np.array([[-5], [-4]])

        #sensor noise
        self.sig_r = 0.1
        self.sig_phi = 0.05 #rad
        # self.sig_r = 0.000000001
        # self.sig_phi = 50

        self.dt = 0.1
        self.tf = 20

        self.x0 = -5
        self.y0 = -3
        self.th0 = math.pi/2 #rad

        given = sio.loadmat('hw2_soln_data.mat')
        self.wtr = given['om']
        self.ttr = given['t']
        self.thtr = given['th']
        self.vtr = given['v']
        self.xtr = given['x']
        self.ytr = given['y']

    def vel_motion_model(self, vc, wc, x, y, th):

        v_hat = vc + np.random.normal(0, (self.a1*vc**2+self.a2*wc**2))
        w_hat = wc + np.random.normal(0, (self.a3*vc**2+self.a4*wc**2))

        x1 = x - v_hat/w_hat*math.sin(th)+v_hat/w_hat*math.sin(self.wrap(th+w_hat*self.dt))
        y1 = y + v_hat/w_hat*math.cos(th)-v_hat/w_hat*math.cos(self.wrap(th+w_hat*self.dt))
        th1 = self.wrap(th + w_hat*self.dt)

        return x1, y1, th1, v_hat, w_hat

    def animation(self, x, y, th):
        xm = -10
        xM = 10
        ym = -10
        yM = 10

        # First set up the figure, the axis, and the plot element we want to animate
        fig = plt.figure()
        ax = plt.axes(xlim=(0, 2), ylim=(-2, 2))
        line, = ax.plot([], [], lw=2)

        # call the animator.  blit=True means only re-draw the parts that have changed.
        anim = animation.FuncAnimation(fig, self.animate, init_func=self.animation_init(line),
                               frames=200, interval=20, blit=True)

        plt.show()

    # initialization function: plot the background of each frame
    def animation_init(self, line):
        line.set_data([], [])
        return line,

    def animate(self, i):
        x = np.linspace(0, 2, 1000)
        y = np.sin(2 * np.pi * (x - 0.01 * i))
        line.set_data(x, y)
        return line,

    def simulate_sensor(self, x, y, th):
        dif1x = self.landmark1[0]-x
        dif1y = self.landmark1[1]-y
        dif2x = self.landmark2[0]-x
        dif2y = self.landmark2[1]-y
        dif3x = self.landmark3[0]-x
        dif3y = self.landmark3[1]-y

        z_r1_tru = math.sqrt(dif1x**2+dif1y**2)
        z_r2_tru = math.sqrt(dif2x**2+dif2y**2)
        z_r3_tru = math.sqrt(dif3x**2+dif3y**2)
        z_b1_tru = np.array([self.wrap(np.arctan2(dif1y,dif1x))])
        z_b2_tru = np.array([self.wrap(np.arctan2(dif2y,dif2x))])
        z_b3_tru = np.array([self.wrap(np.arctan2(dif3y,dif3x))])

        z_r1 = z_r1_tru + np.random.normal(0, self.sig_r)
        z_r2 = z_r2_tru + np.random.normal(0, self.sig_r)
        z_r3 = z_r3_tru + np.random.normal(0, self.sig_r)
        z_b1 = z_b1_tru -th + np.random.normal(0, self.sig_phi)
        z_b2 = z_b2_tru -th + np.random.normal(0, self.sig_phi)
        z_b3 = z_b3_tru -th + np.random.normal(0, self.sig_phi)

        z = [[z_r1],[z_r2],[z_r3],[z_b1],[z_b2],[z_b3]]

        return(z)

    def EKF(self,mu_prev, Sig_prev, u, z):


        [mu_bar, Sig_bar, G, V, M] = self.propagate(mu_prev, Sig_prev, u)

        Q = [[self.sig_r**2, 0],\
            [0, self.sig_phi**2]]

        m = np.array([self.landmark1, self.landmark2, self.landmark3])

        z_hathist = np.array([])
        Hhist = np.array([])
        Shist = np.array([])
        Khist = []
        c = [0, 1, 2]
        # c = [0, 1]
        # c = [0]
        for  i in range(3):
            j = c[i]

            q = (m[j][0]-mu_bar[0])**2+(m[j][1]-mu_bar[1])**2

            zhat1 = math.sqrt(q)
            zhat2 = self.wrap(np.arctan2((m[j][1]-mu_bar[1]),(m[j][0]-mu_bar[0])))
            zhat3 = mu_bar[2]
            z_hat = np.array([[float(zhat1)],[float(self.wrap(zhat2-zhat3))]])
            # z_hat = np.array([[float(zhat1)],[float(zhat2)]])

            # print('zhat = ', z_hat)
            # z_hat = np.array([math.sqrt(q)],\
            #             [math.atan2((m[j][1]-mu_bar[1]),(m[j][0]-mu_bar[0]))-mu_bar[2]])
            H = np.array([[float(-(m[j][0]-mu_bar[0])/(math.sqrt(q))),float(-(m[j][1]-mu_bar[1])/(math.sqrt(q))),0],\
                          [float((m[j][1]-mu_bar[1])/q),float(-(m[j][0]-mu_bar[0])/q),-1]])

            z_now = np.array([[float(z[j])], [float(z[j+3])]])

            S = np.array(H@Sig_bar@H.T+Q)
            K = np.array(Sig_bar@H.T@inv(S))
            mu_bar = np.array(mu_bar+K@(self.wrap(z_now-z_hat)))
            Sig_bar = np.array((np.eye(3)-K@H)@Sig_bar)

            Khist.append(K)

        mu = mu_bar
        Sig = Sig_bar

        return(mu, Sig, Khist)
    #

    def propagate(self, mu_prev, Sig_prev, u):

        xp = mu_prev[0]
        yp = mu_prev[1]
        thp = mu_prev[2]

        v = u[0]
        w = u[1]
        dt = self.dt

        G = np.array([[1, 0, -v/w*math.cos(thp)+v/w*math.cos(self.wrap(thp+w*dt))],\
            [0, 1, -v/w*math.sin(thp)+v/w*math.sin(self.wrap(thp+w*dt))],\
            [0, 0, 1]])

        V = np.array([[1/w*(-math.sin(thp)+math.sin(self.wrap(thp+w*dt))), v/(w**2)*(math.sin(thp)-math.sin(self.wrap(thp+w*dt)))+v/w*(math.cos(self.wrap(thp+w*dt))*dt)],\
            [1/w*(math.cos(thp)-math.cos(self.wrap(thp+w*dt))), -v/(w**2)*(math.cos(thp)-math.cos(self.wrap(thp+w*dt)))+v/w*(math.sin(self.wrap(thp+w*dt))*dt)],\
            [0, dt]])

        M = np.array([[self.a1*v**2+self.a2*w**2, 0],\
            [0, self.a3*v**2+self.a4*w**2]])

        mu_bar = mu_prev + [[-v/w*math.sin(thp)+v/w*math.sin(self.wrap(thp+w*dt))],\
                            [v/w*math.cos(thp)-v/w*math.cos(self.wrap(thp+w*dt))],\
                            [w*dt]]

        # set_trace()
        Sig_bar = G@Sig_prev*G.T+V@M@V.T

        return(mu_bar, Sig_bar, G, V, M)
    #

    def wrap(self, phi):
        phi_new = (phi+np.pi)%(2*np.pi)-np.pi
        # phi_new = phi
        return phi_new

    def plotting(self,x_hat, x, y_hat, y, th_hat, th, vc, v, wc, w, t, xe, ye, the, ve, we, K, sig_x, sig_y, sig_th):

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

        fig1, axs = plt.subplots(3)
        fig1.suptitle("x, y, and theta vs. Time")
        axs[0].plot(t, x_hat, label = "x [m]")
        axs[0].plot(t, x, label = "true x [m]")
        axs[0].legend(loc = "upper right")
        axs[1].plot(t, y_hat, label="y [m]")
        axs[1].plot(t, y, label="true y [m]")
        axs[1].legend(loc = "upper right")
        axs[2].plot(t, th_hat, label = "theta [rad]")
        axs[2].plot(t, th, label = "true theta [rad]")
        axs[2].legend(loc = "upper right")
        axs[2].set_xlabel('time [s]')
        fig1.show()

        fig2, axs = plt.subplots(2)
        fig2.suptitle("velocity and angular vs. Time")
        axs[0].plot(t, vc, label = "v [m/s]")
        axs[0].plot(t, v, label = "true v [m/s]")
        axs[0].legend(loc = "upper right")
        axs[1].plot(t, wc, label="w [m/s]")
        axs[1].plot(t, w, label="true w [m/s]")
        axs[1].set_xlabel('time [s]')
        fig2.show()

        fig3, axs = plt.subplots(3)
        fig3.suptitle("Covariance & Error vs. Time")
        axs[0].plot(t,xe, label="x error [m]")
        axs[0].plot(t,sigx_hi, label="upper covariance")
        axs[0].plot(t,sigx_lo, label="lower covariance")
        axs[0].legend(loc = "upper right")
        axs[1].plot(t,ye, label="y error [m]")
        axs[1].plot(t,sigy_hi, label="upper covariance")
        axs[1].plot(t,sigy_lo, label="lower covariance")
        axs[1].legend(loc = "upper right")
        axs[2].plot(t,the, label="theta error [rad]")
        axs[2].plot(t,sigth_hi, label="upper covariance")
        axs[2].plot(t,sigth_lo, label="lower covariance")
        axs[2].legend(loc = "upper right")
        axs[2].set_xlabel('time [s]')
        fig3.show()

        fig5 = plt.figure(5)
        fig5.suptitle("Kalman Gains vs. Time")
        plt.plot(t, k1r, label="Marker 1 range Kalman gain")
        plt.plot(t, k1b, label="Marker 1 bearing Kalman gain")
        plt.plot(t, k2r, label="Marker 2 range Kalman gain")
        plt.plot(t, k2b, label="Marker 2 bearing Kalman gain")
        plt.plot(t, k3r, label="Marker 3 range Kalman gain")
        plt.plot(t, k3b, label="Marker 3 bearing Kalman gain")
        plt.legend(loc = "upper right")

        fig5.show()
    #
