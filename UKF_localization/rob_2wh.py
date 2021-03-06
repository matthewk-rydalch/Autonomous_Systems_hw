import numpy as np
import scipy
import scipy.signal
import random
import math
import matplotlib.pyplot as plt
from matplotlib import animation
from IPython.core.debugger import set_trace
from numpy.linalg import inv
from numpy.linalg import cholesky
import scipy.io as sio
# import lmd
# import lmd.rad_wrap_pi
# import lmd.rad_wrap_pi as wrap
import pandas as pd
desired_width=320
pd.set_option('display.width', desired_width)
np.set_printoptions(linewidth=desired_width)
pd.set_option('display.max_columns',10)



class Rob2Wh:
    def __init__(self):
        # parameters
        self.a1 = 0.1
        self.a2 = 0.01
        self.a3 = 0.01
        self.a4 = 0.1

        # self.a1 = 0.5
        # self.a2 = 0.02
        # self.a3 = 0.005
        # self.a4 = 0.15

        self.landmark1 = np.array([[6], [4]])
        self.landmark2 = np.array([[-7], [8]])
        self.landmark3 = np.array([[6], [-4]])

        # self.landmark1 = np.array([[-8], [4]])
        # self.landmark2 = np.array([[-7], [8]])
        # self.landmark3 = np.array([[-5], [-4]])

        #sensor noise
        self.sig_r = 0.1
        self.sig_phi = 0.05 #rad
        # self.sig_r = 0.08
        # self.sig_phi = .12

        self.dt = 0.1
        self.tf = 20

        self.x0 = -5
        self.y0 = -3
        self.th0 = math.pi/2 #rad

        #generate sigma points
        self.n = 7 # #u+#x+#z

        # k = 1 # no specific values for k and alpha.  I will use the recommended values on the slides
        # alpha = .5 #?
        # Beta = 2
        self.k = 4 # no specific values for k and alpha.  I will use the recommended values on the slides
        self.alpha = .4 #?
        self.Beta = 2

        self.lam = self.alpha**2*(self.n+self.k)-self.n
        self.gm = np.sqrt(self.n+self.lam)

        # given = sio.loadmat('hw3_1_soln_data.mat') #1 marker
        given = sio.loadmat('hw3_5_soln_data.mat') #3 markers
        # self.z_btr = given['bearing'] #only for 1 marker
        # self.z_rtr = given['range'] #only for 1 marker
        self.wtr = given['om']
        self.ttr = given['t']
        self.thtr = given['th']
        self.vtr = given['v']
        self.xtr = given['x']
        self.ytr = given['y']



    def vel_motion_model(self, vc, wc, x, y, th):

        v_hat = vc + np.random.normal(0, np.sqrt(self.a1*vc**2+self.a2*wc**2))
        w_hat = wc + np.random.normal(0, np.sqrt(self.a3*vc**2+self.a4*wc**2))

        x1 = x - v_hat/w_hat*math.sin(th)+v_hat/w_hat*math.sin(wrap(th+w_hat*self.dt))
        y1 = y + v_hat/w_hat*math.cos(th)-v_hat/w_hat*math.cos(wrap(th+w_hat*self.dt))
        th1 = wrap(th + w_hat*self.dt)

        return x1, y1, th1, v_hat, w_hat

    def simulate_sensor(self, x, y, th, noise = 1):

        dif1x = self.landmark1[0]-x
        dif1y = self.landmark1[1]-y
        dif2x = self.landmark2[0]-x
        dif2y = self.landmark2[1]-y
        dif3x = self.landmark3[0]-x
        dif3y = self.landmark3[1]-y

        z_r1_tru = math.sqrt(dif1x**2+dif1y**2)
        z_r2_tru = math.sqrt(dif2x**2+dif2y**2)
        z_r3_tru = math.sqrt(dif3x**2+dif3y**2)
        z_b1_tru = np.arctan2(dif1y,dif1x)
        z_b2_tru = np.arctan2(dif2y,dif2x)
        z_b3_tru = np.arctan2(dif3y,dif3x)

        z_r1 = z_r1_tru + noise*np.random.normal(0, self.sig_r)
        z_r2 = z_r2_tru + noise*np.random.normal(0, self.sig_r)
        z_r3 = z_r3_tru + noise*np.random.normal(0, self.sig_r)
        z_b1 = wrap(z_b1_tru - th + noise*np.random.normal(0, self.sig_phi))
        z_b2 = wrap(z_b2_tru - th + noise*np.random.normal(0, self.sig_phi))
        z_b3 = wrap(z_b3_tru - th + noise*np.random.normal(0, self.sig_phi))

        z = np.array([[z_r1],[z_r2],[z_r3],[float(z_b1)],[float(z_b2)],[float(z_b3)]])

        return(z)

    def UKF(self,mu_prev, Sig_prev, ut, zt, marker):
        v = ut[0]
        w = ut[1]
        M = np.array([[self.a1*v**2+self.a2*w**2, 0],\
            [0, self.a3*v**2+self.a4*w**2]])

        Q = np.array([[self.sig_r**2, 0],\
            [0, self.sig_phi**2]])

        z1x2 = np.zeros((1,2))
        mu_a_prev = np.concatenate((mu_prev, z1x2.T, z1x2.T), axis=0)
        z3x2 = np.zeros((3,2))
        z2x3 = np.zeros((2,3))
        z2x2 = np.zeros((2,2))
        sap1 = np.concatenate((Sig_prev, z3x2, z3x2), axis=1)
        sap2 = np.concatenate((z2x3, M, z2x2), axis=1)
        sap3 = np.concatenate((z2x3,z2x2,Q), axis=1)
        Sig_a_prev = np.concatenate((sap1, sap2, sap3), axis=0)

        #generating sigma point.  we use the cholesky factor here.
        L = np.linalg.cholesky(Sig_a_prev)
        Xa_prev = np.concatenate((mu_a_prev, mu_a_prev+self.gm*L,\
                     mu_a_prev-self.gm*L), axis = 1)

        Xa_prev[2] = wrap(Xa_prev[2])
        Xa_prev[6] = wrap(Xa_prev[6])

        #pass sigma points through motion model and compute Guassian
        #adding u to each column of Xk_u.  Each column gets passed in individually
        #there should be a for loop here
        Xk_x_prev = Xa_prev[0:3]
        Xk_u = Xa_prev[3:5]
        Xk_z = Xa_prev[5:7]

        #calculate weights
        wm = []
        wc = []
        wm.append(self.lam/(self.n+self.lam))
        wc.append(wm[0]+1-self.alpha**2+self.Beta)

        for i in range(1,2*self.n+1):
            wm.append(1/(2*(self.n+self.lam)))
            wc.append(wm[i])

        wm = np.array([wm])
        wc = np.array([[wc]]).T
        if marker == 0:
            Xk_x_bar = self.propagate(ut,Xk_u,Xk_x_prev)

            mu_bar = np.zeros((3,1))
            mu_bar = (wm@Xk_x_bar.T).T
            mu_bar[2] = wrap(mu_bar[2])


            Sig_bar = np.zeros((3,3))
            for i in range(2*self.n+1):
                term2 = np.array([Xk_x_bar[:,i]]).T-mu_bar
                term2[2] = wrap(term2[2])
                Sig_bar = Sig_bar+wc[i]*term2@term2.T

        else:
            Xk_x_bar = Xk_x_prev
            mu_bar = mu_prev
            Sig_bar = Sig_prev
            # mu2 = np.zeros((3,1))
            # mu2 = (wm@Xk_x_bar.T).T
            # Sig2 = np.zeros((3,3))
            # for i in range(2*n+1):
            #     Sig2 = Sig2 +wc[i]*(np.array([Xk_x_bar[:,i]]).T-mu2)@(np.array([Xk_x_bar[:,i]]).T-mu2).T

        Zbar = self.sigma_measurements(Xk_x_bar,Xk_z, marker)
        zhat = (wm@Zbar).T
        zhat[1] = wrap(zhat[1])
        St = np.zeros((2,2))
        Sig_xz = np.zeros((3,2))
        for i in range(2*self.n+1):
            term1 = np.array([Zbar[i]]).T-zhat
            term1[1] = wrap(term1[1])
            St = St+wc[i]*term1@term1.T
        for i in range(2*self.n+1):
            Sig_xz = Sig_xz + wc[i]*np.array([np.array([Xk_x_bar[:,i]]).T-mu_bar])@(np.array([Zbar[i]]).T-zhat).T
        Sig_xz = np.squeeze(Sig_xz)

        #measurement update for each landmark is done individually
        #there is a for loop  Be sure to recalculate your sigma points
        #update mean and Covariance
        Kt = Sig_xz@inv(St)
        z_marker = np.array([zt[marker],zt[marker+3]])
        term1 = z_marker-zhat
        term1[1] = wrap(term1[1])
        mu_t= mu_bar+Kt@term1
        mu_t[2] = wrap(mu_t[2])
        Sig_t = Sig_bar-Kt@St@Kt.T
        return(mu_t, Sig_t, Kt)
    #

    def propagate(self, ut, Xk_u, state_prev):
        #pass in cholesky requirements and return cholesky_x
        # of the 15 vc and wc, 13 will be the same value

        #extract the states from sigma point model of states

        #propogate state through time as before
        x_prev = np.array([state_prev[0]]).T
        y_prev = np.array([state_prev[1]]).T
        th_prev = np.array([state_prev[2]]).T
        length = len(Xk_u[0])
        xt = []
        yt = []
        tht = []
        vt = []
        wt = []
        Xbar_x = []

        for i in range(length):
            vhat = ut[0]+Xk_u[0][i]
            what = ut[1]+Xk_u[1][i]
            x_new = x_prev[i] - vhat/what*math.sin(th_prev[i])+vhat/what*math.sin(wrap(th_prev[i]+what*self.dt))
            y_new = y_prev[i] + vhat/what*math.cos(th_prev[i])-vhat/what*math.cos(wrap(th_prev[i]+what*self.dt))
            th_new = wrap(th_prev[i] + what*self.dt)
            # [x_new, y_new, th_new, v_new, w_new] = self.vel_motion_model(vhat, what, x_prev[i], y_prev[i], th_prev[i])
            xt.append(x_new)
            yt.append(y_new)
            tht.append(th_new)
            vt.append(vhat)
            wt.append(what)
            Xbar_x.append([[x_new, y_new, th_new]])

        Xbar_x = np.array(Xbar_x).T
        Xbar_x = np.squeeze(Xbar_x)

        return(Xbar_x)
    #

    def sigma_measurements(self, Xbar_x, Xk_z, marker):
        Xk_z = np.squeeze(Xk_z)
        Z_bar = np.zeros((15,2))
        noise = 0
        # z_meas = self.simulate_sensor(Xbar_x[0][0],Xbar_x[1][0],Xbar_x[2][0], noise)
        # Z_bar1 = np.array([z_meas[marker],z_meas[marker+3]])
        # Z_bar[0,:] = np.array(Z_bar1+np.array([Xk_z[:,0]]).T).T
        for i in range(15):
            z_meas = self.simulate_sensor(Xbar_x[0][i],Xbar_x[1][i],Xbar_x[2][i], noise)
            Z_bar1 = np.array([z_meas[marker],z_meas[marker+3]])
            Z_bar[i,:] = np.array(Z_bar1+np.array([Xk_z[:,i]]).T).T
            Z_bar[i,1] = wrap(Z_bar[i,1])
        return Z_bar

    # def wrap(self, phi):
    #     phi_new = (phi+np.pi)%(2*np.pi)-np.pi
    #     # phi_new = phi
    #     return phi_new

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

        fig1, aXk = plt.subplots(3)
        fig1.suptitle("x, y, and theta vs. Time")
        aXk[0].plot(t, x_hat, label = "x [m]")
        aXk[0].plot(t, x, label = "true x [m]")
        aXk[0].legend(loc = "upper right")
        aXk[1].plot(t, y_hat, label="y [m]")
        aXk[1].plot(t, y, label="true y [m]")
        aXk[1].legend(loc = "upper right")
        aXk[2].plot(t, th_hat, label = "theta [rad]")
        aXk[2].plot(t, th, label = "true theta [rad]")
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

        fig3 = plt.figure(5)
        fig3.suptitle("Kalman Gains vs. Time")
        plt.plot(t, k1r, label="Marker 1 range Kalman gain")
        plt.plot(t, k1b, label="Marker 1 bearing Kalman gain")
        plt.plot(t, k2r, label="Marker 2 range Kalman gain")
        plt.plot(t, k2b, label="Marker 2 bearing Kalman gain")
        plt.plot(t, k3r, label="Marker 3 range Kalman gain")
        plt.plot(t, k3b, label="Marker 3 bearing Kalman gain")
        plt.legend(loc = "upper right")

        fig3.show()
    #
r2d = 180.0 / np.pi
d2r = np.pi / 180.0
inv_360 = 1.0 / 360.0
inv_180 = 1.0 / 180.0
inv_pi = 1.0 / np.pi
inv_2pi = 0.5 / np.pi

def deg_wrap_180( angle ):
    """wrap an angle in degrees, -180 <= theta < 180"""
    angle -= 360.0 * np.floor((angle + 180.) * inv_360)
    return angle
#
def deg_wrap_360( angle ):
    """wrap an angle in degrees, 0 <= theta < 360"""
    angle -= 360.0 * np.floor(angle * inv_360)
    return angle
#

# def rad_wrap_pi( angle ):
def wrap( angle ):
    """wrap an angle in rads, -pi <= theta < pi"""
    # angle -= 2*np.pi * np.floor((angle + np.pi) * inv_2pi)
    return angle
#
def rad_wrap_2pi( angle ):
    """wrap an angle in rads, 0 <= theta < 2*pi"""
    angle -= 2*np.pi * np.floor(angle * inv_2pi)
    return angle
#
