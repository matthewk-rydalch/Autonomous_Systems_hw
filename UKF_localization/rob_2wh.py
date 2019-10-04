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
        # if no sensory data is given you have to simulate it below.
        # process_variance = np.random.multivariate_normal([0,0], self.R).T
        # truth = model + [[process_variance[0]], [process_variance[1]]]
        # z_r1_tru = self.landmark1-np.array([[x],[y]])
        # z_r2_tru = self.landmark2-np.array([[x],[y]])
        # z_r3_tru = self.landmark3-np.array([[x],[y]])
        dif1x = self.landmark1[0]-x
        dif1y = self.landmark1[1]-y
        # dif2x = self.landmark2[0]-x
        # dif2y = self.landmark2[1]-y
        # dif3x = self.landmark3[0]-x
        # dif3y = self.landmark3[1]-y

        z_r1_tru = math.sqrt(dif1x**2+dif1y**2)
        # z_r2_tru = math.sqrt(dif2x**2+dif2y**2)
        # z_r3_tru = math.sqrt(dif3x**2+dif3y**2)
        z_b1_tru = self.wrap(np.arctan2(dif1y,dif1x))
        # z_b2_tru = np.array([self.wrap(np.arctan2(dif2y,dif2x))])
        # z_b3_tru = np.array([self.wrap(np.arctan2(dif3y,dif3x))])

        # z_r1 = z_r1_tru + np.random.normal(0, self.sig_r)
        # z_r2 = z_r2_tru + np.random.normal(0, self.sig_r)
        # z_r3 = z_r3_tru + np.random.normal(0, self.sig_r)
        # z_b1 = z_b1_tru - th + np.random.normal(0, self.sig_phi)
        # z_b2 = z_b2_tru - th + np.random.normal(0, self.sig_phi)
        # z_b3 = z_b3_tru - th + np.random.normal(0, self.sig_phi)

        # z = [[z_r1],[z_r2],[z_r3],[z_b1],[z_b2],[z_b3]]

        z = np.array([[z_r1_tru],[float(z_b1_tru)]])

        return(z)

    def UKF(self,mu_prev, Sig_prev, ut, zt):

        #general idea Propogate forward a time step, then look at each landmark.
        #if you want to do it just like the books algorithm, propogate forward a time step
        #and look at one landmark.  Switch the landmark each time.
        #To fix this be sure to recalculate sigma points for each look at a landmark.

        v = ut[0]
        w = ut[1]
        M = np.array([[self.a1*v**2+self.a2*w**2, 0],\
            [0, self.a3*v**2+self.a4*w**2]])

        Q = np.array([[self.sig_r**2, 0],\
            [0, self.sig_phi**2]])

        z1x2 = np.zeros((1,2))
        mu_a_prev = np.concatenate((mu_prev, z1x2.T, z1x2.T), axis=0) #make sure this works
        z3x2 = np.zeros((3,2))
        z2x3 = np.zeros((2,3))
        z2x2 = np.zeros((2,2))
        sap1 = np.concatenate((Sig_prev, z3x2, z3x2), axis=1)
        sap2 = np.concatenate((z2x3, M, z2x2), axis=1)
        sap3 = np.concatenate((z2x3,z2x2,Q), axis=1)
        Sig_a_prev = np.concatenate((sap1, sap2, sap3), axis=0)

        ###Be sure that you use the lower triangle for cholesky

        #generate sigma points
        #check slides for this part.  Mean weights sum to one
        n = 7 # #u+#x+#z
        k = 1 # no specific values for k and alpha.  I will use the recommended values on the slides
        alpha = .5 #?
        lam = alpha**2*(n+k)-n
        gm = np.sqrt(n+lam)
        Beta = 2
        #generating sigma point.  we use the cholesky factor here.  It is on the sqrt(Sig_a)
        L = np.linalg.cholesky(Sig_a_prev)
        Xa_prev = np.concatenate((mu_a_prev, mu_a_prev+gm*L,\
                     mu_a_prev-gm*L), axis = 1)
                     #1 column, 7 columns, 7 columns

        #pass sigma points through motion model and compute Guassian
        #adding u to each column of Xs_u.  Each column gets passed in individually
        #there should be a for loop here
        Xs_x_prev = Xa_prev[0:3]
        Xs_u = Xa_prev[3:5]
        Xs_z = Xa_prev[5:7]
        Xs_x_bar = self.propagate(ut,Xs_u,Xs_x_prev)
        mu_bar = np.zeros((3,1))

        #calculate weights
        wm = []
        wc = []
        wm.append(lam/(n+lam))
        wc.append(wm[0]+1-alpha**2+Beta)

        for i in range(1,2*n+1):
            wm.append(1/(2*(n+lam)))
            wc.append(wm[i])

        wm = np.array([wm])
        wc = np.array([[wc]]).T
        mu_bar = (wm@Xs_x_bar).T

        Sig_bar = np.zeros((3,3))
        for i in range(2*n+1):
            Sig_bar = Sig_bar+wc[i]*(np.array([Xs_x_bar[i]]).T-mu_bar)@(np.array([Xs_x_bar[i]]).T-mu_bar).T
        #predict observations at sigma points and compute gaussian statistics
        #h nonlinear measurment model
        #Xs_x_bar by algorithm is only wrtten for 1 marker.  For more this needs to be changed.  Redraw sample points for each landmark
        Zbar = self.sigma_measurements(Xs_x_bar,Xs_z)
        zhat = (wm@Zbar).T
        St = 0
        Sig_xz = np.zeros((3,2))
        for i in range(2*n+1):
            St = St+wc[i]*(np.array([Zbar[i]]).T-zhat).T@(np.array([Zbar[i]]).T-zhat)
        for i in range(2*n+1):
            Sig_xz = Sig_xz + wc[i]*np.array([np.array([Xs_x_bar[i]]).T-mu_bar])@(np.array([Zbar[i]]).T-zhat).T

        Sig_xz = np.squeeze(Sig_xz)

        #measurement update for each landmark is done individually
        #there is a for loop  Be sure to recalculate your sigma points
        #update mean and Covariance
        Kt = Sig_xz/St
        mu_t= mu_bar+Kt@(zt-zhat)
        Sig_t = Sig_bar-Kt*St@Kt.T

        return(mu_t, Sig_t, Kt)
    #

    def propagate(self, ut, Xs_u, state_prev):
        #pass in cholesky requirements and return cholesky_x
        # of the 15 vc and wc, 13 will be the same value

        #extract the states from sigma point model of states

        #propogate state through time as before
        x_prev = np.array([state_prev[0]]).T
        y_prev = np.array([state_prev[1]]).T
        th_prev = np.array([state_prev[2]]).T
        length = len(Xs_u[0])
        xt = []
        yt = []
        tht = []
        vt = []
        wt = []
        Xbar_x = []

        for i in range(length):
            vhat = ut[0]+Xs_u[0][i]
            what = ut[1]+Xs_u[1][i]
            x_new = x_prev[i] - vhat/what*math.sin(th_prev[i])+vhat/what*math.sin(self.wrap(th_prev[i]+what*self.dt))
            y_new = y_prev[i] + vhat/what*math.cos(th_prev[i])-vhat/what*math.cos(self.wrap(th_prev[i]+what*self.dt))
            th_new = self.wrap(th_prev[i] + what*self.dt)
            # [x_new, y_new, th_new, v_new, w_new] = self.vel_motion_model(vhat, what, x_prev[i], y_prev[i], th_prev[i])
            xt.append(x_new)
            yt.append(y_new)
            tht.append(th_new)
            vt.append(vhat)
            wt.append(what)
            Xbar_x.append([[x_new, y_new, th_new]])
        set_trace()

        Xbar_x = np.array(Xbar_x).T
        Xbar_x = np.squeeze(Xbar_x)

        return(Xbar_x)
    #

    def sigma_measurements(self, Xbar_x, xs_z):

        xs_z = np.squeeze(xs_z)
        Zbar = np.zeros((15,2))
        #!!!!!!!!!start here and also add noise back into sigma_measurement
    def simulate_sensor(self, x, y, th):
        dif1x = self.landmark1[0]-x
        dif1y = self.landmark1[1]-y
        z_r1_tru = math.sqrt(dif1x**2+dif1y**2)
        z_b1_tru = self.wrap(np.arctan2(dif1y,dif1x))
        z = np.array([[z_r1_tru],[float(z_b1_tru)]])
        Zbar1 = self.simulate_sensor(Xbar_x[0][0],Xbar_x[0][1],Xbar_x[0][2])
        Zbar[0,:] = np.array(Zbar1+np.array([xs_z[:,0]]).T).T

        for i in range(1,15):

            Zbar1 = self.simulate_sensor(Xbar_x[i][0],Xbar_x[i][1],Xbar_x[i][2])
            Zbar[i,:] = np.array(Zbar1+np.array([xs_z[:,0]]).T).T
        return Zbar

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
