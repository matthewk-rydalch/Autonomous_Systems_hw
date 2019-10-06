import numpy as np
from importlib import reload
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
import rob_2wh
reload(rob_2wh)
from rob_2wh import Rob2Wh


class Monte_Carlo:
    def UKF(self,mu_prev, Sig_prev, ut, zt, marker):
        self.rob = Rob2Wh()
        v = ut[0]
        w = ut[1]
        M = np.array([[self.rob.a1*v**2+self.rob.a2*w**2, 0],\
            [0, self.rob.a3*v**2+self.rob.a4*w**2]])

        Q = np.array([[self.rob.sig_r**2, 0],\
            [0, self.rob.sig_phi**2]])

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
        Xa_prev = np.concatenate((mu_a_prev, mu_a_prev+self.rob.gm*L,\
                     mu_a_prev-self.rob.gm*L), axis = 1)

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
        wm.append(self.rob.lam/(self.rob.n+self.rob.lam))
        wc.append(wm[0]+1-self.rob.alpha**2+self.rob.Beta)

        for i in range(1,2*self.rob.n+1):
            wm.append(1/(2*(self.rob.n+self.rob.lam)))
            wc.append(wm[i])

        wm = np.array([wm])
        wc = np.array([[wc]]).T
        if marker == 0:
            Xk_x_bar = self.propagate(ut,Xk_u,Xk_x_prev)

            mu_bar = np.zeros((3,1))
            mu_bar = (wm@Xk_x_bar.T).T
            mu_bar[2] = wrap(mu_bar[2])


            Sig_bar = np.zeros((3,3))
            for i in range(2*self.rob.n+1):
                term2 = np.array([Xk_x_bar[:,i]]).T-mu_bar
                term2[2] = wrap(term2[2])
                Sig_bar = Sig_bar+wc[i]*term2@term2.T
            # set_trace()
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
        for i in range(2*self.rob.n+1):
            term1 = np.array([Zbar[i]]).T-zhat
            term1[1] = wrap(term1[1])
            St = St+wc[i]*term1@term1.T
        for i in range(2*self.rob.n+1):
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
            x_new = x_prev[i] - vhat/what*math.sin(th_prev[i])+vhat/what*math.sin(wrap(th_prev[i]+what*self.rob.dt))
            y_new = y_prev[i] + vhat/what*math.cos(th_prev[i])-vhat/what*math.cos(wrap(th_prev[i]+what*self.rob.dt))
            th_new = wrap(th_prev[i] + what*self.rob.dt)
            # [x_new, y_new, th_new, v_new, w_new] = self.rob.vel_motion_model(vhat, what, x_prev[i], y_prev[i], th_prev[i])
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
        # z_meas = self.rob.simulate_sensor(Xbar_x[0][0],Xbar_x[1][0],Xbar_x[2][0], noise)
        # Z_bar1 = np.array([z_meas[marker],z_meas[marker+3]])
        # Z_bar[0,:] = np.array(Z_bar1+np.array([Xk_z[:,0]]).T).T
        for i in range(15):
            z_meas = self.rob.simulate_sensor(Xbar_x[0][i],Xbar_x[1][i],Xbar_x[2][i], noise)
            Z_bar1 = np.array([z_meas[marker],z_meas[marker+3]])
            Z_bar[i,:] = np.array(Z_bar1+np.array([Xk_z[:,i]]).T).T
            Z_bar[i,1] = wrap(Z_bar[i,1])
        return Z_bar

        # def wrap(self, phi):
        #     phi_new = (phi+np.pi)%(2*np.pi)-np.pi
        #     # phi_new = phi
        #     return phi_new

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
