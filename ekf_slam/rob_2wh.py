import numpy as np
import math
import random
from IPython.core.debugger import set_trace
from importlib import reload
import utils
reload(utils)
from utils import wrap


class Rob2Wh:
    def __init__(self, dt, alpha, M, sig_r, sig_phi):
        self.dt = dt
        self.alpha = alpha
        self.M = M
        self.sig_r = sig_r
        self.sig_phi = sig_phi
    #

    def vel_motion_model(self, Ut, Mup, noise = 1):

        #commands and states
        vc = Ut[0]
        wc = Ut[1]
        xt = Mup[0]
        yt = Mup[1]
        tht = Mup[2]

        size = len(xt)

        #Add noise to commands
        v_hat = vc + noise*np.random.normal(0, np.sqrt(self.alpha[0]*vc**2+self.alpha[1]*wc**2),size)
        w_hat = wc + noise*np.random.normal(0, np.sqrt(self.alpha[2]*vc**2+self.alpha[3]*wc**2),size)
        #include a gamma that accounts for imperfect heading
        gama = noise*np.random.normal(0, np.sqrt(self.alpha[4]*vc**2+self.alpha[5]*wc**2),size)
        #propagate states
        x_new = xt - v_hat/w_hat*np.sin(tht)+v_hat/w_hat*np.sin(wrap(tht+w_hat*self.dt))
        y_new = yt + v_hat/w_hat*np.cos(tht)-v_hat/w_hat*np.cos(wrap(tht+w_hat*self.dt))
        th_new = wrap(tht + w_hat*self.dt + gama*self.dt)

        Xt = np.array([x_new,y_new,th_new])

        return Xt
    #

    def model_sensor(self, Mup, noise = 1):
        #states
        xt = Mup[0,:]
        yt = Mup[1,:]
        tht = Mup[2,:]

        #range components w/o sensor noise
        difx = self.M[:,0]-xt
        dify = self.M[:,1]-yt

        #range and bearing w/o sensor noise/truth
        zr_tru = np.sqrt(difx**2+dify**2)
        zb_tru = np.arctan2(dify,difx)

        #add in sensor noise if noise is not specified as 0
        zr = zr_tru + noise*np.random.normal(0, self.sig_r)
        zb = utils.wrap(zb_tru - tht + noise*np.random.normal(0, self.sig_phi))

        z = np.array([zr,zb])

        return(z)
    # def model_sensor(self, state, noise = 1):
    #     #states
    #     xt = state[0,:]
    #     yt = state[1,:]
    #     tht = state[2,:]

    #     #range components w/o sensor noise
    #     dif1x = self.M[0][0]-xt
    #     dif1y = self.M[0][1]-yt
    #     dif2x = self.M[1][0]-xt
    #     dif2y = self.M[1][1]-yt
    #     dif3x = self.M[2][0]-xt
    #     dif3y = self.M[2][1]-yt

    #     #range and bearing w/o sensor noise/truth
    #     z_r1_tru = np.sqrt(dif1x**2+dif1y**2)
    #     z_r2_tru = np.sqrt(dif2x**2+dif2y**2)
    #     z_r3_tru = np.sqrt(dif3x**2+dif3y**2)
    #     z_b1_tru = np.arctan2(dif1y,dif1x)
    #     z_b2_tru = np.arctan2(dif2y,dif2x)
    #     z_b3_tru = np.arctan2(dif3y,dif3x)

    #     #add in sensor noise if noise is not specified as 0
    #     z_r1 = z_r1_tru + noise*np.random.normal(0, self.sig_r)
    #     z_r2 = z_r2_tru + noise*np.random.normal(0, self.sig_r)
    #     z_r3 = z_r3_tru + noise*np.random.normal(0, self.sig_r)
    #     z_b1 = wrap(z_b1_tru - tht + noise*np.random.normal(0, self.sig_phi))
    #     z_b2 = wrap(z_b2_tru - tht + noise*np.random.normal(0, self.sig_phi))
    #     z_b3 = wrap(z_b3_tru - tht + noise*np.random.normal(0, self.sig_phi))

    #     z = np.squeeze(np.array([[z_r1, z_r2, z_r3],[z_b1, z_b2, z_b3]]))
    #     return(z)
    # #
    def generate_command(self, t):
        vc_new = 1+0.5*np.cos(2*np.pi*0.2*t)
        wc_new = -0.2+2*np.cos(2*np.pi*0.6*t)

        return vc_new, wc_new
    #