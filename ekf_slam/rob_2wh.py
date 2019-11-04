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
        # self.dt = dt
        # self.alpha = alpha
        # self.M = M
        # self.sig_r = sig_r
        # self.sig_phi = sig_phi
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
        v_hat = vc + noise*np.random.normal(0, np.sqrt(self.a1*vc**2+self.a2*wc**2),size)
        w_hat = wc + noise*np.random.normal(0, np.sqrt(self.a3*vc**2+self.a4*wc**2),size)
        #include a gamma that accounts for imperfect heading
        gama = noise*np.random.normal(0, np.sqrt(self.a5*vc**2+self.a6*wc**2),size)

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
        difx = self.M[0]-xt
        dify = self.M[1]-yt

        #range and bearing w/o sensor noise/truth
        zr_tru = np.sqrt(difx**2+dify**2)
        zb_tru = np.arctan2(dify,difx)

        #add in sensor noise if noise is not specified as 0
        zr = zr_tru + noise*np.random.normal(0, self.sig_r)
        zb = utils.wrap(zb_tru - tht + noise*np.random.normal(0, self.sig_phi))

        z = np.array([zr,zb])

        return(z)

    def generate_command(self, t):
        vc_new = 1+0.5*np.cos(2*np.pi*0.2*t)
        wc_new = -0.2+2*np.cos(2*np.pi*0.6*t)

        return vc_new, wc_new
    #