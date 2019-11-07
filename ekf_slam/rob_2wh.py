import numpy as np
import math
import random
from IPython.core.debugger import set_trace
from importlib import reload
import utils


class Rob2Wh:
    def __init__(self, dt, alpha, M, sig_r, sig_phi):
        self.dt = dt
        self.alpha = alpha
        # self.M = np.array(np.split(M,int(len(M)/2))).T #just rearrange shape to match what was coded for ekf
        self.M = M
        self.sig_r = sig_r
        self.sig_phi = sig_phi
    #

    def vel_motion_model(self, Ut, Mup, Fx, noise = 1):

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
        Mu = Mup+Fx.T@np.array([-v_hat/w_hat*np.sin(tht)+v_hat/w_hat*np.sin(utils.wrap(tht+w_hat*self.dt)),\
                                v_hat/w_hat*np.cos(tht)-v_hat/w_hat*np.cos(utils.wrap(tht+w_hat*self.dt)),\
                                utils.wrap(tht + w_hat*self.dt + gama*self.dt)])

        return Mu
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
        zb = zb_tru - tht + noise*np.random.normal(0, self.sig_phi) #for some reason I can't wrap my bearing or there are major errors

        z = np.array([zr,zb])

        return(z)
    #
    def generate_command(self, t):
        vc_new = 1+0.5*np.cos(2*np.pi*0.2*t)
        wc_new = -0.2+2*np.cos(2*np.pi*0.6*t)

        return vc_new, wc_new
    #