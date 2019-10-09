import numpy as np
import scipy
import scipy.signal
import random
import math
import matplotlib.pyplot as plt
from matplotlib import animation
from IPython.core.debugger import set_trace
from importlib import reload
from numpy.linalg import inv
from numpy.linalg import cholesky
import scipy.io as sio
import wrap
reload(wrap)
from wrap import wrap


# import lmd
# import lmd.rad_wrap_pi
# import lmd.rad_wrap_pi as wrap

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
        self.particles = 1000

        self.x0 = -5
        self.y0 = -3
        self.th0 = math.pi/2 #rad

        # given = sio.loadmat('hw3_1_soln_data.mat') #1 marker
        # # given = sio.loadmat('hw3_4_soln_data.mat') #3 markers
        # self.z_btr = given['bearing'] #only for 1 marker
        # self.z_rtr = given['range'] #only for 1 marker
        # self.wtr = given['om']
        # self.ttr = given['t']
        # self.thtr = given['th']
        # self.vtr = given['v']
        # self.xtr = given['x']
        # self.ytr = given['y']



    def vel_motion_model(self, ut, state):

        vc = ut[0]
        wc = ut[1]
        xt = state[0]
        yt = state[1]
        tht = state[2]

        v_hat = vc + np.random.normal(0, np.sqrt(self.a1*vc**2+self.a2*wc**2))
        w_hat = wc + np.random.normal(0, np.sqrt(self.a3*vc**2+self.a4*wc**2))
        #include a gamma that accounts for imperfect heading
        #gama = np.random.normal(0, np.sqrt(self.a5*vc**2+self.a6*wc**2))
        #we may not have a5 and a6.  His are a5 = .01 a6 = .01

        x1 = xt - v_hat/w_hat*math.sin(tht)+v_hat/w_hat*math.sin(wrap(tht+w_hat*self.dt))
        y1 = yt + v_hat/w_hat*math.cos(tht)-v_hat/w_hat*math.cos(wrap(tht+w_hat*self.dt))
        th1 = wrap(tht + w_hat*self.dt) #add in gamma*self.dt

        states_new = np.array([[x1],[y1],[th1]])

        return states_new, v_hat, w_hat

    def simulate_sensor(self, state, noise = 1):

        xt = state[0]
        yt = state[1]
        tht = state[2]

        dif1x = self.landmark1[0]-xt
        dif1y = self.landmark1[1]-yt
        dif2x = self.landmark2[0]-xt
        dif2y = self.landmark2[1]-yt
        dif3x = self.landmark3[0]-xt
        dif3y = self.landmark3[1]-yt

        z_r1_tru = math.sqrt(dif1x**2+dif1y**2)
        z_r2_tru = math.sqrt(dif2x**2+dif2y**2)
        z_r3_tru = math.sqrt(dif3x**2+dif3y**2)
        z_b1_tru = np.arctan2(dif1y,dif1x)
        z_b2_tru = np.arctan2(dif2y,dif2x)
        z_b3_tru = np.arctan2(dif3y,dif3x)

        z_r1 = z_r1_tru + noise*np.random.normal(0, self.sig_r)
        z_r2 = z_r2_tru + noise*np.random.normal(0, self.sig_r)
        z_r3 = z_r3_tru + noise*np.random.normal(0, self.sig_r)
        z_b1 = wrap(z_b1_tru - tht + noise*np.random.normal(0, self.sig_phi))
        z_b2 = wrap(z_b2_tru - tht + noise*np.random.normal(0, self.sig_phi))
        z_b3 = wrap(z_b3_tru - tht + noise*np.random.normal(0, self.sig_phi))

        z = np.array([[z_r1],[z_r2],[z_r3],[float(z_b1)],[float(z_b2)],[float(z_b3)]])

        return(z)

    def generate_command(self, t):
        vc_new = 1+0.5*math.cos(2*math.pi*0.2*t)
        wc_new = -0.2+2*math.cos(2*math.pi*0.6*t)

        return vc_new, wc_new

    # def wrap(self, phi):
    #     phi_new = (phi+np.pi)%(2*np.pi)-np.pi
    #     # phi_new = phi
    #     return phi_new
