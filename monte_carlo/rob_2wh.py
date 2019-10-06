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

        given = sio.loadmat('hw3_1_soln_data.mat') #1 marker
        # given = sio.loadmat('hw3_4_soln_data.mat') #3 markers
        self.z_btr = given['bearing'] #only for 1 marker
        self.z_rtr = given['range'] #only for 1 marker
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

    def generate_command(self, t):
        vc_new = 1+0.5*math.cos(2*math.pi*0.2*t)
        wc_new = -0.2+2*math.cos(2*math.pi*0.6*t)

        return vc_new, wc_new

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
