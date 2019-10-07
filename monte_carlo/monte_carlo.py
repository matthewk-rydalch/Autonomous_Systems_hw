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
    def monte_carlo(self,Xk_prev, ut, zt, m, M):
        
        rob = Rob2Wh()

        #Xkt are particles
        Xk_bar = Xkt = phi
        for m in range(M):
            xm = rob.vel_motion_model(ut,xm_prev)
            wm = measurement_model(zt,xm,m)
            Xk_bar = Xk_bar+inner_product(xm,wm)

        for m in range(M):
            drawi() #draw i with probabilty alpha*wt
            addxi() # add xi to Xkt

        return(Xkt)
    #

    def sample_motion_model(self, ut, Xk_u, state_prev):


        return(Xbar_x)
    #

    def measurement_model(self, Xbar_x, Xk_z, marker):

        return Z_bar

        # def wrap(self, phi):
        #     phi_new = (phi+np.pi)%(2*np.pi)-np.pi
        #     # phi_new = phi
        #     return phi_new
