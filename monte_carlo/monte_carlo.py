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
        Xk_bar = Xkt = phi #phi is an empty vector
        for m in range(M):
            xm = rob.vel_motion_model(ut,xm_prev) #be sure noise is included
            wm = self.measurement_model(zt,xm,m) #weight, this actually is a probability calculation using meas model
            Xk_bar = Xk_bar+inner_product(xm,wm) #adds particles to their weights

        #prediction: draw from the proposal
        #correction: weighting by the ratio of target and proposal

        #resampling
        #look at low variance sampler algorithm
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

    def low_var_sampler(self, Xkt, Wt):

        return(Xk_bar)

        # def wrap(self, phi):
        #     phi_new = (phi+np.pi)%(2*np.pi)-np.pi
        #     # phi_new = phi
        #     return phi_new
