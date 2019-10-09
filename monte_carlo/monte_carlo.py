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
    def monte_carlo(self,Xk_prev, ut, zt, M): #do I need a little m?
        set_trace()
        rob = Rob2Wh()

        #Xkt are particles
        phi = np.zeros((M,3)) #phi is an empty vector
        Xk_bar = Xkt = phi
        x_prev = Xk_prev
        for m in range(M):
            #xm is state hypothesis
            x[m], vhat, what = rob.vel_motion_model(ut,x_prev[m]) #be sure noise is included
            w[m] = self.measurement_model(zt,xm,m) #weight, this actually is a probability calculation using meas model
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

    def uniform_point_cloud(self, xgrid, ygrid, M):
        # set_trace()
        Xk_prev = np.zeros((M,3))
        for m in range(M):
            Xk_prev[m][0] = np.random.uniform(low=xgrid[0], high=xgrid[1], size=None)
            Xk_prev[m][1] = np.random.uniform(low=ygrid[0], high=ygrid[1], size=None)
            Xk_prev[m][2] = np.random.uniform(low= -math.pi, high = math.pi, size = None)

        return(Xk_prev)

        # def wrap(self, phi):
        #     phi_new = (phi+np.pi)%(2*np.pi)-np.pi
        #     # phi_new = phi
        #     return phi_new
