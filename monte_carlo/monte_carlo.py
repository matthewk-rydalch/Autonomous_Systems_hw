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
import wrap
reload(wrap)
from wrap import wrap


class Monte_Carlo:
    def monte_carlo(self,Xk_prev, ut, zt, M, rob): #do I need a little m?

        #Xkt are particles
        phi = np.zeros((M,4)) #phi is an empty vector #include weights (M,4)?
        Xk_bar = Xkt = phi
        x_prev = Xk_prev
        xp = np.zeros((M,3))
        wp = np.zeros((M,1))
        weight_sum = 0
        for m in range(M):
            #xm is state
            state_new, vhat, what = rob.vel_motion_model(ut,x_prev[m]) #be sure noise is included
            xp[m] = state_new[0:3,0]
            wp[m] = self.measurement_model(zt,xp[m], rob.simulate_sensor, rob.sig_phi) #weight, this actually is a probability calculation using meas model
            #this is actually just appending the x and weight to Xk_bar.
            Xk_bar[m] = np.concatenate((xp[m],wp[m]), axis=0) #adds particles to their weights
            weight_sum = weight_sum + wp[m]
        Xk_bar[:,3] = Xk_bar[:,3]/weight_sum
        #prediction: draw from the proposal
        #correction: weighting by the ratio of target and proposal

        #resampling
        #look at low variance sampler algorithm
        # do the low variance sampler for each particle
        for m in range(M):
            drawi() #draw i with probabilty alpha*wt
            addxi() # add xi to Xkt

        return(Xkt)
    #

    def sample_motion_model(self, ut, Xk_u, state_prev):
        #not necissary?  I think this is just my vel_motion_model

        return(Xbar_x)
    #

    def measurement_model(self, zt, xm, simulate_sensor, sig_phi):
        noise = 0
        zm = simulate_sensor(xm, noise)

        wm = self.prob(zt[0]-zm[0], sig_phi)*self.prob(wrap(zt[1]-zm[1]), sig_phi)
        #call simulate_sensor?
        #states have no sensor noise here.  They are truth
        #calc range and bearing then add noise
        #calc range and bearing from particle to landmark as well for the comparison

        #probability that the particles state gets the following measurement.  Compare range and bearings

        #wrapping is important
        #calculate Error
        #calculate the probability given a normal dist and sensor noise characteristics
        #there is a probability calculation in the book  table 5.2
        #product of the two is the weight
        #product of the three probabilties for each landmark is the final probability.  This may not be done in this function?


        return wm

    def prob(self, delta, sig_2):
        probabilty = 1/(math.sqrt(2*math.pi*sig_2))*math.exp(-delta**2/(2*sig_2))
        return probabilty

    def low_var_sampler(self, xt, wt):
        M = len(xt)
        phi = np.zeros((M,3))
        xbar = phi
        r = np.random.uniform(0, 1.0/M, size=None)
        c = wt[0]
        i = 0
        for m in range(M):
            U = r+(m-1.0)/M
            while U>c:
                i = i+1
                c = c+wt[i]
            xbar[m] = xt[i]
        return(xbar)

    def uniform_point_cloud(self, xgrid, ygrid, M):
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
