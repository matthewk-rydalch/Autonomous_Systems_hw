import numpy as np
from importlib import reload
import random
import math
from IPython.core.debugger import set_trace
from numpy.linalg import inv

import utils
reload(utils)
from utils import wrap
from utils import low_var_sampler


class Monte_Carlo:
    def monte_carlo(self,Xk_prev, ut, zt, M, rob):

        #initialize variables
        phi = np.zeros((M,4))
        Xk_bar = Xkt = phi #Xk variables are the particles
        x_prev = Xk_prev #each element of Xk in the algorithm are symbolized by x

        ##prediction step
        #propogate the previous particles
        xp = rob.vel_motion_model(ut,x_prev) #xp is the particles at the current time. Noise is included

        ##Correction step
        #get and normalize the weights
        wp = self.measurement_model(zt,xp, rob.simulate_sensor, rob.sig_phi) #wp is the weights of each particle at the current time.  Weight is also the probability
        weight_sum = sum(np.squeeze(wp))
        wp_norm = np.squeeze(wp/weight_sum)

        #resampling
        Xkt = low_var_sampler(xp, wp_norm)

        return(Xkt)
    #

    def measurement_model(self, zt, xm, simulate_sensor, sig_phi):

        #get range and bearing for each particle without sensor noise
        noise = 0
        zm = simulate_sensor(xm, noise)

        #calculate weights based on probability of each particle's range and bearing given the actual measurement and sensor noise
        set_trace()
        wm = self.prob(zt[0]-zm[0], sig_phi)*self.prob(wrap(zt[3]-zm[3]), sig_phi) #there is something weird going on here.  

        #product of the three probabilties for each landmark is the final probability.  This may not be done in this function?

        return wm
    #

    def prob(self, delta, sig_2):

        #probabilty of a delta given the variance sig^2
        probabilty = 1/(np.sqrt(2*math.pi*sig_2))*np.exp(-delta**2/(2*sig_2))

        return probabilty
    #

    def uniform_point_cloud(self, xgrid, ygrid, M):

        #initialize point or particle cloud
        Xk_prev = np.zeros((3,M))

        #for each particle determine a random selection of states from a uniform distribution
        for m in range(M):
            Xk_prev[0][m] = np.random.uniform(low=xgrid[0], high=xgrid[1], size=None)
            Xk_prev[1][m] = np.random.uniform(low=ygrid[0], high=ygrid[1], size=None)
            Xk_prev[2][m] = np.random.uniform(low= -math.pi, high = math.pi, size = None)

        return(Xk_prev)
    #
