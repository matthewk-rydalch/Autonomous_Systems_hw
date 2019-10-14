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

        #need to include particle deprevation
        #finds the number of unique particles
        #if unique particles are less than .1/.5? then add a little noise to them
        #noise is added to all of the particles

        ##prediction step
        #propogate the previous particles
        xp = rob.vel_motion_model(ut,x_prev) #xp is the particles at the current time. Noise is included

        ##Correction step
        #get and normalize the weights
        wp = self.measurement_model(zt,xp, rob.simulate_sensor, rob.sig_r, rob.sig_phi) #wp is the weights of each particle at the current time.  Weight is also the probability
        weight_sum = sum(np.squeeze(wp))
        wp_norm = np.squeeze(wp)/weight_sum

        #resampling
        Xkt = low_var_sampler(xp, wp_norm)

        return(Xkt)
    #

    def measurement_model(self, zt, xm, simulate_sensor, sig_r, sig_phi):
        #get range and bearing for each particle without sensor noise
        noise = 0
        zm = simulate_sensor(xm, noise)

        #calculate weights based on probability of each particle's range and bearing given the actual measurement and sensor noise
        dr1 = zt[0]-zm[0]
        dr2 = zt[1]-zm[1]
        dr3 = zt[2]-zm[2]

        db1 = wrap(zt[3]-zm[3])
        db2 = wrap(zt[4]-zm[4])
        db3 = wrap(zt[5]-zm[5])

        # mr = np.mean(dr)
        # mb = np.mean(db)
        wmr1 = self.prob(dr1, sig_r)
        wmr2 = self.prob(dr2, sig_r)
        wmr3 = self.prob(dr3, sig_r)

        wmb1 = self.prob(db1, sig_phi)
        wmb2 = self.prob(db2, sig_phi)
        wmb3 = self.prob(db3, sig_phi)

        wm = wmr1*wmr2*wmr3*wmb1*wmb2*wmb3
        # wm = self.prob(dr, sig_phi)*self.prob(db, sig_phi)
        #product of the three probabilties for each landmark is the final probability.  This may not be done in this function?

        return wm
    #

    def prob(self, delta, sig):

        #probabilty of a delta given the variance sig^2. Table 5.2
        sig_2 = sig**2 #need variance not standard deviation
        probabilty = 1/(np.sqrt(2*math.pi*sig_2))*np.exp(-delta**2/(2*sig_2))

        return probabilty
    #

    def uniform_point_cloud(self, xgrid, ygrid, M):

        # #initialize point or particle cloud
        Xk_prev = np.zeros((3,M))

        #for each particle determine a random selection of states from a uniform distribution
        for m in range(M):
            Xk_prev[0][m] = np.random.uniform(low=xgrid[0], high=xgrid[1], size=None)
            Xk_prev[1][m] = np.random.uniform(low=ygrid[0], high=ygrid[1], size=None)
            Xk_prev[2][m] = np.random.uniform(low= -math.pi, high = math.pi, size = None)
        #
        # np.save(test_file, Xk_prev)

        # #testing with uniform grid
        # for m in range(1:M):
        #     Xk_prev[0][m] = Xk_prev[0][m-1]+dx
        #     Xk_prev[1][m] = Xk_prev[1][m-1]+dy
        #     Xk_prev[2][m] = Xk_prev[2][m-1]+dth
        # Xk_prev[0,:] = np.arange(-9.75,9.75,.5)
        # Xk_prev[1,:] = np.arange(-9.75,9.75,.5)
        # xk_prev[2,:] = np.arange(-)

        # Xk_prev4 = np.load('test_chi.npy')
        # Xk_prev = Xk_prev4[0:3]
        return(Xk_prev)
    #
