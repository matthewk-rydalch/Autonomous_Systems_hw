import numpy as np
# import random
import math
from IPython.core.debugger import set_trace
from numpy.linalg import inv
import utils

class Slam:
    def __init__(self, vel_motion_model, model_sensor, sig_r, sig_phi, M, alpha, dt):
        
        self.g = vel_motion_model
        self.h = model_sensor
        self.sig_r = sig_r
        self.sig_phi = sig_phi
        self.M = M
        self.alpha = alpha
        self.dt = dt
    #

    def ekf(self, Mup, Sig_p, Ut, Zt):

        ##propagation step
        Gt, Vt, Mt = self.prop_jacobians(Mup, Sig_p, Ut)
        Mu_bar = self.g(Ut, Mup, noise=0)
        Sig_bar = Gt@Sig_p@Gt.T+Vt@Mt@Vt.T

        ##correction step
        Qt = [[self.sig_r**2, 0],\
             [0, self.sig_phi**2]]
        for i in range(len(self.M)):
            Ht = self.correction_jacobians(self.M[i,:], Mu_bar)
            Kt = Sig_bar@Ht.T@inv(Ht@Sig_bar@Ht.T+Qt)
            Mu_bar = Mu_bar + np.array([Kt@(Zt[:,i]-self.h(Mu_bar, noise=0)[:,i])]).T
            Mu_bar[2] = utils.wrap(Mu_bar[2])
            Sig_bar = (np.eye(3)-Kt@Ht)@Sig_bar
        
        Mu = Mu_bar
        Sig = Sig_bar

        return(Mu, Sig, Kt)
    #
    def prop_jacobians(self, Mup, Sig_p, Ut):

        xp = Mup[0]
        yp = Mup[1]
        thp = Mup[2]
        vt = Ut[0]
        wt = Ut[1]
        dt = self.dt
        G = np.array([[1, 0, -vt/wt*math.cos(thp)+vt/wt*math.cos(utils.wrap(thp+wt*dt))],\
            [0, 1, -vt/wt*math.sin(thp)+vt/wt*math.sin(utils.wrap(thp+wt*dt))],\
            [0, 0, 1]])

        V = np.array([[1/wt*(-math.sin(thp)+math.sin(utils.wrap(thp+wt*dt))), vt/(wt**2)*(math.sin(thp)-math.sin(utils.wrap(thp+wt*dt)))+vt/wt*(math.cos(utils.wrap(thp+wt*dt))*dt)],\
            [1/wt*(math.cos(thp)-math.cos(utils.wrap(thp+wt*dt))), -vt/(wt**2)*(math.cos(thp)-math.cos(utils.wrap(thp+wt*dt)))+vt/wt*(math.sin(utils.wrap(thp+wt*dt))*dt)],\
            [0, dt]])

        M = np.array([[self.alpha[0]*vt**2+self.alpha[1]*wt**2, 0],\
            [0.0, self.alpha[2]*vt**2+self.alpha[3]*wt**2]])

        return(G, V, M)
    
    def correction_jacobians(self, m, Mu_bar):

        xt = Mu_bar[0]
        yt = Mu_bar[1]
        tht = Mu_bar[2]
        q = (m[0]-xt)**2+(m[1]-yt)**2
        H = np.squeeze(np.array([[-(m[0]-xt)/np.sqrt(q), -(m[1]-yt)/np.sqrt(q), np.array([0.0])],\
                                [(m[1]-yt)/q, -(m[0]-xt)/q, np.array([-1.0])]]))

        return H
    #
#initialization
    #robot starts in its own reference fram all landmarks unknown

##compare with ekf
#prediction steps
    #g
        #we have the pose states but we also have the landmark states
        #the landmarks should not be affected in the prediciton steps
        #use an identity matrix of the appropriate size with zeros appended on it to make there be no change to the markers' states.  See Slides for update the state space.
    #covariance:
        #jacobian of the motion (pose jacobian) is the same, but an identity matrix is used for the markers.  See slides update covariance
        #previous sigma is a matrix of covariances of the pose, markers, and their correlations.
    #extra steps
        #Fx needs to be applied as in slide 41.  Rtx is the pose noise.  We don't use the measurment noise like we used before.  They won't be functions of the speed.

#measurment update
    #when you first see a landmark you have to initialize it
    #we use the belief of the robot location to intialize the landmarks
    #use low-dim space to get the jacobian of markers slide 46
    #we do little jacobian calculations and then we map them into the complete jacobian.
    #you don't necissarily need to make these f matrices and multiply them.  You can just place them where you know they should go.
    #Once we have H everything mostly works the same.
