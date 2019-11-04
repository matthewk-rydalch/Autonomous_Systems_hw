import numpy as np
# import random
import math
from IPython.core.debugger import set_trace
from numpy.linalg import inv

class Slam:
    def __init__(self, vel_motion_model, model_sensor, sig_r, sig_phi, M, dt):
        
        self.g = vel_motion_model
        self.h = model_sensor
        self.sig_r = sig_r
        self.sig_phi = sig_phi
        self.M = M
        self.dt = dt
    #

    def ekf(self, Mup, Sig_p, Ut, Zt):

        xp = Mup[0]
        yp = Mup[1]
        thp = Mup[2]
        vt = Ut[0]
        wt = Ut[1]
        dt = self.dt

        ##propagation step
        Gt, Vt, Mt = self.jacobians(Mup, Sig_p, Ut)
        Mu_bar = g(Ut, Mup, noise=0)
        Sig_bar = Gt@Sig_p@Gt.T+Vt@Mt@Vt.T

        ##correction step
        Qt, Ht = self.correction_jacobians()
        Kt = Sig_bar@Ht.T@inv(Ht@Sig_bar@Ht.T+Qt)
        Mu = Mu_bar + Kt@(Zt-self.h(Mu_bar, noise=0))
        Sig = (np.eye(3)-Kt@Ht)@Sig_bar

        # z_hathist = np.array([])
        # Hhist = np.array([])
        # Shist = np.array([])
        # Khist = []
        # c = [0, 1, 2]
        # for  i in range(3):
        #     j = c[i]

        #     qt = (M[j][0]-Mu_bar[0])**2+(M[j][1]-Mu_bar[1])**2

        #     zhat1 = math.sqrt(q)
        #     zhat2 = self.wrap(np.arctan2((m[j][1]-mu_bar[1]),(m[j][0]-mu_bar[0])))
        #     zhat3 = mu_bar[2]
        #     z_hat = np.array([[float(zhat1)],[float(self.wrap(zhat2-zhat3))]])
        #     # z_hat = np.array([[float(zhat1)],[float(zhat2)]])

        #     # print('zhat = ', z_hat)
        #     # z_hat = np.array([math.sqrt(q)],\
        #     #             [math.atan2((m[j][1]-mu_bar[1]),(m[j][0]-mu_bar[0]))-mu_bar[2]])

        #     z_now = np.array([[float(z[j])], [float(z[j+3])]])

        #     S = np.array(H@Sig_bar@H.T+Q)
        #     K = np.array(Sig_bar@H.T@inv(S))
        #     mu_bar = np.array(mu_bar+K@(self.wrap(z_now-z_hat)))
        #     Sig_bar = np.array((np.eye(3)-K@H)@Sig_bar)

        #     Khist.append(K)

        # mu = mu_bar
        # Sig = Sig_bar

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
            [0, self.alpha[2]*vt**2+self.alpha[3]*wt**2]])

        return(G, V, M)
    
    def correction_jacobians(self, Mu_bar)

        xt = Mu_bar[0]
        yt = Mu_bar[1]
        tht = Mu_bar[2]

        q = (M[0]-xt)**2+(M[1]-yt)**2

        Q = [[self.sig_r**2, 0],\
             [0, self.sig_phi**2]]

        H = np.array([(-M[0]-xt)/np.sqrt(q), (-M[1]-yt)/np.sqrt(q), np.array([0.0])],\
                     [(M[1]-yt)/q, (-M[0]-xt)/q, -1]])

        return Q, H
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
