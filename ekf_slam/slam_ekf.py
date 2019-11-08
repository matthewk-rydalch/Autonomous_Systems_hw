import numpy as np
# import random
import math
from IPython.core.debugger import set_trace
from numpy.linalg import inv
import utils

class Slam:
    def __init__(self, vel_motion_model, model_sensor, sig_r, sig_phi, M, alpha, dt, N, Fx):
        
        self.g = vel_motion_model
        self.h = model_sensor
        self.sig_r = sig_r
        self.sig_phi = sig_phi
        self.M = M
        self.alpha = alpha
        self.dt = dt
        self.N = N
        self.Fx = Fx

        #calculate Fxj's since they are constant
        self.Fxj = []
        for j in range(1,N+1):
            mat1 = np.eye(3,3)
            mat2 = np.zeros((2,3))
            mat3 = np.zeros((5,2*j-2))
            mat4 = np.zeros((3,2))
            mat5 = np.eye(2,2)
            mat6 = np.zeros((5,2*N-2*j))
            mat7 = np.concatenate((mat1,mat2),axis=0)
            mat8 = np.concatenate((mat4,mat5),axis=0)
            self.Fxj.append(np.concatenate((mat7.T,mat3.T,mat8.T,mat6.T),axis=0).T)
    #

    def ekf(self, Mup, Sig_p, Ut, Zt, ct):

        ##propagation step
        Fx = self.Fx
        Gt, Vt, Mt = self.prop_jacobians(Mup, Sig_p, Ut)
        Gt = np.eye(3+2*self.N,3+2*self.N)+Fx.T@Gt@Fx
        Mub = self.g(Ut, Mup, Fx, noise=0)
        Sig_bar = Gt@Sig_p@Gt.T+Fx.T@Vt@Mt@Vt.T@Fx

        ##correction step
        Qt = [[self.sig_r**2, 0],\
             [0, self.sig_phi**2]]
        for j in range(ct):

            #initialize marker if it has not been seen already
            if self.M[0,0] == 0.0 and self.M[0,1] == 0.0: 
                Mub[3+2*j], Mub[4+2*j] = self.initialize_marker(Mub, Zt, j)

            #see corection_jacobians for lines 12-16 of the algorithm in the book                  
            Ht, zhat = self.correction_jacobians(self.M[j,:], Mub, j)
            Kt = Sig_bar@Ht.T@inv(Ht@Sig_bar@Ht.T+Qt)
            Mub = Mub + np.array([Kt@(Zt[:,j]-np.squeeze(zhat))]).T
            Mub[2] = utils.wrapf(Mub[2])
            Sig_bar = (np.eye(len(Kt))-Kt@Ht)@Sig_bar

        Mu = Mub
        Sig = Sig_bar

        return(Mu, Sig)
    #
    def prop_jacobians(self, Mup, Sig_p, Ut):

        xp = Mup[0]
        yp = Mup[1]
        thp = Mup[2]
        vt = Ut[0]
        wt = Ut[1]
        dt = self.dt

        G = np.array([[0, 0, -vt/wt*math.cos(thp)+vt/wt*math.cos(utils.wrap(thp+wt*dt))],\
            [0, 0, -vt/wt*math.sin(thp)+vt/wt*math.sin(utils.wrap(thp+wt*dt))],\
            [0, 0, 0]])

        V = np.array([[1/wt*(-math.sin(thp)+math.sin(utils.wrap(thp+wt*dt))), vt/(wt**2)*(math.sin(thp)-math.sin(utils.wrap(thp+wt*dt)))+vt/wt*(math.cos(utils.wrap(thp+wt*dt))*dt)],\
            [1/wt*(math.cos(thp)-math.cos(utils.wrap(thp+wt*dt))), -vt/(wt**2)*(math.cos(thp)-math.cos(utils.wrap(thp+wt*dt)))+vt/wt*(math.sin(utils.wrap(thp+wt*dt))*dt)],\
            [0, dt]])

        M = np.array([[self.alpha[0]*vt**2+self.alpha[1]*wt**2, 0],\
            [0.0, self.alpha[2]*vt**2+self.alpha[3]*wt**2]])

        return(G, V, M)
    
    def correction_jacobians(self, m, Mub, j):

        #compute expected observation
        delta = np.array([Mub[3+2*j]-Mub[0],\
                            Mub[4+2*j]-Mub[1]])
        q = delta.T@delta
        zhat = np.array([np.sqrt(q[0]), np.arctan2(delta[1],delta[0])-Mub[2]])

        H_lo = np.squeeze(np.array([[np.sqrt(q[0])*delta[0], -np.sqrt(q[0])*delta[1], np.array([0.0]), np.sqrt(q[0])*delta[0], np.sqrt(q[0])*delta[1]],\
                                    [delta[1], -delta[0], -q[0], -delta[1], delta[0]]]))

        H = 1/q[0]*H_lo@self.Fxj[j]
        
        return H, zhat
    #
    def initialize_marker(self, Mub, Zt, j):
        mubx = Mub[0]
        muby = Mub[1]
        mubth = Mub[2]
        rt = Zt[0][j]
        phi_t = Zt[1][j]

        mujx = mubx+rt*np.cos(utils.wrap(phi_t+mubth))
        mujy = muby+rt*np.sin(utils.wrap(phi_t+mubth))

        return mujx, mujy
#initialization
    #robot starts in its own reference frame all landmarks unknown

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
