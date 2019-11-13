import numpy as np
# import random
import math
from IPython.core.debugger import set_trace
from numpy.linalg import inv
import utils

class Slam:
    def __init__(self, vel_motion_model, sig_r, sig_phi, alpha, dt, N, Fx, Mtr, particles):

        self.g = vel_motion_model
        self.sig_r = sig_r
        self.sig_phi = sig_phi
        self.alpha = alpha
        self.dt = dt
        self.N = N
        self.Fx = Fx
        self.j = [0]*N #used to tell if variable has been initialized
        self.Mtr = Mtr
        self.particles = particles
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

    def fast_slam(self, Yp, Ut, Zt, ct):

        for k in range(self.particles):
            #break out parts of Yp for kth particle
            Ypk = Yp[k]
            Xp = Ypk[0]
            Mup = Ypk[1]
            Sig_p = Ypk[2]

            #propogate pose (sample pose line 4 in algorithm?)
            Fx = np.eye(3,3)
            Xt = self.g(Xp, Ut, Fx, noise=0)

            for j in ct:
                #initialize marker if it has not been seen already
                if self.j[j]==0:              
                    # Mub[3+2*j], Mub[4+2*j] = self.initialize_marker(Mub, Zt, j)
                    self.j[j]=1 #variable used to tell if already initialized
                    set_trace()
                    h_function = initialize_marker(Zt[j],Xt)
                    muj = inv(h_function)

        # ##propagation step
        # Fx = self.Fx
        # Gt, Vt, Mt = self.prop_jacobians(Mup, Sig_p, Ut)
        # Gt = np.eye(3+2*self.N,3+2*self.N)+Fx.T@Gt@Fx
        # Mub = self.g(Ut, Mup, Fx, noise=0)
        # Sig_bar = Gt@Sig_p@Gt.T+Fx.T@Vt@Mt@Vt.T@Fx

        # ##correction step
        # Qt = np.array([[self.sig_r**2, 0],\
        #      [0, self.sig_phi**2]])

        # for j in ct:

        #     #initialize marker if it has not been seen already
        #     if self.j[j]==0:
        #         # [Mub[3+2*j],Mub[4+2*j]]=self.Mtr[j]               
        #         Mub[3+2*j], Mub[4+2*j] = self.initialize_marker(Mub, Zt, j)
        #         self.j[j]=1 #variable used to tell if already initialized

        #     delta = np.array([Mub[3+2*j]-Mub[0],\
        #             Mub[4+2*j]-Mub[1]])
        #     q = delta.T@delta
        #     zhat = np.array([np.sqrt(q[0]), utils.wrap(np.arctan2(delta[1],delta[0])-Mub[2])])

        #     H_lo = np.squeeze(np.array([[-np.sqrt(q[0])*delta[0], -np.sqrt(q[0])*delta[1], np.array([0.0]), np.sqrt(q[0])*delta[0], np.sqrt(q[0])*delta[1]],\
        #                                 [delta[1], -delta[0], -q[0], -delta[1], delta[0]]]))
        #     Ht = 1/q[0]*H_lo@self.Fxj[j]
        #     Kt = Sig_bar@Ht.T@inv(Ht@Sig_bar@Ht.T+Qt)
        #     difz = Zt[:,j]-np.squeeze(zhat)
        #     difz[1] = utils.wrap(difz[1])
        #     Mub = Mub + np.array([Kt@difz]).T
        #     Mub[2] = utils.wrap(Mub[2])
        #     Sig_bar = (np.eye(len(Kt))-Kt@Ht)@Sig_bar

        # Mu = Mub
        # Sig = Sig_bar

        return(Mu, Sig)
    #
    def prop_jacobians(self, Mup, Sig_p, Ut):

        xp = Mup[0]
        yp = Mup[1]
        thp = Mup[2]
        vt = Ut[0]
        wt = Ut[1]
        dt = self.dt

        G = np.array([[0, 0, -vt/wt*math.cos(thp)+vt/wt*math.cos(thp+wt*dt)],\
            [0, 0, -vt/wt*math.sin(thp)+vt/wt*math.sin(thp+wt*dt)],\
            [0, 0, 0]])

        V = np.array([[1/wt*(-math.sin(thp)+math.sin(thp+wt*dt)), vt/(wt**2)*(math.sin(thp)-math.sin(thp+wt*dt))+vt/wt*(math.cos(thp+wt*dt)*dt)],\
            [1/wt*(math.cos(thp)-math.cos(thp+wt*dt)), -vt/(wt**2)*(math.cos(thp)-math.cos(thp+wt*dt))+vt/wt*(math.sin(thp+wt*dt)*dt)],\
            [0, dt]])

        M = np.array([[self.alpha[0]*vt**2+self.alpha[1]*wt**2, 0],\
            [0.0, self.alpha[2]*vt**2+self.alpha[3]*wt**2]])

        return(G, V, M)
    #
    
    def initialize_marker(self, Mub, Zt, j):
        mubx = Mub[0]
        muby = Mub[1]
        mubth = Mub[2]
        rt = Zt[0][j]
        phi_t = Zt[1][j]

        mujx = mubx+rt*np.cos(phi_t+mubth)
        mujy = muby+rt*np.sin(phi_t+mubth)

        return mujx, mujy

    def uniform_point_cloud(self, xgrid, ygrid, k):

        # #initialize point or particle cloud
        Xk_prev = np.zeros((3,k))

        #for each particle determine a random selection of states from a uniform distribution
        for i in range(k):
            Xk_prev[0][i] = np.random.uniform(low=xgrid[0], high=xgrid[1], size=None)
            Xk_prev[1][i] = np.random.uniform(low=ygrid[0], high=ygrid[1], size=None)
            Xk_prev[2][i] = np.random.uniform(low= -math.pi, high = math.pi, size = None)

        return(Xk_prev)
