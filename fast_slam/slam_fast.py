import numpy as np
# import random
import math
from IPython.core.debugger import set_trace
from numpy.linalg import inv
import utils

class Slam:
    def __init__(self, vel_motion_model, model_sensor, sig_r, sig_phi, alpha, dt, N, Fx, Mtr, particles):

        self.g = vel_motion_model
        self.h = model_sensor
        self.sig_r = sig_r
        self.sig_phi = sig_phi
        self.alpha = alpha
        self.dt = dt
        self.N = N
        self.Fx = Fx
        self.j = [0]*N #used to tell if variable has been initialized
        self.Mtr = Mtr
        self.particles = particles
        self.p0 = 1.0/self.particles
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
    
        self.Qt = np.array([[self.sig_r**2, 0],\
             [0, self.sig_phi**2]])
    #

    def fast_slam(self, Yp, Ut, Zt, ct, time_step):
        #initialize variables to hold particles before resampling
        Y_new = []
        w_new = []
        # if len(ct) == 0:
        #     a=1
        if len(ct) == 0:
            j = 1
        else:
            j = ct[time_step%len(ct)]# j is the landmarker being observed this time step

        ###for each particle
        for k in range(self.particles):

            #break out parts of Yp for kth particle
            Xp, Mup, Sig_p = self.retrieve(Yp, k)

            #propogate pose (sample pose line 4 in algorithm)
            Xt = self.g(Ut, Xp, Fx = np.eye(3,3), noise=1)

            ###for each marker
            # set the new mu to the old, and then change those that are seen
            Muk = Mup
            Sig_k = Sig_p
                
            if self.j[j]==0:  #initialize marker if it has not been seen already

                self.j[j]=1 #variable used to tell if already initialized
                Muk[j] = self.initialize_marker(Zt[:,j], Xt) #h**-1
                H = self.meas_jacobian(Xt, Muk[j]) #h prime
                Sig_k[j] = np.array(inv(H)@self.Qt@inv(H).T)
                wj = np.array([[self.p0]]) #default importance weight
            else:
                zhat, c_ignore = self.h(Xt, Mup[j], fov = 360, noise = 0) #for this leave the fov at 360.  Markers are already canceled out by ct
                H = self.meas_jacobian(Xt, Mup[j]) #h prime
                Q = H@Sig_p[j]@H.T+self.Qt
                K = Sig_p[j]@H.T@inv(Q)
                Muk[j] = Mup[j] + K@(Zt[:,j]-np.squeeze(zhat))
                Sig_k[j] = (np.eye(2)-K@H)@Sig_p[j]
                wj = np.linalg.det(2*np.pi*Q)**(-1/2)*np.exp(-1/2*(Zt[:,j]-zhat.T)@inv(Q)@(Zt[:,j]-zhat.T).T)

            Y_new.append([Xt, Muk, Sig_k])
            w_new.append(wj)
        wp_norm = self.normalize(w_new)

        # # make weights vector and normalize
        # w_new = np.array(w_new)
        # weight_sum = sum(np.squeeze(w_new))
        # print('weight sum = ', weight_sum)
        # wp_norm = np.squeeze(w_new) / np.squeeze(weight_sum)
        max_weight_ind = np.argmax(wp_norm)
        Yt = utils.low_var_sampler(Y_new, wp_norm)
        # if Yt[0][0][0] == Yt[1][0][0]:
        #     print('possible particle deprevation issue')
        #     Ya = np.array(Yt).T
        #     print('X = ',Ya[0])
        #     print('Mu = ',Ya[1])
        #     print('Sig = ',Ya[2])
        #     a =1

        return(Yt, max_weight_ind)
    #

    def meas_jacobian(self, Xt, Muj):
        delta = np.array([Muj[0][0]-Xt[0],\
                Muj[0][1]-Xt[1]])
        q = np.squeeze(delta.T@delta)
        H = 1/q*np.array([[np.sqrt(q)*delta[0], np.sqrt(q)*delta[1]],\
                         [-delta[1], delta[0]]])
    
        return np.squeeze(H)
        
    def initialize_marker(self, Zmt, Xt):
        xt = Xt[0]
        yt = Xt[1]
        tht = Xt[2]
        rt = Zmt[0]
        phi_t = Zmt[1]

        mujx = xt+rt*np.cos(phi_t+tht)
        mujy = yt+rt*np.sin(phi_t+tht)

        muj = np.array([mujx, mujy]).T

        return muj

    def uniform_point_cloud(self, xgrid, ygrid, k):

        # #initialize point or particle cloud
        Xk_prev = np.zeros((3,k))

        #for each particle determine a random selection of states from a uniform distribution
        for i in range(k):
            Xk_prev[0][i] = np.random.uniform(low=xgrid[0], high=xgrid[1], size=None)
            Xk_prev[1][i] = np.random.uniform(low=ygrid[0], high=ygrid[1], size=None)
            Xk_prev[2][i] = np.random.uniform(low= -math.pi, high = math.pi, size = None)

        return(Xk_prev)
    
    def retrieve(self, Yp, k):
        Ypk = Yp[k]
        Xp = Ypk[0]
        Mup = Ypk[1]
        Sig_p = Ypk[2]

        return Xp, Mup, Sig_p

    def normalize(self, w):

        weight_sum = sum(w)
        # print('weight sum = ', weight_sum)
        if weight_sum < 0:
            print("weight sum too small")
        w_norm = np.squeeze(w) / np.squeeze(weight_sum)

        return w_norm

