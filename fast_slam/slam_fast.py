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
        self.j = np.zeros((particles,N)) #used to tell if variable has been initialized
        self.Mtr = Mtr
        self.particles = particles
        self.p0 = 1.0/self.particles
    
        self.Qt = np.array([[self.sig_r**2, 0],\
             [0, self.sig_phi**2]])
    #

    def fast_slam(self, Yp, Ut, Zt, ct, time_step):
        #initialize variables to hold particles before resampling
        Y_new = []
        w_new = []

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
            wj = np.ones(len(Muk))
            wk = 1
            j = ct[time_step%len(ct)]
            # for j in ct:
            if self.j[k][j]==0:  #initialize marker if it has not been seen already

                self.j[k][j]=1 #variable used to tell if already initialized
                Muk[j] = self.initialize_marker(Zt[:,j], Xt) #h**-1
                H = self.meas_jacobian(Xt, Muk[j]) #h prime
                Sig_k[j] = np.array(inv(H)@self.Qt@inv(H).T)
                wj = np.array([[self.p0]]) #default importance weight
            elif self.j[k][j]==1:
                zhat, c_ignore = self.h(Xt, Mup[j], fov = 2*np.pi, noise = 0) #for this leave the fov at 2 pi.  Markers are already canceled out by ct
                H = self.meas_jacobian(Xt, Mup[j]) #h prime
                Q = H@Sig_p[j]@H.T+self.Qt
                K = Sig_p[j]@H.T@inv(Q)
                Muk[j] = Mup[j] + K@(Zt[:,j]-np.squeeze(zhat))
                Muk[j][0][1] = utils.wrap(Muk[j][0][1])
                Sig_k[j] = (np.eye(2)-K@H)@Sig_p[j]
                wj = np.linalg.det(2*np.pi*Q)**(-1/2)*np.exp(-1/2*(Zt[:,j]-zhat.T)@inv(Q)@(Zt[:,j]-zhat.T).T)
                if wj == 0:
                    bdsas = 1
                    # print('wj = ', wj[j])
            Y_new.append([Xt, Muk, Sig_k])
            w_new.append(wj)
        wp_norm, w_sum = self.normalize(w_new)
        max_weight_ind = np.argmax(wp_norm)

        Yt = utils.low_var_sampler(Y_new, wp_norm)
        if Yt[0][0][0] == Yt[99][0][0]:
            print('possible particle deprevation issue')

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
    
    def retrieve(self, Yp, k):
        Ypk = Yp[k]
        Xp = Ypk[0]
        Mup = Ypk[1]
        Sig_p = Ypk[2]

        return Xp, Mup, Sig_p

    def normalize(self, w):

        weight_sum = sum(w)
        print('weight sum = ', weight_sum)

        w_norm = np.squeeze(w) / np.squeeze(weight_sum)

        return w_norm, weight_sum

