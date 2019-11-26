import numpy as np
import copy
from IPython.core.debugger import set_trace

class MDP:
    def __init__(self, rx, pc, p90, gamma, plotter, map):
        self.r_min = rx
        self.rx = rx
        self.pc = pc
        self.p90 = p90
        self.gamma = gamma
        self.plotter = plotter
        self.map = map

    def value_iteration(self, vhat):
        #initialize value function
        for i in range(vhat.shape[0]):
            for j in range(vhat.shape[1]):
                if vhat[i][j]==0:
                    vhat[i][j]=self.r_min
        v_prev = np.zeros((vhat.shape[0],vhat.shape[1]))

        #iterate until convergence
        i=0
        while sum(sum(abs(vhat-v_prev))) > .0000000000001 and i < 10000:
            v_prev = vhat
            v_hat, policy = self.mdp_summations(vhat)
            vhat = self.gamma*v_hat
            vhat[self.map!=0]=self.map[self.map!=0] #this reinputs the values of the obstacles, walls, and goal in each iteration
            i = i+1
            print(i)

        return vhat, policy

    def mdp_summations(self, vhat):
        #vhat is an nxm matrix
        n = vhat.shape[0]
        m = vhat.shape[1]
        #set one (no uncertainty)
        vn = vhat[2:n-1,1:m-1]
        ve = vhat[1:n-1,2:m-1]
        vs = vhat[1:n-2,1:m-1]
        vw = vhat[1:n-1,1:m-2]
        v_n = copy.copy(vhat) #shallow copy
        v_e = copy.copy(vhat)
        v_s = copy.copy(vhat)
        v_w = copy.copy(vhat)
        v_n[1:n-2,1:m-1] = vn
        v_e[1:n-1,1:m-2] = ve
        v_s[2:n-1,1:m-1] = vs
        v_w[1:n-1,2:m-1] = vw
        #step 2, add uncertainty
        vn_unc = self.pc*v_n+self.p90*(v_e+v_w)
        ve_unc = self.pc*v_e+self.p90*(v_n+v_s)
        vs_unc = self.pc*v_s+self.p90*(v_e+v_w)
        vw_unc = self.pc*v_w+self.p90*(v_s+v_n)
        vhat_array = np.array([vn_unc, ve_unc, vs_unc, vw_unc])
        v_hat = np.max(self.rx+vhat_array, axis=0)
        policy = np.argmax(self.rx+vhat_array, axis=0)

        return v_hat, policy

    def optimal_path(self, vhat, policy, x0, r_goal):
        j = 0
        path = [x0]
        xt = x0
        while vhat[xt[0]][xt[1]]<r_goal-100 and j < 100:
            if policy[xt[0],xt[1]] == 0:
                ut = np.array([0,1])
            elif policy[xt[0],xt[1]] == 1:
                ut = np.array([1,0])
            elif policy[xt[0],xt[1]] == 2:
                ut = np.array([0,-1])
            elif policy[xt[0],xt[1]] == 3:
                ut = np.array([-1,0])
            
            xt = xt+ut
            path.append(xt)
            j = j+1
            print(j)
        return path


