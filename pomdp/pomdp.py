import numpy as np
import itertools as itls

class POM:

    def __init__(self, r, p, pruning):
        self.r_x1_u1 = r[0]
        self.r_x1_u2 = r[1]
        self.r_x2_u1 = r[2]
        self.r_x2_u2 = r[3]
        self.r_x1_u3 = r[4]
        self.r_x2_u3 = r[5]
        self.p_x12_u3 = p[0]  # probability control u3 worked
        self.p_x11_u3 = p[1]
        self.p_x21_u3 = p[2]
        self.p_x22_u3 = p[3]
        self.p_z1_x1 = p[4]
        self.p_z1_x2 = p[5]
        self.p_z2_x1 = p[6]
        self.p_z2_x2 = p[7]
        self.pruning = pruning

    def value_function(self, T):
        seg = np.zeros((3, 2))
        for i in range(T):
            seg = self.sense(seg)
            seg = self.prune(seg)
            seg = self.predict(seg)
            seg = self.prune(seg)

        return seg

    def sense(self, seg):

        #gather probabilties of measurements
        n = len(seg[0])
        p_z1 = np.zeros((n, 2))
        p_z2 = np.zeros((n, 2))
        p_z1[0, 0] = self.p_z1_x1
        p_z1[1, 1] = self.p_z1_x2
        p_z2[1, 1] = self.p_z2_x2
        p_z2[0, 0] = self.p_z2_x1

        #get inidividual measurement components of value function
        v_z1 = seg@p_z1
        v_z2 = seg@p_z2

        #get every combination of V_z1 and V_z2
        ind = range(len(v_z1))
        perms = np.array(list(itls.permutations(ind, 2)))
        reps = np.array([list(ind[:]), list(ind[:])]).T
        perms = np.concatenate([perms, reps])


        seg = np.array(v_z1[perms[:, 0]]+v_z2[perms[:, 1]])


        return seg

    def predict(self, seg):

        P_prop = np.array([[self.p_x11_u3, self.p_x12_u3],
                           [self.p_x21_u3, self.p_x22_u3]])
        payoff_u1 = np.array([[self.r_x1_u1, self.r_x1_u2]])
        payoff_u2 = np.array([[self.r_x2_u1, self.r_x2_u2]])

        size = seg.shape
        seg = seg@P_prop-np.ones(size)
        seg = np.concatenate([payoff_u1, payoff_u2, seg])

        return seg

    def prune(self, seg):

        if self.pruning == 1:
            dx = 0.0001
            P = np.array([np.arange(1.0/dx)*dx, 1.0-np.arange(1.0/dx)*dx])
            ind = np.unique(np.argmax(seg@P, axis=0))
            seg = seg[ind,:]

        return seg
