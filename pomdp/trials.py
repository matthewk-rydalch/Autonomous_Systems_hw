import numpy as np
import itertools as itls

class Trial:
    def __init__(self, p):
        self.p_x12_u3 = p[0]  # probability control u3 worked
        self.p_x11_u3 = p[1]
        self.p_x21_u3 = p[2]
        self.p_x22_u3 = p[3]
        self.p_z1_x1 = p[4]
        self.p_z1_x2 = p[5]
        self.p_z2_x1 = p[6]
        self.p_z2_x2 = p[7]

    def tri(self, seg, state, bel):

        ut = 3
        while ut == 3:
            bel = self.measure(state, bel)
            ut, state, bel = self.act(state, bel, seg)

        if state == 1 and ut == 1:
            final_state = 'lava!!!!!!'
        elif state == 1 and ut == 2:
            final_state = 'door'
        elif state == 2 and ut == 1:
            final_state = 'door'
        elif state == 2 and ut == 2:
            final_state = 'lava!!!!!!!'
        else:
            final_state = 'something went wrong'

        return final_state

    def measure(self, state, bel):

        # measure
        number = np.random.uniform(0, 1)
        if number < self.p_z1_x1:
            zt = state
        else:
            if state == 1:
                zt = 2
            else:
                zt = 1

        #update the belief
        pz = self.p_z1_x1 * bel + self.p_z1_x2 * (1 - bel)

        if zt == 1:
            bel = self.p_z1_x1 * bel / pz
        elif zt == 2:
            bel = self.p_z1_x2 * (1 - bel) / pz

        return bel

    def act(self, state, bel, seg):

        #determine action
        size = len(seg)
        line_prob = []
        for i in range(size):
            line_prob.append(seg[i][0]*bel+seg[i][1]*(1-bel))
        ui = np.argmax(line_prob)
        ut = ui + 1

        #update state and belief
        if ut >= 3:
            #update state
            ut = 3
            number = np.random.uniform(0, 1)
            if number < self.p_x12_u3:
                if state == 1:
                    state = 2
                elif state == 2:
                    state = 1
            #update belief
            bel = self.p_x11_u3*bel+self.p_x12_u3*(1-bel)

        return ut, state, bel
