from matplotlib import pyplot as plt
import numpy as np

class Viz:
    def plotter(self, Vhat, policy,path):
        #
        # x = []
        # y = []
        # Dx = []
        # Dy = []
                #
                # x.append(j)
                # y.append(i)
                # Dx.append(dx)
                # Dy.append(dy)

        fig1 = plt.figure()
        #plot value function
        plt.imshow(Vhat)#, origin='lower')
        #plot arrows (policy)
        for i in range(policy.shape[0]):
            for j in range(policy.shape[1]):
                if policy[i][j] == 0:
                    dx = 0.0
                    dy = 0.5
                elif policy[i][j] == 1:
                    dx = 0.5
                    dy = 0.0
                elif policy[i][j] == 2:
                    dx = 0.0
                    dy = -0.5
                elif policy[i][j] == 3:
                    dx = -0.5
                    dy = 0.0
                plt.arrow(j,i,dx,dy, head_width=.5)
        path = np.array(path)
        plt.plot(path[:,0],path[:,1])
        # plt.plot(28,20, 'r*')
        plt.show()