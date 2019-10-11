from IPython.core.debugger import set_trace
from importlib import reload

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

import rob_2wh
reload(rob_2wh)
from rob_2wh import Rob2Wh

class Animator:
    def animator(self, x, y, th, xhat, yhat, thhat, elements, rob, Xk):
        self.xtru = x
        self.ytru = y
        self.thtru = th
        self.xhat = xhat
        self.yhat = yhat
        self.thhat = thhat

        mx1, my1, mx2, my2, mx3, my3 = rob.landmark1[0], rob.landmark1[1],\
         rob.landmark2[0], rob.landmark2[1], rob.landmark3[0], rob.landmark3[1]
        # check = plt.plot(x,y,'g')

        fig, ax = plt.subplots()
        xtru, ytru, xdata, ydata, thdata, xhist, yhist, xpoint, ypoint = [], [], [], [], [], [], [], [], []
        plt.axes(xlim=(rob.xgrid), ylim=(rob.ygrid))
        plt.plot(mx1,my1,'g^')
        plt.plot(mx2,my2,'g^')
        plt.plot(mx3,my3,'g^')
        robot, = plt.plot([], [], 'ro', markersize=12, animated=True)
        line_hat, = plt.plot([], [], 'y', animated=True)
        line_tru, = plt.plot([], [], 'b', animated=True)
        arrow, = plt.plot([], [], 'k*', markersize=3, animated=True)
        particles, = plt.plot([], [], 'k.', markersize=3, animated=True)
        f = np.linspace(-3, 3, 200)

        # initialization function: plot the background of each frame
        def init():
            # set_trace()
            line_tru.set_data(xtru,ytru)
            line_hat.set_data(xhist,yhist)
            robot.set_data(xdata,ydata)
            arrow.set_data(xpoint, ypoint)
            # set_trace()
            particles.set_data(Xk[0][:,0], Xk[0][:,1])

            return robot, line_hat, line_tru, arrow, particles

        def update(frame):
            set_trace()
            i = int(frame)
            xdata = self.xhat[i]
            ydata = self.yhat[i]
            thdata = self.thhat[i]
            r = 1
            xpoint = xdata + r*np.cos(thhat[i])
            ypoint = ydata + r*np.sin(thhat[i])
            xhist.append(self.xhat[i])
            yhist.append(self.yhat[i])
            xtru.append(self.xtru[i])
            ytru.append(self.ytru[i])
            robot.set_data(xdata, ydata)
            line_tru.set_data(xtru, ytru)
            line_hat.set_data(xhat, yhat)
            arrow.set_data(xpoint, ypoint)
            particles.set_data(Xk[i][:,0], Xk[i][:,1])
            return robot, line_hat, line_tru, arrow, particles

        ani = animation.FuncAnimation(fig, update,
                            init_func=init, frames = 201, interval = 20, blit=True)
        plt.show()
