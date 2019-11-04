"""
Matplotlib Animation Example

author: Jake Vanderplas
email: vanderplas@astro.washington.edu
website: http://jakevdp.github.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from IPython.core.debugger import set_trace
from rob_2wh import rob_2wh

class visualizer:
    def animator(self, x, y, th, xhat, yhat, thhat, elements):
        self.xtru = x
        self.ytru = y
        self.thtru = th
        self.xhat = xhat
        self.yhat = yhat
        self.thhat = thhat

        rob = rob_2wh()
        mx1, my1, mx2, my2, mx3, my3 = rob.landmark1[0], rob.landmark1[1],\
         rob.landmark2[0], rob.landmark2[1], rob.landmark3[0], rob.landmark3[1]
        # check = plt.plot(x,y,'g')

        fig, ax = plt.subplots()
        xtru, ytru, xdata, ydata, thdata, xhist, yhist, xpoint, ypoint = [], [], [], [], [], [], [], [], []
        plt.axes(xlim=(-10, 10), ylim=(-10, 10))
        plt.plot(mx1,my1,'g^')
        plt.plot(mx2,my2,'g^')
        plt.plot(mx3,my3,'g^')
        robot, = plt.plot([], [], 'ro', markersize=12, animated=True)
        line_hat, = plt.plot([], [], 'y', animated=True)
        line_tru, = plt.plot([], [], 'b', animated=True)
        arrow, = plt.plot([], [], 'k*', markersize=3, animated=True)
        f = np.linspace(-3, 3, 200)

        # initialization function: plot the background of each frame
        def init():
            # ax.set_xlim(-10, 10)
            # ax.set_ylim(-10, 10)
            line_tru.set_data(xtru,ytru)
            line_hat.set_data(xhist,yhist)
            robot.set_data(xdata,ydata)
            arrow.set_data(xpoint, ypoint)
            # marker1.set_data(mx1,my1)
            # marker2.set_data(mx2,my2)
            # marker3.set_data(mx3,my3)
            return robot, line_hat, line_tru, arrow

        def update(frame):
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
            return robot, line_hat, line_tru, arrow

        ani = animation.FuncAnimation(fig, update,
                            init_func=init, frames = 201, interval = 20, blit=True)
        plt.show()
