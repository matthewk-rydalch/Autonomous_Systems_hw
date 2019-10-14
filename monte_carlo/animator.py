from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

class Animator:
    def animator(self, xt, yt, tht, xhat, yhat, thhat, elements, rob, Xk):

        #hats are predicted states
        #Xk are the particles

        #make states and estimates global
        self.xhat = xhat
        self.yhat = yhat
        self.thhat = thhat

        #gather landmark locations
        mx1, my1, mx2, my2, mx3, my3 = rob.landmark1[0], rob.landmark1[1],\
         rob.landmark2[0], rob.landmark2[1], rob.landmark3[0], rob.landmark3[1]

        #initialize lists
            #tru is a list of the truth
            #data is the curent estimate of the robot
            #hist is history of the estimate list
            #xpoint is the x direction of the estimated heading
        xtru, ytru, xdata, ydata, thdata, xhist, yhist, xpoint, ypoint = [], [], [], [], [], [], [], [], []

        #initialize animations
        fig, ax = plt.subplots()
        plt.axes(xlim=(rob.xgrid), ylim=(rob.ygrid))

        #static plots
        plt.plot(mx1,my1,'g^') #plot markers
        plt.plot(mx2,my2,'g^')
        plt.plot(mx3,my3,'g^')
        plt.plot(xhat,yhat,'y') #plot estimated path
        plt.plot(xt,yt,'b') #plot true path

        #initialize animations elements
        robot, = plt.plot([], [], 'ro', markersize=12, animated=True)
        arrow, = plt.plot([], [], 'k*', markersize=3, animated=True)
        particles, = plt.plot([], [], 'k.', markersize=3, animated=True)
        f = np.linspace(-3, 3, 200)

        def init():

            robot.set_data(xdata,ydata)
            arrow.set_data(xpoint, ypoint)
            particles.set_data(Xk[0][:,0], Xk[0][:,1])

            return robot, arrow, particles

        def update(frame):

            #gather the right elements
            i = int(frame)
            xdata = self.xhat[i]
            ydata = self.yhat[i]
            thdata = self.thhat[i]
            xpoint = xdata + np.cos(thhat[i])
            ypoint = ydata + np.sin(thhat[i])

            #set the animation elements
            robot.set_data(xdata, ydata)
            arrow.set_data(xpoint, ypoint)
            particles.set_data(Xk[i][:,0], Xk[i][:,1])

            return robot, arrow, particles

        #run the animation
        ani = animation.FuncAnimation(fig, update,
                            init_func=init, frames = 201, interval = 20, blit=True)
        plt.show()
