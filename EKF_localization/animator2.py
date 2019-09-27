import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from IPython.core.debugger import set_trace

class animator:
    def animator(self, x, y, th, elements):
        self.x = x
        self.y = y
        self.th = th

        fig, ax = plt.subplots()
        xdata, ydata = [], []
        ln, = plt.plot([], [], 'r', animated=True)
        f = np.linspace(-3, 3, 200)

        # initialization function: plot the background of each frame
        def init():
        	ax.set_xlim(-20, 0)
        	ax.set_ylim(-10, 10)
        	ln.set_data(xdata,ydata)
        	return ln,

        def update(frame):
            i = int(frame)
            xdata.append(self.x[i])
            ydata.append(self.y[i])
            ln.set_data(xdata, ydata)
            return ln,

        ani = animation.FuncAnimation(fig, update, frames=f,
                            init_func=init, blit=True, interval = 2.5,repeat=False)
        plt.show()
