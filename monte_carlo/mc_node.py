#Monte Carlo Simulation

from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np

import rob_2wh
import animator
import monte_carlo
import plotter
reload(rob_2wh)
reload(animator)
reload(monte_carlo)
reload(plotter)
from rob_2wh import Rob2Wh
from animator import Animator
from monte_carlo import Monte_Carlo
from plotter import Plotter

def main():

    #user inputs
    markers = 3
    given = 0 #set to one if data is given

    #instatiate objects
    rob = Rob2Wh()
    animate = Animator()
    mc = Monte_Carlo()
    plot = Plotter()

    #create lists
    t = []
    vc = []
    wc = []
    xt = []
    yt = []
    tht = []
    zt = []
    mu = []
    Sig = []
    xe = []
    ye = []
    the = []
    Xk = []

    #collect initial parameters
    states_new = np.array([[rob.x0], [rob.y0], [rob.th0]])
    elements = int(rob.tf/rob.dt)
    M = rob.particles

    #collect given info
    if given != 0:
        t_given = rob.ttr
        x_given = rob.xtr
        y_given = rob.ytr
        th_given = rob.thtr
        v_given = rob.vtr
        w_given = rob.wtr
        z_given = np.squeeze(np.array([[rob.z_rtr],[rob.z_btr]]))

    #initialize particles
    Xkt = mc.uniform_point_cloud(rob.xgrid, rob.ygrid, M)
    # Xkt = np.array([[rob.x0]*M, [rob.y0]*M, [rob.th0]*M])
    ##loop through each time step
    for i in range(0,elements+1):

        ##extract truth for time step
        if given == 0: #generate truth
            t.append(i*rob.dt)
            vc_new, wc_new = rob.generate_command(t[i])
            ut = np.array([[vc_new],[wc_new]])

            #propogate truth
            # if i != 0:
            states_new = rob.vel_motion_model(ut, states_new)
            z_new = rob.simulate_sensor(states_new)

        else: #get truth from given data
            print('truth given')
            t.append(t_given[0][i])
            vc_new, wc_new = rob.generate_command(t[i])
            ut = np.array([[vc_new],[wc_new]])
            states_new = np.array([x_given[0][i], y_given[0][i], th_given[0][i]])
            if markers == 1:
                z_new = np.array([[z_given[0,i], 0, 0, z_given[1,i], 0, 0]]).T
            else:
                z_new = rob.simulate_sensor(states_new) #may need to change depending on the data given

        Xkt = mc.monte_carlo(Xkt, ut, z_new, M, rob)
        x_new = np.mean(Xkt[0,:])
        y_new = np.mean(Xkt[1,:])
        th_new = np.mean(Xkt[2,:])
        mu_new = np.array([x_new, y_new, th_new])
        Sig_new = np.cov(Xkt)

        #append values to lists
        mu.append(mu_new)
        Sig.append(Sig_new)
        xt.append(states_new[0])
        yt.append(states_new[1])
        tht.append(states_new[2])
        xe.append(mu_new[0]-xt[i])
        ye.append(mu_new[1]-yt[i])
        the.append(mu_new[2]-tht[i])
        Xk.append(Xkt)

    #prep varialbes for plotting and animation
    size = len(mu)
    x_hat = []
    y_hat =[]
    th_hat = []
    sig_x = []
    sig_y = []
    sig_th = []
    for i in range(size):
        x_hat.append(mu[i][0]) #hats are the estimates
        y_hat.append(mu[i][1])
        th_hat.append(mu[i][2])
        sig_x.append(Sig[i][0][0])
        sig_y.append(Sig[i][1][1])
        sig_th.append(Sig[i][2][2])

    animate.animator(xt, yt, tht, x_hat,y_hat,th_hat, elements, rob, Xk)

    plot.plotting(x_hat, xt, y_hat, yt, th_hat, tht,\
        t, xe, ye, the, sig_x, sig_y, sig_th)

    return(xt, yt, tht, zt, mu, Sig)
#

if __name__ == '__main__':
	 [x, y, th, z, mu, Sig] = main()
#
