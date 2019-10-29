#use a for loop for each cell on each time step?
from IPython.core.debugger import set_trace
import scipy
import scipy.io as sio
from importlib import reload
import math
import numpy as np
# import ocp_grid_map
import map_params
# reload(ocp_grid_map)
reload(map_params)
# from ocp_grid_map import ocp_grid_map
from map_params import Map_params

def main():
    # mapping = ocp_grid_map()
    map = Map_params()

    #load given data on robot position
    given = sio.loadmat('state_meas_data.mat')
    X = given['X'] #x,y,th of the robot at each time step
    z = given['z'] #range and bearing at each time step
    thk = given['thk'] #11 angles for the laser range finders

    p0 = map.map
    #lt_i is the log odds ratio at time t
    #l = log(p/(1-p))
    l0 = math.log(p0/(1-p0))
    set_trace()
    #l0 is the log odds ratio of the belief at the beginning
    lprev_i = l0
    steps = len(X[1,:])
    for i in range(steps):
        xt = X[:,i]
        zt = z[:,:,i]

        lt_i = mapping.ocp_grid_map(lprev_i, xt, zt)

        lprev_i = lt_i



if __name__ == '__main__':
	 main()
#
