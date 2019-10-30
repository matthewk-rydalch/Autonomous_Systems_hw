#use a for loop for each cell on each time step?
from IPython.core.debugger import set_trace
import os
import scipy
import scipy.io as sio
from importlib import reload, import_module
import math
import numpy as np
from matplotlib import pyplot as plt
ocp_grid_map = reload(import_module("ocp_grid_map"))
from ocp_grid_map import OcpGridMap
animator = reload(import_module("animator"))
from animator import Animator
utils = reload(import_module("utils"))

##########################

animate = Animator()
mapping = OcpGridMap()
params = utils.read_param('map_params.yaml')

# fig1, ax = plt.subplots()
#load given data on robot position
given = sio.loadmat('state_meas_data.mat')
X = given['X'] #x,y,th of the robot at each time step
z = np.nan_to_num(given['z'], np.inf) #range and bearing at each time step
thk = given['thk'] #11 angles for the laser range finders
#set up log ratios l = log(p/(1-p))
p0 = params['map0']*np.ones((params['xlim'],params['ylim']))
l0 = np.log(p0/(1-p0))
lprev_i = np.log(p0/(1-p0))

steps = len(X[1,:])
for i in range(steps):
    xt = X[:,i]
    zt = z[:,:,i]
    lt_i = mapping.ocp_grid_map(lprev_i, xt, zt)
    pt_i = 1-1/(1+np.exp(lt_i))
    lprev_i = lt_i
    print('i = ', i)
    animate.animator(pt_i, xt)

#
