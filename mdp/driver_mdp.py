import numpy as np
import scipy
from IPython.core.debugger import set_trace
import scipy.io as sio
import copy
from importlib import reload, import_module
visuals = reload(import_module("visuals"))
from visuals import Viz
mdp = reload(import_module("mdp"))
from mdp import MDP

#####parameters#####
gamma = 1
rx = -2 #initial value for each cell, and the cost of moving a space
r_goal = 100000
r_obs = -5000
r_wall = -100
pc = .8 #probability control is correct
p90 = .1 #probability control is 90 degrees off
x0 = np.array([28, 20])
##########

#load and initialize map walls and obstacles
map_mat = sio.loadmat('map.mat')
goal = map_mat['goal']*r_goal
obs1 = map_mat['obs1']*r_obs
obs2 = map_mat['obs2']*r_obs
obs3 = map_mat['obs3']*r_obs
walls = map_mat['walls']*r_wall
map = (goal+obs1+obs2+obs3+walls).T
xm = map_mat['xm']
ym = map_mat['ym']
map_xy = np.array([np.squeeze(xm),np.squeeze(ym)])
R90 = np.array([[np.sin(np.pi/2.0), np.cos(np.pi/2.0)],[np.cos(np.pi/2.0), -np.sin(np.pi/2.0)]])
map_xy = R90@map_xy #map needs to be rotated 90 degrees counterclockwise

#dependent paramters
N = map.shape[0]*map.shape[1]

#instatiate classes
viz = Viz()
mdp = MDP(rx, pc, p90, gamma, viz.plotter, map)

#driver

# Vhat = map.reshape(N, 1)
Vhat = copy.copy(map)
Vhat, policy = mdp.value_iteration(Vhat)
path = mdp.optimal_path(Vhat, policy, x0, r_goal)
viz.plotter(Vhat, policy, path)
#
# for i in range(map.shape[0]):
#     for j in range(map.shape[1]):
#         map[i][j] = Vhat[i*map.shape[1]+j]
