# Implement algorithm 15.1 with the objective of representing the value function for the illustrative example of section 15.2
# Compute the value function for the example for a time horizon of 2 and show that you obtain the linear value function constraints of equation 15.31 (along with a few others that have not been pruned away).
# Develop a simple pruning strategy to prune away some of the obviously superfluous constraints (replicates of initial payoff constraints, constraints dominated by (lying below) the payoff constraint for u3).
# Modify the state transition probability for u3 as well as the measurement probabilities for states x1 and x2. Compute value functions for different probabilities. Do your results make sense? Change the payoffs associated with the control actions and compute the value function. Do the results make sense?
# Using the probability and payoff parameters of your choice, use the associated value function to choose control actions. Assume that your true initial state is x1 and that your belief is 0.6. What outcomes do you obtain for 10 trials? Do you outcomes align with your expectations? Did your value function produce good results?

import numpy as np
import scipy
from IPython.core.debugger import set_trace
import scipy.io as sio
import copy
from importlib import reload, import_module
visuals = reload(import_module("visuals"))
from visuals import Viz
pomdp = reload(import_module("pomdp"))
from pomdp import POM

###parameters###
T = 2 #planning horizon, 2 comes from problem statement 2
#include probabillities


#instatiate objects
pom = POM()
viz = Viz()

#run algorithm
G = pom.value_function(T) #G is gamma, which is a set of parameter vectors that specify a linear function over the belief space
