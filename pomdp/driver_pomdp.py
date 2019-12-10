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
trials = reload(import_module("trials"))
from trials import Trial
pomdp = reload(import_module("pomdp"))
from pomdp import POM

### user defined values
T = 2 #planning horizon, 2 comes from problem statement 2
pruning = 1
trials = 10

###given parameters###
r_x1_u1 = -100
r_x1_u2 = 100
r_x2_u1 = 100
r_x2_u2 = -50
r_x1_u3 = -1
r_x2_u3 = -1
r = [r_x1_u1, r_x1_u2, r_x2_u1, r_x2_u2, r_x1_u3, r_x2_u3]
p_x12_u3 = 0.8 #probability control u3 worked
p_x11_u3 = 0.2
p_x21_u3 = 0.8
p_x22_u3 = 0.2
p_z1_x1 = 0.7
p_z1_x2 = 0.3
p_z2_x1 = 0.3
p_z2_x2 = 0.7
p = [p_x12_u3, p_x11_u3, p_x21_u3, p_x22_u3, p_z1_x1, p_z1_x2, p_z2_x1, p_z2_x2]

# ###made up parameters###
# r_x1_u1 = -20
# r_x1_u2 = 300
# r_x2_u1 = 100
# r_x2_u2 = -50
# r_x1_u3 = -12
# r_x2_u3 = r_x1_u3
# r = [r_x1_u1, r_x1_u2, r_x2_u1, r_x2_u2, r_x1_u3, r_x2_u3]
# p_x12_u3 = 0.98 #probability control u3 worked
# p_x11_u3 = 1-p_x12_u3
# p_x21_u3 = p_x12_u3
# p_x22_u3 = 1-p_x21_u3
# p_z1_x1 = 0.61
# p_z1_x2 = 1-p_z1_x1
# p_z2_x1 = p_z1_x2
# p_z2_x2 = 1-p_z2_x1
# p = [p_x12_u3, p_x11_u3, p_x21_u3, p_x22_u3, p_z1_x1, p_z1_x2, p_z2_x1, p_z2_x2]

#instatiate objects
pom = POM(r, p, pruning)
tri = Trial(p)

#run algorithm
seg = pom.value_function(T) #Seg contains the end points of the line segments
print('line endpoints = ', seg)

#execute simulated control
outcome = []
for i in range(trials):
    outcome.append(tri.tri(seg))
    print('i = ', i)

print('outcomes = ', outcome)
