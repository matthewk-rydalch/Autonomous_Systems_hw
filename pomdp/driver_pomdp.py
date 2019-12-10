from IPython.core.debugger import set_trace

from importlib import reload, import_module
trials = reload(import_module("trials"))
from trials import Trial
pomdp = reload(import_module("pomdp"))
from pomdp import POM

### user defined values
T = 2 #planning horizon, 2 comes from problem statement 2
pruning = 1
trials = 50
state0 = 1
bel0 = 0.6


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
    outcome.append(tri.tri(seg, state0, bel0))

print('outcomes = ', outcome)
