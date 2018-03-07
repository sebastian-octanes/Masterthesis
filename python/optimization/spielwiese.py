from race_track import RaceTrack
from cost_function import CostFunction
from scipy_advanced_bicycle import Optimization
from vehicle_model import VehicleModel
import numpy as np
import math
from scipy import interpolate
from matplotlib import pyplot as plt



from casadi import *

# ---- vehicle model ----
lf = 0.9 
lr = 0.640  
lb = 1.440    
dt = 0.05
max_lat_acc = 20
vmax = 30 # m/s
vmin = 0  # no driving backwards
psimin = -(30.0/180)*math.pi #in rad
psimax = (30.0/180)*math.pi #in rad



def cost_dist_(X):
    cost = 0
    N = 1
    for i in range(N):
        cost = cost = cost + 0.3 - 0.3 * x[i*6 +2]/34.0 
    return cost

N = 1

#control
u = MX.sym('u', 2*N)
#States
x = MX.sym("x", 4*N)

for k in range(N):
    beta = arctan((lr/(lf +lr)) * tan(u[k * 2 + 1]))
    beta_max = arctan((lf + lr) * max_lat_acc *0.25 / x[k*6 +2]**2)
    ineq = beta + beta_max        
    x1 = x[k*4 + 0] + x[k*4 +2] * dt * cos(x[k*4 + 3] + beta)
    x2 = x[k*4 + 1] + x[k*4 +2] * dt * sin(x[k*4 + 3] + beta)
    x3 = x[k*4 + 2] + u[k*2] * dt 
    x4 = x[k*4 +3] + (x[k*4 +2]*dt/lr) * sin(beta)
xdot = vertcat(x1,x2,x3,x4)
      
f = Function('f', [x,u],[xdot, ineq])

#==============================================================================
# p = opti.parameter(2,4)
# ar = [[0,1,3,4], [0,1,3,1]]
# opti.set_value(p[0,:], ar[0])
# opti.set_value(p[1,:], ar[1])
# p1 = p[0,0:2]
# p2 = p[0,2:4]
# ineq1 = (x[0]-p1[0]) * (p2[1] - p1[1]) - (x[1] - p1[1]) * (p2[0]- p1[0])            
# 
# 
#==============================================================================


U = MX.sym("U", 2)
X0 = MX([0,0,0.1,0])

X, INEQ = f(X0, U)
X = vertcat(X, INEQ)

J = 0.3 * X[0*4 +2]/34.0
G = X[0:5]


nlp = {'x':U, 'f':J, 'g': G}

# Allocate an NLP solver
#opts = {"ipopt.tol":1e-10, "ipopt.print_level": 3}
opts = {}
solver = nlpsol("solver", "ipopt", nlp, opts)



# Bounds on u and initial condition
lb = np.array([])
ub = np.array([])
for i in range(N):
    lb = np.append(lb, np.array([-inf, -inf, vmin, -inf]))
    ub = np.append(ub, np.array([inf, inf, vmax, inf]))
lb = np.append(lb, 0)
ub = np.append(ub, inf)


arg = {}
arg["lbx"] = [-9, psimin]
arg["ubx"] = [9, psimax]

# Bounds on g
arg["lbg"] = lb
arg["ubg"] = ub

# Solve the problem
res = solver(**arg)

#==============================================================================
# 

# arg["x0"] =   0.4
# 
# # Bounds on g
# arg["lbg"] = [10,0,0]
# arg["ubg"] = [10,0,2]
#==============================================================================
