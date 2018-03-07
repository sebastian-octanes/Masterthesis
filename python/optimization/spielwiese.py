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

N = 30

#control
u = MX.sym('u', 2*N)
#States
x = MX.sym("x", 4)


beta = arctan((lr/(lf +lr)) * tan(u[1]))
beta_max = arctan((lf + lr) * max_lat_acc *0.25 / x[2]**2)
ineq1 = beta + beta_max
ineq2 = -beta + beta_max        
ineq = vertcat(ineq1, ineq2)
x_tmp0 = x[0] + x[2] * dt * cos(x[3] + beta) 
x_tmp1 = x[1] + x[2] * dt * sin(x[3] + beta)
xdot = vertcat(x_tmp0, x_tmp1) 
x_tmp2 = x[2] + u[2] * dt 
xdot = vertcat(xdot, x_tmp2)
x_tmp3 = x[3] + (x[2]*dt/lr) * sin(beta)
xdot = vertcat(xdot, x_tmp3)

for i in range(N-1):
    x0 = xdot[i*4] 
    x1 = xdot[i*4 + 1] 
    x2 = xdot[i*4 + 2] 
    x3 = xdot[i*4 + 3] 
    beta = arctan((lr/(lf +lr)) * tan(u[1 * 2 + 1]))
    beta_max = arctan((lf + lr) * max_lat_acc *0.25 / x2**2)
    ineq1 = beta + beta_max
    ineq2 = -beta + beta_max
    ineq = vertcat(ineq, ineq1, ineq2)
    x_tmp = x0 + x2 * dt * cos(x3 + beta)
    xdot = vertcat(xdot, x_tmp)
    x_tmp = x1 + x2 * dt * sin(x3 + beta)
    xdot = vertcat(xdot, x_tmp)
    x_tmp = x2 + u[k * 2] * dt 
    xdot = vertcat(xdot, x_tmp)
    x_tmp = x3 + (x2*dt/lr) * sin(beta)     
    xdot = vertcat(xdot, x_tmp)    
    

#==============================================================================
# for k in range(1):
#     beta = arctan((lr/(lf +lr)) * tan(u[k * 2 + 1]))
#     beta_max = arctan((lf + lr) * max_lat_acc *0.25 / x[k*6 +2]**2)
#     ineq1 = beta + beta_max
#     ineq2 = -beta + beta_max        
#     x0 = x[k*4 + 0] + x[k*4 +2] * dt * cos(x[k*4 + 3] + beta)  
#     x1 = x[k*4 + 1] + x[k*4 +2] * dt * sin(x[k*4 + 3] + beta)
#     x2 = x[k*4 + 2] + u[k*2] * dt 
#     x3 = x[k*4 +3] + (x[k*4 +2]*dt/lr) * sin(beta)
#     
#     ineq = vertcat(ineq, ineq1, ineq2)    
#     beta = arctan((lr/(lf +lr)) * tan(u[k * 2 + 1]))
#     beta_max = arctan((lf + lr) * max_lat_acc *0.25 / x[k*6 +2]**2)
#     ineq1 = beta + beta_max
#     ineq2 = -beta + beta_max 
#     x4 = x0 + x2 * dt * cos(x3 + beta)
#     x5 = x1 + x2 * dt * sin(x3 + beta)
#     x6 = x2 + u[k * 2] * dt 
#     x7 = x3 + (x2*dt/lr) * sin(beta)     
#     
#     
# #xdot = vertcat(x0,x1,x2,x3)
# xdot = vertcat(x0,x1,x2,x3,x4,x5,x6,x7)
#==============================================================================
     
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


U = MX.sym("U", (N)*2)
X0 = MX([0,0,0.1,0,])

X, INEQ = f(X0, U)
X = vertcat(X, INEQ)

J = 0.3 * X[0*4 +2]/34.0 + 0.3 * X[1*4 +2]/34.0
G = X[0:N*6]


nlp = {'x':U, 'f':J, 'g': G}

# Allocate an NLP solver
opts = {"ipopt.tol":1e-10, "ipopt.print_level": 4}
solver = nlpsol("solver", "ipopt", nlp, opts)



# Bounds on u and initial condition
lbg = np.array([])
ubg = np.array([])
lbx = np.array([])
ubx = np.array([])

for i in range(N):
    lbg = np.append(lbg, np.array([-inf, -inf, vmin, -inf, 0, 0]))
    ubg = np.append(ubg, np.array([inf, inf, vmax, inf, inf, inf]))
    lbx = np.append(lbx, np.array([-9, psimin]))
    ubx = np.append(ubx, np.array([ 9, psimax]))
        

arg = {}
arg["lbx"] = lbx
arg["ubx"] = ubx

# Bounds on g
arg["lbg"] = lbg
arg["ubg"] = ubg

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
