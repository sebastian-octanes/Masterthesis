# -*- coding: utf-8 -*-
"""
Created on Wed Mar  7 15:22:00 2018

@author: weller
"""

from casadi import *
from numpy import *

dt = 0.1
N = 4
lf = 0.9 
lr = 0.640  
lb = 1.440    
dt = 0.05
max_lat_acc = 20
vmax = 60 # m/s
vmin = 0.001  # no driving backwards
psimin = -(30.0/180)*math.pi #in rad
psimax = (30.0/180)*math.pi #in rad


x = SX.sym("x",(N+1) *6)
G = vertcat(x[0], x[1], x[2], x[3])

for i in range (N):

    beta = arctan((lr/(lf +lr)) * tan(x[i*6 + 5]))
    max_beta =  arctan(1.0/2 * (lf + lr) * max_lat_acc / x[(i*6) +2]**2)    
    ineq1 = -beta + max_beta
    ineq2 =  beta + max_beta
    G = vertcat(G, x[(i+1)*6   ] - x[i*6   ] + x[(i)*6 +2] * dt * cos(x[i*6 +3] + beta))
    G = vertcat(G, x[(i+1)*6 +1] - x[i*6 +1] + x[(i)*6 +2] * dt * sin(x[i*6 +3] + beta))
    G = vertcat(G, x[(i+1)*6 +2] - x[i*6 +2] + x[(i)*6 +4] * dt)
    G = vertcat(G, x[(i+1)*6 +3] - x[i*6 +3] + x[(i)*6 +2] * dt /lr * sin(beta))
    G = vertcat(G, ineq1, ineq2)
 
#==============================================================================
# = x[1] + x[2] * dt * sin(x[3] + beta)
# 
# = x[2] + u[0] * dt 
# 
# = x[3] + (x[2]*dt/lr) * sin(beta)
# 
#==============================================================================


J = 1/x[2]
for i in range(N):
    print i
    J = J +  1/x[(i+1)*6 + 2]


nlp = {'x':x, 'f':J, 'g': G}

# Allocate an NLP solver
opts = {"ipopt.tol":1e-10, "ipopt.print_level": 3}
solver = nlpsol("solver", "ipopt", nlp, opts)


# Bounds on u and initial condition
lbg = np.array([0,0,0.01,0])
ubg = np.array([0,0,0.01,0])

lbx = np.array([-inf, -inf, vmin, -inf, -9, -psimin])
ubx = np.array([ inf,  inf, vmax,  inf,  9,  psimax])

x0 =  np.array([0,0,0.01,0, 0 ,0])
for i in range(N):
    x0  = np.append(x0,  np.array([0, 0, 0.01, 0,   0,   0]))
    lbg = np.append(lbg, np.array([0, 0, 0, 0,   0,   0]))
    ubg = np.append(ubg, np.array([0, 0, 0, 0, inf, inf]))
    lbx = np.append(lbx, np.array([-inf, -inf, vmin, -inf,  -9, psimin]))
    ubx = np.append(ubx, np.array([ inf,  inf, vmax,  inf,   9, psimax]))


arg = {}
arg["lbx"] = lbx
arg["ubx"] = ubx

# Bounds on g
arg["lbg"] = lbg
arg["ubg"] = ubg
arg["x0" ] = x0

res = solver(**arg)
print res["x"]

#==============================================================================
# 
# print res["x"][0]
# 
# tmp = np.arange(res["x"].size()[0])
# for i in range(res["x"].size()[0]):
#     print i
#     #tmp[i] = round(res["x"][i], 6)   
#     #tmp[i] = float("{0:.2f}".format(res["x"][i]))
#     tmp[i] = res["x"][i]
# 
# for i in range(0,res["x"].size()[0] /6, 1):
#     print tmp[i*6 : i*6 + 6]
# 
#==============================================================================
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    