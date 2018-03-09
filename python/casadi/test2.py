# -*- coding: utf-8 -*-
"""
Created on Wed Mar  7 15:22:00 2018

@author: weller
"""

from casadi import *
from numpy import *

dt = 0.01
N = 40
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
    #max_beta =  arctan(1.0/2 * (lf + lr) * max_lat_acc / x[(i*6) +2]**2)    
    #ineq1 = -beta + max_beta
    #ineq2 =  beta + max_beta
    G = vertcat(G, x[(i+1)*6   ] - (x[i*6   ] + x[(i)*6 +2] * dt * cos(x[i*6 +3] + beta)))
    G = vertcat(G, x[(i+1)*6 +1] - (x[i*6 +1] + x[(i)*6 +2] * dt * sin(x[i*6 +3] + beta)))
    G = vertcat(G, x[(i+1)*6 +2] - (x[i*6 +2] + x[(i)*6 +4] * dt))
    G = vertcat(G, x[(i+1)*6 +3] - (x[i*6 +3] + x[(i)*6 +2] * dt /lr * sin(beta)))
    #G = vertcat(G, ineq1, ineq2)
 

J = 1/x[2]
for i in range(N):
    J = J + 1/x[(i+1)* 6 + 2]


nlp = {'x':x, 'f':J, 'g': G}

# Allocate an NLP solver
opts = {"ipopt.tol":1e-10, "ipopt.print_level": 2}
solver = nlpsol("solver", "ipopt", nlp, opts)


# Bounds on u and initial condition
lbg = np.array([0,0,0.01,0])
ubg = np.array([0,0,0.01,0])

lbx = np.array([-inf, -inf, vmin, -inf, -9, -psimin])
ubx = np.array([ inf,  inf, vmax,  inf,  9,  psimax])

x0 =  np.array([0.01,0.01,0.01,0.01, 0.01 ,0.01])
for i in range(N):
    x0  = np.append(x0,  np.array([0.01, 0.01, 0.01, 0.01,   0.01,   0.01]))
    lbg = np.append(lbg, np.array([0, 0, 0, 0]))#,   0,   0]))
    ubg = np.append(ubg, np.array([0, 0, 0, 0]))#, inf, inf]))
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
#print res["x"]


for i in range(res["x"].size()[0] - 6):
    x0[i] = res["x"][i+6] 

for i in range(6):
    x0[-i] = res["x"][-i]
    
x = res["x"][-6:-2]
lbg[0] = x[0]
lbg[1] = x[1]
lbg[2] = x[2]
lbg[3] = x[3]
ubg[0] = x[0]
ubg[1] = x[1]
ubg[2] = x[2]
ubg[3] = x[3]


res = solver(**arg)
#print res["x"]

#==============================================================================
# 
# tmp = np.arange(res["x"].size()[0])
# for i in range(res["x"].size()[0]):
#     if(i%6 == 0):
#         print "\n"
#     print res["x"][i]
# 
#==============================================================================
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    