# -*- coding: utf-8 -*-
"""
Created on Tue Mar  6 14:28:28 2018

@author: weller
"""
import math
from casadi import *
lf = 0.9 
lr = 0.640  
lb = 1.440    
dt = 0.05
max_lat_acc = 20
vmax = 30 # m/s
vmin = 0  # no driving backwards
psimin = -(30.0/180)*math.pi #in rad
psimax = (30.0/180)*math.pi #in rad


N = 35

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
x_tmp2 = x[2] + u[0] * dt 
xdot = vertcat(xdot, x_tmp2)
x_tmp3 = x[3] + (x[2]*dt/lr) * sin(beta)
xdot = vertcat(xdot, x_tmp3)

for i in range(N-1):
    x0 = xdot[i*4] 
    x1 = xdot[i*4 + 1] 
    x2 = xdot[i*4 + 2] 
    x3 = xdot[i*4 + 3] 
    
    beta = arctan((lr/(lf +lr)) * tan(u[i * 2 + 3]))
    beta_max = arctan((lf + lr) * max_lat_acc *0.25 / x2**2)
    
    ineq1 = beta + beta_max
    ineq2 = -beta + beta_max
    ineq = vertcat(ineq, ineq1, ineq2)
    x_tmp = x0 + x2 * dt * cos(x3 + beta)
    xdot = vertcat(xdot, x_tmp)
    x_tmp = x1 + x2 * dt * sin(x3 + beta)
    xdot = vertcat(xdot, x_tmp)
    x_tmp = x2 + u[i * 2 + 2] * dt 
    xdot = vertcat(xdot, x_tmp)
    x_tmp = x3 + (x2*dt/lr) * sin(beta)     
    xdot = vertcat(xdot, x_tmp)    
    
 
f = Function('f', [x,u],[xdot, ineq])



U = MX.sym("U", (N)*2)
X0 = MX([0,0,0.1,0])
#X0 = MX.sym("h", 4)

X, INEQ = f(X0, U)
X = vertcat(X, INEQ)

J =  1/X[2]
for i in range(1,N,1):
    J =  J + 1/X[i*4 + 2]

parameter = [0,1,3,4]

p1 = parameter[0:2]
p2 = parameter[2:4]
ineq1 = (X0[0]-p1[0]) * (p2[1] - p1[1]) - (X0[1] - p1[1]) * (p2[0]- p1[0])            

X = vertcat(X, ineq1)


G = X[0:N*6 +1]


nlp = {'x':U, 'f':J, 'g': G}

# Allocate an NLP solver
opts = {"ipopt.tol":1e-10, "ipopt.print_level": 1}
solver = nlpsol("solver", "ipopt", nlp, opts)


# Bounds on u and initial condition
lbg = np.array([])
ubg = np.array([])
lbx = np.array([])
ubx = np.array([])

for i in range(N):
    lbg = np.append(lbg, np.array([-inf, -inf, vmin, -inf]))
    ubg = np.append(ubg, np.array([inf, inf, vmax, inf]))
    lbx = np.append(lbx, np.array([-9, psimin]))
    ubx = np.append(ubx, np.array([ 9, psimax]))

for i in range(N):
    lbg = np.append(lbg, np.array([0,0]))
    ubg = np.append(ubg, np.array([inf,inf]))

lbg = np.append(lbg, np.array([0]))
ubg = np.append(ubg, np.array([inf]))
       

arg = {}
arg["lbx"] = lbx
arg["ubx"] = ubx

# Bounds on g
arg["lbg"] = lbg
arg["ubg"] = ubg


# Solve the problem
res = solver(**arg)


print(res["x"])

