from scipy.optimize import minimize
import numpy as np
from matplotlib import pyplot as plt
#import Bicycle as bicycle
import math
from casadi import *



def const1(x):
	return x[0] - 2 * x[1] + 2
def const2(x):
	return x[0] - 2 * x[1] + 6
def const3(x):
	return x[0] + 2 * x[1] + 2

cons =({'type': 'ineq', 'fun': const1},
       {'type': 'ineq', 'fun': const2},
       {'type': 'ineq', 'fun': const3})

fun = lambda x: (x[0] - 1)**2 + (x[1] - 2.5)**2

bnds = ((0, None),(0, None))

#res = minimize(fun, (2,0), method ='SLSQP', bounds = bnds, constraints=cons)





def cost_dist_(X):
    p1 = np.array([3,0])
    p2 = np.array([3,20])
    N = 10
    cost = 0.0
    for i in range(0,N,1):
        p3 = np.array([X[i*6], X[i*6 +1]])		
        cost = cost + math.fabs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))
    return cost

def cost_dist(X):
	N = X.size/6
	cost = 0.0
	for i in range(0,N,1):
		cost = cost + math.sqrt(X[i*6]**2 + X[i*6 +1]**2)
	return cost

def const_eq_base(X):
	#fix first 4 values so they wont be optimized
	X0 = np.zeros(4)	
	X0[0] = 2.0
	X0[2] = 4.0
	X0[3] = math.pi/2
	ceq = np.zeros(4)
	ceq = X[0:4] - X0
	return ceq

def const_eq(X):
	N = 10
	ceq = np.zeros(N*4)
	for i in range(0,N,1):
		current_State = X[i*6: (i+1)*6]
		ceq[i*4: (i+1)*4] = X[(i+1)*6: (i+1)*6 + 4] - model(current_State)		
	#return an array of 0= Xnext_state - model(Xnow_state)	
	return ceq
	
def model(X):
    lf = 0.9 
    lr = 0.640  
    lb = 1.440    
    dt = 0.05
    x,y,v,orient,acc,steer = X
    Xnext = np.zeros(4)
    beta = np.arctan((lr/(lf +lr)) * math.tan(steer))
    Xnext[0] = x + v * dt * math.cos(orient + beta)
    Xnext[1] = y + v * dt * math.sin(orient + beta)
    Xnext[2] = v + acc * dt
    Xnext[3] = orient + (v*dt/lr) * math.sin(beta)
    return Xnext


    
dt = 0.05 #s
Tsim = 1 # for now
N = 10	 # for now



umin = [-9, -0.6] #-1g break performance and -0.6 rad 
umax = [9, 0.6] #1g acceleration and 0.6 rad 
vmax = 30 # m/s
vmin = 0  # no driving backwards
psimin = -(30.0/180)*math.pi #in rad
psimax = (30.0/180)*math.pi #in rad


#constraints and bounds
#define bounds fitting to N and Statevector
bnds = ((None, None),(None, None),(vmin, vmax),(None, None),(-9, 9),(psimin, psimax))*(N + 1)
#constraints
# {'type': 'eq', 'fun': const_eq}
cons =({'type': 'eq', 'fun': const_eq_base}, {'type': 'eq', 'fun': const_eq})

#init state vector
X0 = np.zeros(6*(N+1))
X0[0] = 2.0
X0[1] = 1.0
X0[2] = 0.0
X0[3] = math.pi/2
X0[4] = 0.5
X0[5] = 0.0


#==============================================================================
# res = minimize(cost_dist_, X0, method ='SLSQP', bounds = bnds, constraints=cons)
# for i in range (0, N+1, 1):
# 	plt.plot(res.x[i*6], res.x[i*6 +1], "o")
# plt.show()
# 	
#==============================================================================

#==============================================================================
# 
# N = 10
# 
# opti = casadi.Opti()
# # --- decision variables -----
# x = MX.sym("x", 4)
# pos_x = x[0]
# pos_y = x[1]
# v     = x[2]
# orient= x[3]
# u = MX.sym('u',2)
# acc   = u[0]
# steer = u[1]
# 
# # ---- vehicle model ----
# lf = 0.9 
# lr = 0.640  
# lb = 1.440    
# dt = 0.05
# 
# beta = np.arctan((lr/(lf +lr)) * tan(u[1]))
# xdot = pos_x + v * dt * cos(orient + beta)
# ydot = pos_y + v * dt * sin(orient + beta)
# vdot = v + u[0] * dt
# odot = orient + (v*dt/lr) * sin(beta)
# vehic_dot = vertcat(xdot, ydot, vdot, odot)
# 
# f = Function('f', [x,u], [vehic_dot])
# 
# 
# X = MX.sym('X', 4 * (N+1))
# 
# U = MX.sym('U', 2 * (N+1))
# 
# J = MX.sym('J', 1)
# 
# for k in range(N):    
#     X[k*4 : k*4 +4] = f(X[k*4:(k*4)+4], U[k*2 : k*2 + 2])
#     J[0] = J[0] + sqrt(X[k*4]**2 + X[k*4 +1]**2)  
#     
# #J = mtimes(U.T,U) # u'*u in Matlab
# G = X[0:2]     # x(1:2) in Matlab
# 
# nlp = {'x': U, 'f': J, 'g': G}
# 
# # Allocate an NLP solver
# opts = {"ipopt.tol":1e-10, "expand":True}
# solver = nlpsol("solver", "ipopt", nlp, opts)
# arg = {}
# 
# 
# # Bounds on u and initial condition
# arg["lbx"] = -0.5
# arg["ubx"] =  0.5
# arg["x0"] =   0.4
# 
# # Bounds on g
# arg["lbg"] = [10,0]
# arg["ubg"] = [10,0]
# 
# 
# res = solver(**arg)
# print res['x']
# 

# 
# 
# 
#==============================================================================

# ---- bounds ----
umin = [-9, -0.6] #-1g break performance and -0.6 rad 
umax = [9, 0.6] #1g acceleration and 0.6 rad 
vmax = 30 # m/s
vmin = 0  # no driving backwards
psimin = -(30.0/180)*math.pi #in rad
psimax = (30.0/180)*math.pi #in rad

# ---- vehicle model ----
lf = 0.9 
lr = 0.640  
lb = 1.440    
dt = 0.05
max_lat_acc = 20

N = 10
opti = Opti()

all_var = opti.variable(6*(N+1) + N)
#x = opti.variable(6 * (N + 1))
x = all_var[0:6*(N+1)]
bet = all_var[6*(N+1):]
#ineq = [N]
for k in range(N):
    beta = arctan((lr/(lf +lr)) * tan(x[k * 6 + 5]))
    bet[k] = beta + arctan((lf + lr) * max_lat_acc *0.25 / x[k*6 +2]**2)        
    x[(k+1)*6 + 0] = x[k*6 + 0] + x[k*6 +2] * dt * cos(x[k*6 + 3] + beta)
    x[(k+1)*6 + 1] = x[k*6 + 1] + x[k*6 +2] * dt * sin(x[k*6 + 3] + beta)
    x[(k+1)*6 + 2] = x[k*6 + 2] + x[k*6 +4] * dt 
    x[(k+1)*6 + 3] = x[k*6 +3] + (x[k*6 +2]*dt/lr) * sin(beta)


#vehicle bounds
#opti.subject_to(bet[0] >= 0.0)
opti.subject_to(opti.bounded(vmin, x[2::6], vmax))
opti.subject_to(opti.bounded(-9, x[4::6], 9))
opti.subject_to(opti.bounded(psimin, x[5::6], psimax))
#initial state 
init_model_state = opti.parameter(4)
init_m = [2.0, 1.0, 0.1, math.pi/2]
opti.set_value(init_model_state, init_m)
opti.subject_to(x[0:4] == init_model_state[0:4])
opti.subject_to(x[4:6] == [0, 0])


#==============================================================================
# #tangential points constraint
# p = opti.parameter(4)
# ar = [0,1,3,1]
# opti.set_value(p, ar)
# p1 = [p[0], p[1]]
# p2 = [p[2], p[3]]
# ineq1 = (x[0]-p1[0]) * (p2[1] - p1[1]) - (x[1] - p1[1]) * (p2[0]- p1[0])            
# 
# opti.subject_to(ineq1 >= 0) 
# 
#==============================================================================
#tangential points constraint

p = opti.parameter(2,4)
ar = [[0,1,3,4], [0,1,3,1]]
opti.set_value(p[0,:], ar[0])
opti.set_value(p[1,:], ar[1])
p1 = p[0,0:2]
p2 = p[0,2:4]
ineq1 = (x[0]-p1[0]) * (p2[1] - p1[1]) - (x[1] - p1[1]) * (p2[0]- p1[0])            

opti.subject_to(ineq1 >= 0) 


def cost_dist_(X):
    cost = 0
    N = 10
    for i in range(N):
        cost = cost = cost + 0.3 - 0.3 * x[i*6 +2]/34.0 
    return cost
 
opti.minimize(cost_dist_(x))
opti.solver("ipopt")
sol = opti.solve()
print sol.value(x)


