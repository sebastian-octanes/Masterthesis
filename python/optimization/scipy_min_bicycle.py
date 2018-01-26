from scipy.optimize import minimize
import numpy as np
from matplotlib import pyplot as plt
#import Bicycle as bicycle
import math

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
	N = X.size/6
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
	N = X.size/6 -1
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

def callb(X):
	print X
        
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

res = minimize(cost_dist, X0, method ='SLSQP', bounds = bnds, constraints=cons, callback = callb)
for i in range (0, N+1, 1):
	plt.plot(res.x[i*6], res.x[i*6 +1], "o")
plt.show()
	

#for k in np.arange(0, Tsim, dt):
	
#	res = minimize(cost_dist, X0, method ='SLSQP', bounds = bnds, constraints=cons)
#	print res
#	X0[0:4] = model(X0[0:6])	






