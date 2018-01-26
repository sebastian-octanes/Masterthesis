import nlopt
import numpy as np
import math



def cost_dist(X):
	p1 = np.array([3,0])
	p2 = np.array([3,20])
	N = X.size/6
	cost = 0.0
	for i in range(0,N,1):
		p3 = np.array([X[i*6], X[i*6 +1]])		
		cost = cost + math.fabs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))
	return cost

def const_eq_base(X):
	#fix first 4 values so they wont be optimized
	X0 = np.zeros(4)	
	X0[0] = 2.0
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
	dt = 0.1
	x,y,v,orient,acc,steer = X
	Xnext = np.zeros(4)
	beta = np.arctan((lr/(lf +lr)) * math.tan(steer))
        Xnext[0] = x + v * dt * math.cos(orient + beta)
        Xnext[1] = y + v * dt * math.sin(orient + beta)
        Xnext[2] = v + acc * dt
        Xnext[3] = orient + (v*dt/lr) * math.sin(beta)
	return Xnext

        
dt = 0.1 #s
Tsim = 1 # for now
N = 1	 # for now



umin = [-9, -0.6] #-1g break performance and -0.6 rad 
umax = [9, 0.6] #1g acceleration and 0.6 rad 
vmax = 30 # m/s
vmin = 0  # no driving backwards
psimin = -(30.0/180)*math.pi #in rad
psimax = (30.0/180)*math.pi #in rad


#constraints and bounds
#define bounds fitting to N and Statevector
lb = np.array([- np.inf, - np.inf, vmin, psimin, -9, -0.6])
lb = np.tile(lb, N)
print lb
#constraints
# {'type': 'eq', 'fun': const_eq}
cons =({'type': 'eq', 'fun': const_eq_base}, {'type': 'eq', 'fun': const_eq})

#init state vector
X0 = np.zeros(6*(N+1))
X0[0] = 2.0
X0[1] = 0.0
X0[2] = 0.0
X0[3] = math.pi/2
X0[4] = 0.5
X0[5] = 0.0

opt = nlopt.opt('NLOPT_GN_CRS2_LM', N*6)
opt.set_min_objective(f)
opt.set

for k in np.arange(0, Tsim, dt):
	
	res = minimize(cost_dist, X0, method ='SLSQP', bounds = bnds, constraints=cons)
	print res
	X0[0:4] = model(X0[0:6])	






