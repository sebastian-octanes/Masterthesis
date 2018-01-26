from scipy.optimize import minimize
import numpy as np
from matplotlib import pyplot as plt
#import Bicycle as bicycle
import math
from race_track import RaceTrack
from vehicle_model import VehicleModel
from cost_function import CostFunction
from constraints import Constraints

class Optimization:
    
    def __init__(self, track_ = RaceTrack(), cost_function_ = CostFunction(), vehicle_model_ = VehicleModel()):
            self.track = track_
            self.costFunction = cost_function_
            self.vehicleModel = vehicle_model_
            self.vehicleModel.dt = 0.05
            self.constraints = Constraints(self.vehicleModel)
   
    def cost_dist_(self, X):
        p1 = np.array([3,0])
        p2 = np.array([3,20])
        N = X.size/6
        cost = 0.0
        for i in range(0,N,1):
            p3 = np.array([X[i*6], X[i*6 +1]])		
            cost = cost + math.fabs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))
        return cost

    def cost_dist(self, X):
        N = X.size/6
        cost = 0.0
        for i in range(0,N,1):
            cost = cost + math.sqrt(X[i*6]**2 + X[i*6 +1]**2)
        return cost

#==============================================================================
#     def const_eq_base(self, X):
# 	#fix first 4 values so they wont be optimized
#         X0 = np.zeros(4)	
#         X0[0] = 2.0
#         X0[2] = 4.0
#         X0[3] = math.pi/2
#         ceq = np.zeros(4)
#         ceq = X[0:4] - X0
#         return ceq
# 
#==============================================================================
#==============================================================================
#     def const_eq(self, X):
#         N = X.size/6 -1
#         ceq = np.zeros(N*4)
#         for i in range(0,N,1):
#             current_State = X[i*6: (i+1)*6]
#             ceq[i*4: (i+1)*4] = X[(i+1)*6: (i+1)*6 + 4] - self.vehicleModel.compute_next_state(current_State)		
#         #return an array of 0= Xnext_state - model(Xnow_state)	
#         return ceq
# 	
#==============================================================================

    def callb(self, X):
        print X
    
    
        
    def optimize(self):
        """filler"""
        
        N = 10	 # for now
     
        #constraints and bounds
        #define bounds fitting to N and Statevector
        
        bnds = self.vehicleModel.get_bounds(N)       
        print bnds
        #constraints
        # {'type': 'eq', 'fun': const_eq}
        
        cons =({'type': 'eq', 'fun': self.constraints.const_eq_base}, 
               {'type': 'eq', 'fun': self.constraints.const_eq})
        
        #init state vector
        #[x, y, v, orient, acc, steer_angle]
        X0 = np.zeros(6*(N+1))
        X0[0] = 2.0
        X0[1] = 1.0
        X0[2] = 0.0
        X0[3] = math.pi/2
        X0[4] = 0.5
        X0[5] = 0.0
        
        
        res = minimize(self.costFunction.cost_dist_line, X0, method ='SLSQP', bounds = bnds, constraints=cons, callback = self.callb)
        for i in range (0, N+1, 1):
        	plt.plot(res.x[i*6], res.x[i*6 +1], "o")
        plt.show()
        	

#for k in np.arange(0, Tsim, dt):
	
#	res = minimize(cost_dist, X0, method ='SLSQP', bounds = bnds, constraints=cons)
#	print res
#	X0[0:4] = model(X0[0:6])	






