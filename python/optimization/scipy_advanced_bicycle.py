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
   

    def callb(self, X):
        print X
    
    def create_init_state_vec(self, N = 10, x= 0, y = 0, v = 0, orient = math.pi/2):
        X0 = np.zeros(6*(N+1))
        X0[0] = y
        X0[1] = x
        X0[2] = v
        X0[3] = orient
        X0[4] = 0.0
        X0[5] = 0.0
        return X0
        
    def optimize(self, N):
        """filler"""

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

        X0 = self.create_init_state_vec(N)
        
        res = minimize(self.costFunction.cost_dist_line, X0, method ='SLSQP', bounds = bnds, constraints=cons, callback = self.callb)
        for i in range (0, N+1, 1):
        	plt.plot(res.x[i*6], res.x[i*6 +1], "o")
        plt.show()
        	

#for k in np.arange(0, Tsim, dt):
	
#	res = minimize(cost_dist, X0, method ='SLSQP', bounds = bnds, constraints=cons)
#	print res
#	X0[0:4] = model(X0[0:6])	






