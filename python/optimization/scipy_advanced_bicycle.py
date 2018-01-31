from scipy.optimize import minimize
import numpy as np
from matplotlib import pyplot as plt
#import Bicycle as bicycle
import math
from race_track import RaceTrack
from vehicle_model import VehicleModel
from cost_function import CostFunction
from constraints import Constraints
import time

class Optimization:
    
    def __init__(self, track_ = RaceTrack(), cost_function_ = CostFunction(), vehicle_model_ = VehicleModel()):
            self.track = track_
            self.costFunction = cost_function_
            self.vehicleModel = vehicle_model_
            self.dt = 0.1
            self.vehicleModel.dt = self.dt           
            self.process = []

    def callb(self, X):
        print X
    
    def create_init_state_vec(self, N = 10, x= 0, y = 0, v = 0, orient = math.pi/2):
        X0 = np.zeros(6*(N+1))
        X0[0] = x
        X0[1] = y
        X0[2] = v
        X0[3] = orient
        X0[4] = 0.0
        X0[5] = 0.0
        return X0
        
    def log_position(self, pos):
        self.process.append(pos)
        
    def plot_race_process(self):
        tmp = np.array(self.process)
        plt.plot(tmp[:,0], tmp[:,1])
        plt.show()        
            
    def optimize(self, N, loops = 10):
        """filler"""
        #Tsim = 2
        
        #set initial position 
        X0 = self.create_init_state_vec(N, 1, 0, 3.0, math.pi/2)

        #constraints and bounds
        #define bounds fitting to N and Statevector
        
        bnds = self.vehicleModel.get_bounds(N)       
        print bnds
        
        #constraints
        # {'type': 'eq', 'fun': const_eq}
        self.constraints = Constraints(X0[:4], self.vehicleModel)
        cons =({'type': 'eq', 'fun': self.constraints.constraint_fix_init_state}, 
               {'type': 'eq', 'fun': self.constraints.constraint_vehicle_model})
        
        #init state vector
        #[x, y, v, orient, acc, steer_angle]
        
        print "X0"        
        print X0
        #for k in np.arange(0, Tsim, self.dt):
        self.log_position(X0[0:4])
        start = time.time()
        for k in range (0, loops, 1):         
            res = minimize(self.costFunction.cost_dist_track, X0, method ='SLSQP', bounds = bnds, constraints=cons)#, callback = self.callb)        
            x_new = self.vehicleModel.compute_next_state(res.x[0:6])          
            self.log_position(x_new)
            X0 = res.x
            X0[0:4] = x_new
            self.constraints.set_initial_state(X0[0:4])
            print ("schritt " + repr(k) + "von " + repr(loops) + ' ausgefuehrt')
            print X0
        print X0
        end = time.time()
        print "time to compute in s: " + repr(end-start)
        self.plot_race_process()

            	

#for k in np.arange(0, Tsim, dt):
	
#	res = minimize(cost_dist, X0, method ='SLSQP', bounds = bnds, constraints=cons)
#	print res
#	X0[0:4] = model(X0[0:6])	






