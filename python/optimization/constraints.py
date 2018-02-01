import numpy as np


class Constraints:
        
    def __init__(self, initState_, vehicleModel_ ):
        self.vehicleModel = vehicleModel_
        self.initState = initState_
            
    def set_initial_state(self, initState_):
        self.initState = initState_
        
    def constraint_fix_init_state(self, x):
        """constraints the first 4 values to not be changed as of initial state"""
        ceq = x[0:4] - self.initState
        return ceq

    def constraint_vehicle_model(self, x):
        N = x.size/6 -1
        ceq = np.zeros(N*4)
        for i in range(0,N,1):
            current_State = x[i*6: (i+1)*6]
            ceq[i*4: (i+1)*4] = x[(i+1)*6: (i+1)*6 + 4] - self.vehicleModel.compute_next_state(current_State)		
        #return an array of 0= Xnext_state - model(Xnow_state)	
        return ceq

    def ineq_constraint_vehicle_model(self, x):
        N = x.size/6 -1
        ineq = np.zeros(N*2)
        for i in range(0,N,1):
            beta = np.arctan((self.vehicleModel.lr/(self.vehicleModel.lf +self.vehicleModel.lr)) * np.tan(x[i*6 +5]))          
            beta_max = np.arctan((self.vehicleModel.lf + self.vehicleModel.lr) * self.vehicleModel.max_lat_acc *0.25 / x[i*6 +2]**2)            
            ineq[i*2] = -beta + beta_max
            ineq[i*2 +1] = beta + beta_max
        return ineq
        
        