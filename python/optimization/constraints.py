import numpy as np
from scipy import interpolate


class Constraints:
        
    def __init__(self, initState_, vehicleModel_ , raceTrack_):
        self.vehicleModel = vehicleModel_
        self.initState = initState_
        self.raceTrack = raceTrack_
       
            
    def set_initial_state(self, initState_):
        self.initState = initState_
        
    def constraint_fix_init_state(self, x):
        """constraints the first 4 values to not be changed as of initial state"""
        ceq = x[0:4] - self.initState
        return ceq
    
    
    """always use this constraint in combination with the ineq_constraint_vehicle_model constraint as it limits the max steering angle possible depending on speed"""
    def constraint_vehicle_model(self, x):
        N = x.size/6 -1
        ceq = np.zeros(N*4)
        for i in range(0,N,1):
            current_State = x[i*6: (i+1)*6]
            ceq[i*4: (i+1)*4] = x[(i+1)*6: (i+1)*6 + 4] - self.vehicleModel.compute_next_state(current_State)		
        #return an array of 0= Xnext_state - model(Xnow_state)	
        return ceq

    """ineq constraints always have to be positive"""    
    def ineq_constraint_vehicle_model(self, x):
        N = x.size/6 -1
        ineq = np.zeros(N*2)
        for i in range(0,N,1):
            beta = np.arctan((self.vehicleModel.lr/(self.vehicleModel.lf +self.vehicleModel.lr)) * np.tan(x[i*6 +5]))          
            beta_max = np.arctan((self.vehicleModel.lf + self.vehicleModel.lr) * self.vehicleModel.max_lat_acc *0.25 / x[i*6 +2]**2)            
            ineq[i*2] = -beta + beta_max
            ineq[i*2 +1] = beta + beta_max
        return ineq
    
        
    
    def ineq_constraint_vehicle_bounds_set_tangent_points(self, X0):
        N = X0.size/6
        self.vehicleBoundPoints = np.empty([N, 4, 2])
        for i in range(0,N,1):
            p1 = np.array([X0[6*i], X0[6*i +1]])
            arc = self.raceTrack.get_spline_arc_pos(p1)         
            #left bound
            arc = self.raceTrack.get_bnd_left_spline_arc_pos(p1)
            tck, u = self.raceTrack.get_spline_tck_bnds_left() 
            x,y = interpolate.splev(arc -0.004, tck ,der = 0)             
            p1 = np.array([x, y])        
            x,y = interpolate.splev(arc +0.004, tck ,der = 0)
            p2 = np.array([x, y]) 
            self.vehicleBoundPoints[i][0] = p1            
            self.vehicleBoundPoints[i][1] = p2            

            #right bound
            arc = self.raceTrack.get_bnd_right_spline_arc_pos(p1)
            tck, u = self.raceTrack.get_spline_tck_bnds_right()
            x,y = interpolate.splev(arc -0.004, tck ,der = 0)             
            p1 = np.array([x, y])        
            x,y = interpolate.splev(arc +0.004, tck ,der = 0)
            p2 = np.array([x, y]) 
            self.vehicleBoundPoints[i][2] = p1            
            self.vehicleBoundPoints[i][3] = p2
            
        
    def ineq_constraint_vehicle_bounds(self, X):
        N = X.size/6 
        ineq = np.zeros(N*2)
        for i in range(1,N,1):
            x = np.array([ X[i*6], X[i*6 + 1]])
            #left bound: if point is right of left bound d = positive
            p1 = self.vehicleBoundPoints[i][0]            
            p2 = self.vehicleBoundPoints[i][1]
            #d = (x[0]-p1[0]) * (p2[1] - p1[1]) - (x[1] - p1[1])*(p2[0]- p1[0])
            ineq[i*2] = (x[0]-p1[0]) * (p2[1] - p1[1]) - (x[1] - p1[1]) * (p2[0]- p1[0])            
            #right bound: if point is left of right bound d = negative keep in mind that this is only true as long the points are kpet in the same order
            p1 = self.vehicleBoundPoints[i][2]            
            p2 = self.vehicleBoundPoints[i][3] 
            ineq[i*2 +1] = -( (x[0]-p1[0]) * (p2[1] - p1[1]) - (x[1] - p1[1])*(p2[0]- p1[0]) )
        return ineq
     
     
    def ineq_const_track_bounds_casadi(self, x, opti, tangent_point_parameter):
        N = x.size1()/6    
        for i in range(0,N -1,1):
        
            p1 = tangent_point_parameter[i, 0:2]
            p2 = tangent_point_parameter[i, 2:4]
            ineq = (x[i*6]-p1[0]) * (p2[1] - p1[1]) - (x[i*6 +1] - p1[1]) * (p2[0]- p1[0]) 
            print "worked"
            opti.subject_to(ineq >= 0)
            p1 = tangent_point_parameter[i, 4:6]
            p2 = tangent_point_parameter[i, 6:8]
            ineq = ((x[i*6]-p1[0]) * (p2[1] - p1[1]) - (x[i*6 + 1] - p1[1])*(p2[0]- p1[0]))
            opti.subject_to(ineq <= 0)
    
    def ineq_const_track_casadi_update_p(self, x, opti, tangent_point_parameter):
        N = x.size1()/6 
        for i in range(0,N -1,1):
            ar = [self.vehicleBoundPoints[i][0][0], self.vehicleBoundPoints[i][0][1], self.vehicleBoundPoints[i][1][0], self.vehicleBoundPoints[i][1][1],
                  self.vehicleBoundPoints[i][2][0], self.vehicleBoundPoints[i][2][1], self.vehicleBoundPoints[i][3][0], self.vehicleBoundPoints[i][3][1]]
            opti.set_value(tangent_point_parameter[i,:],ar)