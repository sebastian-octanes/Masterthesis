# -*- coding: utf-8 -*-
"""
Created on Thu Jan  4 16:48:35 2018

@author: weller
"""

import numpy as np
import math
from casadi import *

class VehicleModel:
    
    lf = 0.9    # distance tire to COG
    lr = 0.640  
    lb = 1.440    #width of car
    beta = 0.0    #slip angle
    gammaf = 0.0  #steering angle
    A     = 2.5    #vehicle cross section
    FEngine = 4000   # 5000N for car
    mass    = 170   #kg
    Cd      = 0.5   #drag coefficient
    p       = 1.225  # air desity in kg/m^3
    A       = 2.0    #vehicle cross section
    Crr     = 0.014  #roll resistance coefficient
    max_speed = 20/3.6 # 120km/h /3.6 = m/s
    max_long_acc = 10   #m/s**2 longitudinal acceleration max
    max_long_dec = 10   #m/s**2 longitudinal deceleration max
    max_lat_acc = 20  # 2g lateral acceleration
    max_steering_angle = (30.0/180.0)*math.pi #
    max_acceleration_time = 4.0 #seconds
 
    
    def __init__(self, dt_):
        self.dt = dt_
   
    """ this compute_next_state function is used for the equality constraint as it does not limit the max_beta. max_beta will be limited with an inequality constraint"""
    def compute_next_state(self, current_state):
        x,y,v,orient,acc,steer = current_state
        Xnext = np.zeros(4)
        beta = np.arctan((self.lr/(self.lf +self.lr)) * math.tan(steer))
        Xnext[0] = x + v * self.dt * math.cos(orient + beta)
        Xnext[1] = y + v * self.dt * math.sin(orient + beta)
        Xnext[3] = orient + (v*self.dt/self.lr) * math.sin(beta)
        Xnext[2] = v + acc * self.dt
        if(Xnext[2] > self.max_speed): Xnext[2] = self.max_speed
        return Xnext
   
    def vehicle_model_cassadi(self, x):
        N = x.size1()/6
        for k in range(N-1):
            beta = arctan((self.lr/(self.lf +self.lr)) * tan(x[k * 6 + 5]))
            x[(k+1)*6 + 0] = x[k*6 + 0] + x[k*6 +2] * self.dt * cos(x[k*6 + 3] + beta)
            x[(k+1)*6 + 1] = x[k*6 + 1] + x[k*6 +2] * self.dt * sin(x[k*6 + 3] + beta)
            x[(k+1)*6 + 2] = x[k*6 + 2] + x[k*6 +4] * self.dt 
            x[(k+1)*6 + 3] = x[k*6 + 3] +(x[k*6 +2]*self.dt/self.lr) * sin(beta)
  
  
    def vehicle_model_cassadi_(self, x, G, N):
        for i in range (N):
            beta = arctan((self.lr/(self.lf +self.lr)) * tan(x[i*6 + 5]))
            max_beta =  arctan(1.0/2 * (self.lf + self.lr) * self.max_lat_acc / x[(i*6) +2]**2)    
            ineq1 = -beta + max_beta
            ineq2 =  beta + max_beta
            G = vertcat(G, x[(i+1)*6   ] - (x[i*6   ] + x[(i)*6 +2] * self.dt * cos(x[i*6 +3] + beta)))
            G = vertcat(G, x[(i+1)*6 +1] - (x[i*6 +1] + x[(i)*6 +2] * self.dt * sin(x[i*6 +3] + beta)))
            G = vertcat(G, x[(i+1)*6 +2] - (x[i*6 +2] + x[(i)*6 +4] * self.dt))
            G = vertcat(G, x[(i+1)*6 +3] - (x[i*6 +3] + x[(i)*6 +2] * self.dt /self.lr * sin(beta)))
            #G = vertcat(G, ineq1, ineq2)
        return G

        
    """ use this function to compute the next state in the simulation environment only! here the max_beta will be limited in the function hence it is not usable for the mpc controller.
        use compute_next_state for the mpc controller"""
    def compute_next_state_(self, current_state):
        x,y,v,orient,acc,steer = current_state
        
        Xnext = np.zeros(4)
        beta = np.arctan((self.lr/(self.lf +self.lr)) * math.tan(steer))
        max_beta =  np.arctan(1.0/2 * (self.lf + self.lr) * self.max_lat_acc / v**2)    
        if(beta >= 0):
            beta = min(beta, max_beta)
        else:
            beta = - min(-beta, max_beta)
            
        Xnext[0] = x + v * self.dt * math.cos(orient + beta)
        Xnext[1] = y + v * self.dt * math.sin(orient + beta)
        Xnext[3] = orient + (v*self.dt/self.lr) * math.sin(beta)
        Xnext[2] = v + acc * self.dt
        if(Xnext[2] > self.max_speed): Xnext[2] = self.max_speed

        return Xnext
        
    def get_bounds(self, N):
            
        #define bounds fitting to N and Statevector          
        bnds = ((None, None),(None, None),
                (0, self.max_speed),(None, None),
                (-self.max_long_dec, self.max_long_acc),
                (-self.max_steering_angle, self.max_steering_angle))*(N + 1)
        return bnds    
     
    def set_bounds_casadi(self, x, opti):
        opti.subject_to(opti.bounded(0, x[2::6], self.max_speed))
        opti.subject_to(opti.bounded(-self.max_long_dec, x[4::6], self.max_long_acc))
        opti.subject_to(opti.bounded(-self.max_steering_angle, x[5::6], self.max_steering_angle))
   
    def get_max_steer_angle(self):
        return self.max_steering_angle

    def get_max_acc(self):
        return self.max_long_acc
        
    def get_max_dec(self):
        return self.max_long_dec

        
    def compute_friction(self,v):
        #rolling resistance: Fr = Crr * m * g
        #air resistance: Fa = pCdA/2 * v^2
        #air resistance: Plost = 0.0 p * A * v^3 * Cd
        #simplified: 0.5 * p * A * Cd * v(t)^2
        a = (self.FEngine * self.throttle_position)/self.mass  - ((self.p * self.Cd * self.A * v * v)/(2 * self.mass))     
        #add roll resistance depending on travel direction or else car will move backward if throttle is zero
        if(v > 0):
            a -=  (self.Crr * self.mass * 9.81)/self.mass
        elif (v < 0):
            a+=  (self.Crr * self.mass * 9.81)/self.mass
        print "speed is " + repr(v * 3.6) + "km/h"  
                       
        return a
        

        
    def set_throttle_position(self, throttle):
        if(throttle <= 1.0):
            self.throttle_position = throttle
        elif (throttle <= 100):
            self.throttle_position = throttle/100.0
        else: 
            self.throttle_position = 0.0            

    def set_steering_angle(self, steer):
        self.gammaf = (math.pi/8) * -steer
    
    def set_dt(self, dt_):
        self.dt = dt_
    def get_dt(self):
        return self.dt
        
    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.
    
        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point
    
        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy
