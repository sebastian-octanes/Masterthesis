# -*- coding: utf-8 -*-
"""
Created on Thu Jan  4 16:48:35 2018

@author: weller
"""

import numpy as np
import math

class VehicleModel:
    
    lf = 0.9    # distance tire to COG
    lr = 0.640  
    lb = 1.440    #width of car
    beta = 0.0    #slip angle
    gammaf = 0.0  #steering angle
    t = 0.0    #time
    Ftime = 20.0    #full simulation time
    A     = 2.5    #vehicle cross section
    FEngine = 4000   # 5000N for car
    mass    = 170   #kg
    Cd      = 0.5   #drag coefficient
    p       = 1.225  # air desity in kg/m^3
    A       = 2.0    #vehicle cross section
    Crr     = 0.014  #roll resistance coefficient
    max_speed = 120/3.6 # 120km/h /3.6 = m/s
    max_long_acc = 1   #1g longitudinal acceleration max
    max_long_dec = 1   #1g longitudinal deceleration max
    max_lat_acc = 3  # 2g lateral acceleration
    max_steering_angle = (30.0/180.0)*math.pi #
    max_acceleration_time = 4.0 #seconds
 
    
    def __init__(self, dt_ = 0.1):
        self.dt = dt_
   
    def compute_next_state(self, current_state):
        x,y,v,orient,acc,steer = current_state
        Xnext = np.zeros(4)
        beta = np.arctan((self.lr/(self.lf +self.lr)) * math.tan(steer))
        Xnext[0] = x + v * self.dt * math.cos(orient + beta)
        Xnext[1] = y + v * self.dt * math.sin(orient + beta)
        Xnext[2] = v + acc * self.dt
        Xnext[3] = orient + (v*self.dt/self.lr) * math.sin(beta)
        return Xnext
        
    def compute_next_state_(self, current_state):
        x,y,v,orient,acc,steer = current_state
        Xnext = np.zeros(4)
        steer_max = (self.lf + self.lr) * self.max_lat_acc / v**2
        if(steer > steer_max):
            steer = steer_max
        if(steer < -steer_max):
            steer = -steer_max
        beta = np.arctan((self.lr/(self.lf +self.lr)) * math.tan(steer))
#        max_beta =  np.arctan(1/2 * (self.lf + self.lr) * self.max_lat_acc / v**2)
        Xnext[0] = x + v * self.dt * math.cos(orient + beta)
        Xnext[1] = y + v * self.dt * math.sin(orient + beta)
        Xnext[2] = v + acc * self.dt
        Xnext[3] = orient + (v*self.dt/self.lr) * math.sin(beta)
        return Xnext
        
    def get_bounds(self, N):
            
        #define bounds fitting to N and Statevector          
        bnds = ((None, None),(None, None),
                (0, self.max_speed),(None, None),
                (self.max_long_dec, self.max_long_acc),
                (-self.max_steering_angle, self.max_steering_angle))*(N + 1)
        return bnds    
     
   
   
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
        
    def compute_vehicle_model(self):
        self.beta = np.arctan((self.lr/(self.lf +self.lr)) * math.tan(self.gammaf))
        self.x += self.v * self.dt * math.cos(self.omega + self.beta)
        self.y += self.v * self.dt * math.sin(self.omega + self.beta)
        self.omega += (self.v*self.dt/self.lr) * math.sin(self.beta)
        self.v += self.compute_friction(self.v) * self.dt
        self.t += self.dt
        return (self.x, self.y)
        
    def draw_vehicle_model(self):
        car_body = []
        p1 = [self.x + self.lf, self.y - self.lb/2]
        p2 = [self.x + self.lf, self.y + self.lb/2]
        p3 = [self.x - self.lr, self.y + self.lb/2]
        p4 = [self.x - self.lr, self.y - self.lb/2]
        
        p1 = self.rotate([self.x, self.y], p1, self.omega)
        car_body.append(p1)
        p2 = self.rotate([self.x, self.y], p2, self.omega)        
        car_body.append(p2)
        p3 = self.rotate([self.x, self.y], p3, self.omega)        
        car_body.append(p3)
        p4 = self.rotate([self.x, self.y], p4, self.omega)        
        car_body.append(p4)
        return car_body
        
    def set_throttle_position(self, throttle):
        if(throttle <= 1.0):
            self.throttle_position = throttle
        elif (throttle <= 100):
            self.throttle_position = throttle/100.0
        else: 
            self.throttle_position = 0.0            

    def set_steering_angle(self, steer):
        self.gammaf = (math.pi/8) * -steer
    
   
        
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
