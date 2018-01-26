# -*- coding: utf-8 -*-
"""
Created on Thu Jan  4 16:48:35 2018

@author: weller
"""

import numpy as np
import math

class Bicycle:
    
    #bicycle model kinematic
    x       = 0.0    # x-coordinate of center of mass
    y       = 0.0    # y-coordinate of center of mass
    omega   = 0.0    # intertial heading of car relative to inertial frame
    v       = 0.0    # speed of vehicle
    a       = 0.0    # acceleration
    lf      = 0.0    # distance tire to COG
    lr      = 0.0
    lb      = 0.0    #width of car
    beta    = 0.0    #slip angle
    dt      = 0.0    #delta t
    gammaf  = 0.0    #steering angle
    t       = 0.0    #time
    Ftime   = 0.0   #full simulation time
    mass    = 0.0   #kg
    Cd      = 0.0   #drag coefficient
    p       = 0.0  # air desity in kg/m^3
    A       = 0.0    #vehicle cross section
    FEngine = 0.0   # 50N for car
    throttle_position = 0.0

    
    def __init__(self):
        self.resetBicycle()
   
    def computeFriction(self,v):
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
        
    def computeBicycleModel(self):
        self.beta = np.arctan((self.lr/(self.lf +self.lr)) * math.tan(self.gammaf))
        self.x += self.v * self.dt * math.cos(self.omega + self.beta)
        self.y += self.v * self.dt * math.sin(self.omega + self.beta)
        self.omega += (self.v*self.dt/self.lr) * math.sin(self.beta)
        self.v += self.computeFriction(self.v) * self.dt
        self.t += self.dt
        return (self.x, self.y)
        
    def drawBicycleModel(self):
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
        
    def setThrottlePosition(self, throttle):
        if(throttle <= 1.0):
            self.throttle_position = throttle
        elif (throttle <= 100):
            self.throttle_position = throttle/100.0
        else: 
            self.throttle_position = 0.0            

    def setSteeringAngle(self, steer):
        self.gammaf = (math.pi/8) * -steer

    def resetBicycle(self):
        self.x = 0.0
        self.y = 0.0
        self.omega = 0.0    # intertial heading of car relative to inertial frame
        self.v = 0.0    # speed of vehicle
        self.a = 0.0    # acceleration
        self.lf = 0.9    # distance tire to COG
        self.lr = 0.640  
        self.lb = 1.440    #width of car
        self.beta = 0.0    #slip angle
        self.dt = 0.1    #delta t
        self.gammaf = 0.0  #steering angle
        self.t = 0.0    #time
        self.Ftime = 20.0    #full simulation time
        self.A     = 2.5    #vehicle cross section
        self.FEngine = 4000   # 5000N for car
        self.mass    = 170   #kg
        self.Cd      = 0.5   #drag coefficient
        self.p       = 1.225  # air desity in kg/m^3
        self.A       = 2.0    #vehicle cross section
        self.Crr     = 0.014  #roll resistance coefficient
        self.max_speed = 120/3.6 # 120km/h /3.6 = m/s
        self.max_lat_acc = 2  # 2g lateral acceleration
        self.max_steering_angle = 30*180/math.pi #
        self.max_acceleration_time = 4.0 #seconds
 
        
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
