# -*- coding: utf-8 -*-
"""
Created on Thu Jan  4 16:48:35 2018

@author: weller
"""

import numpy as np
import math
from casadi import *

class VehicleModel:
    
    lf = 1.092    # distance tire to COG
    lr = 0.9  
    lb = 1.99    #width of car
    I  = 1000    # kgm²
    A     = 2.5    #vehicle cross section
    r     = 0.2
    beta = 0.0    #slip angle
    gammaf = 0.0  #steering angle
    FEngine = 4000   # 5000N for car
    m       = 600    # mass in kg
    g       = 9.81   #m/s²
    Cd      = 1.083   #drag coefficient
    Cf      = 25668.5
    Cr      = 25668.5 # N/rad
    rho     = 1.225  # air desity in kg/m^3
    Crr     = 0.014  #roll resistance coefficient
    Af      = 2.25   #m²	
    mu	    = 0.0027   #roll resistance
    max_speed = 100/3.6 # 120km/h /3.6 = m/s
    max_long_acc = 10   #m/s**2 longitudinal acceleration max
    max_long_dec = 10   #m/s**2 longitudinal deceleration max
    max_lat_acc = 20  # 2g lateral acceleration
    max_steering_angle = (30.0/180.0)*math.pi #
    max_acceleration_time = 4.0 #seconds

#tire model
    Df   = 3000 #N
    Db   = 3274 #N
    xmf  = 0.25 #0.6 * math.pi/180 #Degrees
    xmb  = 21.3 * math.pi/180#Degrees
    betaf= 17645#rad
    betab= 1.34 #rad
    #yaf  = 2952 #N
    #yab  = 3291 #N
    yaf  = 2000 #N
    yab  = 2791 #N

 
    
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
   

    def compute_next_state_long(self, current_state):
	#x,y,v,psi,acc,phi = current_state
	#print("current_state", current_state)
  	X, Y, x_d, psi, y_d, psi_d, acc, phi = current_state

	#long
	Faero = 1/2.0 * self.rho * self.Cd * self.Af * x_d**2
	Fzf   = self.m*self.g*self.lf / (self.lf + self.lr)
	Fzr   = self.m*self.g*self.lr / (self.lf + self.lr)
	Rxf   = self.mu * Fzf
	Rxr   = self.mu * Fzr
	if (fabs(x_d) <= 0.1):
		Rxf = 0
		Rxr = 0
	#Fxl   = torque / r
	Fxl   = self.m * acc   
	x_d = x_d + self.dt * ((Fxl - Rxf - Rxr -Faero)/self.m) 
	#print("y_d * Psi_d", y_d*psi_d)

	#lat
	theta_f = math.arctan((y_d + self.lf * psi_d)/ x_d)
	theta_r = math.arctan((y_d - self.lr * psi_d)/ x_d)
#	if(theta_f > 2.0):
#		theta_f = 2.0
#	if(theta_f < -2.0):
#		theta_f = -2.0
#	if(theta_r > 2.0):
#		theta_r = 2.0
#	if(theta_r < -2.0):
#		theta_r = -2.0

	print("theta_r", theta_r)
	#Fyf  = 2 * self.Cf * ( phi - theta_f)
	Fyf = self.pacejka_tire_model_f(phi - theta_f)	
	#Fyr  = 2 * self.Cr * (-theta_r)
	Fyr = self.pacejka_tire_model_b(-theta_r)
	y_d  = y_d + self.dt * ((Fyf + Fyr)/self.m - x_d * psi_d)

	psi_d = psi_d + self.dt * (self.lf*Fyf - self.lr*Fyr)/self.I
	
	psi = psi + self.dt * psi_d
	X = X + self.dt * (x_d * cos(psi) - y_d *sin(psi))
	Y = Y + self.dt * (x_d * sin(psi) + y_d *cos(psi))
	
	Xnext = np.zeros(6)
	Xnext[0] = X
	Xnext[1] = Y
	Xnext[2] = x_d
	Xnext[3] = psi
	Xnext[4] = y_d
	Xnext[5] = psi_d
	#print("Xnext", Xnext)
	return Xnext



    def pacejka_tire_model_f(self, slip_angle):
	D = self.Df	
	C = 1 + (1 - (2.0/math.pi))* np.arcsin(self.yaf/D)
	B = math.tan(self.betaf)/C*D
	E = (B * self.xmf - math.tan(math.pi/2.0*C))/(B*self.xmf - math.atan(B*self.xmf))
	y = D*sin(C*atan(B*slip_angle - E*(B*slip_angle -math.atan(B*slip_angle))))
	return y 
    
    def pacejka_tire_model_f_(self, slip_angle):
	D = self.Df	
	C =  1.6
	B = math.tan(self.betaf)/(C*D)
	#y = D * math.sin(C * math.atan(B * slip_angle))	
	E = (B * self.xmf - math.tan(math.pi/2.0*C))/(B*self.xmf - math.atan(B*self.xmf))
	print("C: ", C)
	print("B: ", B)
	print("E: ", E)

	y = D*sin(C*atan(B*slip_angle - E*(B*slip_angle -atan(B*slip_angle))))
	return y  

    def pacejka_tire_model_b(self, slip_angle):
	D = self.Db	
	C = 1 + (1 - (2/math.pi))* np.arcsin(self.yab/D)
	B = math.tan(self.betab)/C*D
	E = (B * self.xmb - math.tan(math.pi/2*C))/(B*self.xmb - math.atan(B*self.xmb))
	y = D*sin(C*atan(B*slip_angle - E*(B*slip_angle -math.atan(B*slip_angle))))
	return y    
  

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
	#Xnext[3] = orient + self.dt * (v * math.cos(beta)/(self.lr + self.lf)) * math.tan(steer)
    
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
