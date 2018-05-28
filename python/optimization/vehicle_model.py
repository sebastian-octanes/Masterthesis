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
    F_long_max = 3000   # 3000N for car
    #P_engine = 39 #kW
    P_engine = 50.5 #kW
    m       = 600    # mass in kg
    g       = 9.81   #m/s²
    Cd      = 1.083   #drag coefficient
    Cf      = 25668.5
    Cr      = 25668.5 # N/rad
    rho     = 1.225  # air desity in kg/m^3
    Crr     = 0.014  #roll resistance coefficient
    Af      = 2.25   #m²
    mu	    = 0.0027   #roll resistance
    max_speed = 120/3.6 # 120km/h /3.6 = m/s
    max_long_acc = 7.46   #m/s**2 longitudinal acceleration max
    max_long_dec = 10   #m/s**2 longitudinal deceleration max
    max_lat_acc = 20  # 2g lateral acceleration
    max_steering_angle = (30.0/180.0)*math.pi #
    max_acceleration_time = 4.0 #seconds

#tire model
    Df   = 2984.0 #N
    Db   = 3274.0 #N
    xmf  = 0.25 #rad
    xmb  = 0.37 #rad
    betaf = math.pi/2.0 - 0.00001#rad
    betab = math.pi/2.0 - 0.00001#rad
    yaf  = 2952.0 #N
    yab  = 3270.0 #N #ya has to be smaller than D
 


    def __init__(self, dt_):
        self.dt = dt_

    """ this compute_next_state function is used for the equality constraint as it does not limit the max_beta. max_beta will be limited with an inequality constraint"""
    def compute_next_state(self, current_state):
 	x,y,v,orient, y_d, psi_d, acc,steer = current_state
        Xnext = np.zeros(4)
        beta = np.arctan((self.lr/(self.lf +self.lr)) * math.tan(steer))
        Xnext[0] = x + v * self.dt * math.cos(orient + beta)
        Xnext[1] = y + v * self.dt * math.sin(orient + beta)
        Xnext[3] = orient + (v*self.dt/self.lr) * math.sin(beta)
        Xnext[2] = v + acc * self.dt
        if(Xnext[2] > self.max_speed): Xnext[2] = self.max_speed
        return Xnext


    def compute_next_state_long_(self, current_state):
	#x,y,v,psi,acc,phi = current_state
	print("current_state", current_state)
  	X, Y, x_d, psi, y_d, psi_d, acc, phi = current_state

	#long
	Frx = self.F_long_max * acc/10.0
	
	#lat
	theta_f = math.atan((y_d + self.lf * psi_d)/ x_d)
	theta_r = math.atan((y_d - self.lr * psi_d)/ x_d)

	Ffy = self.pacejka_tire_model_complex(phi - theta_f, front = True)
	Fry = self.pacejka_tire_model_complex(- theta_r, front = False)

	x_d = x_d + self.dt * (Frx - Ffy*sin(phi) + self.m*y_d*psi_d)*(1.0/self.m)  	
	y_d = y_d + self.dt * (Fry + Ffy*cos(phi) - self.m*x_d*psi_d)*(1.0/self.m)

	psi_d = psi_d + self.dt * (self.lf*Ffy*cos(phi) - self.lr*Fry)/self.I

	psi = psi + self.dt * psi_d
	X = X + self.dt * (x_d * cos(psi) - y_d *sin(psi))
	Y = Y + self.dt * (x_d * sin(psi) + y_d *cos(psi))
	
	#print("x_d", x_d)	
	#print("y_d", y_d)
	Xnext = np.zeros(6)
	Xnext[0] = X
	Xnext[1] = Y
	Xnext[2] = x_d
	Xnext[3] = psi
	Xnext[4] = y_d
	Xnext[5] = psi_d
	#print("Xnext", Xnext)
	return Xnext



    def compute_next_state_long(self, current_state):
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
	#as long as state vector uses acc recompute it to power 
	power = self.P_engine * acc/10.0
	Fxl   = power * 1000/fabs(x_d)

	if(acc < 0):
		Fxl = self.F_long_max * acc/10.0
	if(Fxl > self.F_long_max):
		Fxl = self.F_long_max
	if(x_d >= 0):
		Frx = Fxl - Rxf - Rxr -Faero
	elif(x_d < 0):
		Frx = Fxl + Rxf + Rxr +Faero

	#lat
	theta_f = math.atan((y_d + self.lf * psi_d)/ x_d)
	theta_r = math.atan((y_d - self.lr * psi_d)/ x_d)

	Ffy = self.pacejka_tire_model_complex(phi - theta_f, front = True)
	Fry = self.pacejka_tire_model_complex(- theta_r, front = False)

	x_d = x_d + self.dt * (Frx - Ffy*sin(phi) + self.m*y_d*psi_d)*(1.0/self.m)  	
	y_d = y_d + self.dt * (Fry + Ffy*cos(phi) - self.m*x_d*psi_d)*(1.0/self.m)

	psi_d = psi_d + self.dt * (self.lf*Ffy*cos(phi) - self.lr*Fry)/self.I

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
	return Xnext


    def pacejka_tire_model_linear(self, slip_angle, front):
	#linear model	
	if(front == True):
		D = self.Df
		xm = self.xmf
	else:
		D = self.Db
		xm = self.xmb
	C = D/xm
	y = C * slip_angle
	return y

    def pacejka_tire_model(self, slip_angle, front):
	#dont forget that CBE have to be computed just once and then not again
	if(front == True):
		D = self.Df
		ya = self.yaf
		beta = self.betaf
	else:
		D = self.Db
		ya = self.yab
		beta = self.betab
	C = 1 + (1 - (2.0/math.pi))* np.arcsin(ya/D)
	B = math.tan(beta)/(C*D)
	y = D*sin(C*atan(B*slip_angle))	
	return y


    def pacejka_tire_model_complex(self, slip_angle, front):
	#very good approximation of our vehicle
	if(front == True):
		D = self.Df
		ya = self.yaf
		beta = self.betaf
		xm = self.xmf
	else:
		D = self.Db
		ya = self.yab
		beta = self.betab
		xm = self.xmb
	#print(ya/D)
	C = 1 + (1 - (2.0/math.pi))* np.arcsin(ya/D)
	B = math.tan(beta)/(C*D)
	E = (B * xm - math.tan(math.pi/(2.0*C)))/(B*xm - math.atan(B*xm))
	y = D*sin(C*atan(B*slip_angle - E*(B*slip_angle -atan(B*slip_angle))))
	return y

    def pacejka_tire_model_very_complex(self, slip_angle, front):
	#includes aero calculation
	if(front == True):
		D = self.Df
		ya = self.yaf
		beta = self.betaf
		xm = self.xmf
	else:
		D = self.Db
		ya = self.yab
		beta = self.betab
		xm = self.xmb
	C = 1 + (1 - (2.0/math.pi))* np.arcsin(ya/D)
	B = math.tan(beta)/(C*D)
	E = (B * xmf - math.tan(math.pi/(2.0*C)))/(B*xm - math.atan(B*xm))
	y = D*sin(C*atan(B*slip_angle - E*(B*slip_angle -atan(B*slip_angle))))
	return y



    """ use this function to compute the next state in the simulation environment only! here the max_beta will be limited in the function hence it is not usable for the mpc controller.
        use compute_next_state for the mpc controller"""
    def kinematic_model(self, current_state):
        x,y,v,orient, y_d, psi_d, acc,steer = current_state

        Xnext = np.zeros(6)
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

    def dynamic_model(self, current_state):
	X, Y, x_d, psi, y_d, psi_d, acc, phi = current_state

	#lat
	theta_f = math.atan((y_d + self.lf * psi_d)/ x_d)
	theta_r = math.atan((y_d - self.lr * psi_d)/ x_d)
	Ffy = 2 * self.pacejka_tire_model_complex(phi - theta_f, front = True)
	Fry = 2 * self.pacejka_tire_model_complex(- theta_r, front = False)

	#long
	Frx = 0
	Ffx = 0
	if(acc <= 0): 	#breaking
		Frx = self.F_long_max * (acc/10.0)* 2 
		Ffx = self.F_long_max * (acc/10.0)* 2
	else:		#accelerating
		Frx = self.P_engine * 1000* (acc/10.0) * 1.0/abs(x_d) 
		Ffx = 0
		if(Frx > 2* self.F_long_max):
			Frx = 2 * self.F_long_max
		
	Fx = Ffx + Frx
	x_d = x_d + self.dt * (Fx - Ffy*sin(phi) + self.m*y_d*psi_d)*(1.0/self.m)  	
	y_d = y_d + self.dt * (Fry + Ffy*cos(phi) - self.m*x_d*psi_d)*(1.0/self.m)

	psi_d = psi_d + self.dt * (self.lf*Ffy*cos(phi) - self.lr*Fry)/self.I

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
	return Xnext

    def kamsches_model(self, current_state):
	X, Y, x_d, psi, y_d, psi_d, acc, phi = current_state
	
	
	#lat
	theta_f = math.atan((y_d + self.lf * psi_d)/ x_d)
	theta_r = math.atan((y_d - self.lr * psi_d)/ x_d)

	Ffy = 2 * self.pacejka_tire_model_complex(phi - theta_f, front = True)
	Fry = 2 * self.pacejka_tire_model_complex(- theta_r, front = False)
	
	print("\n")
	print("Ffy", Ffy)
	print("Fry", Fry)


	#long
	Frx = 0
	Ffx = 0
	if(acc <= 0): 	#breaking
		Frx = self.F_long_max * (acc/10.0)* 2 
		Ffx = self.F_long_max * (acc/10.0)* 2
	else:		#accelerating
		Frx = self.P_engine * 1000* (acc/10.0) * 1.0/abs(x_d) 
		Ffx = 0
		if(Frx > 2* self.F_long_max):
			Frx = 2 * self.F_long_max

	#kammscher kreis
	
	Ff = np.sqrt(Ffx*Ffx + Ffy*Ffy)
	if(Ff > 2* self.F_long_max):
		alpha = np.arctan2(Ffx, Ffy)
		Ffx = math.sin(alpha) * self.F_long_max * 2
		Ffy = math.cos(alpha) * self.F_long_max * 2
	
	Fr = np.sqrt(Frx*Frx + Fry*Fry)
	if(Fr > 2* self.F_long_max):
		alpha = np.arctan2(Frx, Fry)
		Frx = math.sin(alpha) * self.F_long_max *2
		Fry = math.cos(alpha) * self.F_long_max *2
		
	Fx = Frx + Ffx		

	print("x_d", x_d)
	print("y_d", y_d)

	x_d = x_d + self.dt * (Fx - Ffy*sin(phi) + self.m*y_d*psi_d)*(1.0/self.m)  	
	y_d = y_d + self.dt * (Fry + Ffy*cos(phi) - self.m*x_d*psi_d)*(1.0/self.m)

	psi_d = psi_d + self.dt * (self.lf*Ffy*cos(phi) - self.lr*Fry)/self.I

	psi = psi + self.dt * psi_d
	X = X + self.dt * (x_d * cos(psi) - y_d *sin(psi))
	Y = Y + self.dt * (x_d * sin(psi) + y_d *cos(psi))

	if(x_d <= 0.1):
		x_d = 1.0	

	Xnext = np.zeros(6)
	Xnext[0] = X
	Xnext[1] = Y
	Xnext[2] = x_d
	Xnext[3] = psi
	Xnext[4] = y_d
	Xnext[5] = psi_d
	return Xnext




    def get_bounds(self, N):

        #define bounds fitting to N and Statevector
	bnds = ((None, None),(None, None),
                (0.1, self.max_speed),(None, None),(None, None),(None, None),
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
