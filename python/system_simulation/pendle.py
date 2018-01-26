import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def pendulum(x, t, u):
	X1 = x[0]
	X2 = x[1]
	dX1dt = x[1]
	dX2dt = -10*x[0]/math.sqrt(2) + u
	xret = np.zeros(2)
	xret[0] = dX1dt
	xret[1] = dX2dt	
	return xret

def pendulum_ode(x, t):
	X1 = x[0]
	X2 = x[1]
	dX1dt = x[1]
	dX2dt = -10*x[0]/math.sqrt(2) + 0.1
	xret = np.zeros(2)
	xret[0] = dX1dt
	xret[1] = dX2dt	
	return xret

def pendulum_k(x, t, u):
	x1 = x[0]
	x2 = x[1]
	xret = np.zeros(2)
	xret[0] = x1 *0.965 + 0.099 * x2 + 0.005 * u
	xret[1] = x1 *-0.699+ 0.965 * x2 + 0.100 * u
	return xret

t = np.linspace(0,4,400)
u = np.zeros(len(t))
Xout1 = np.zeros(len(t))
Xout2 = np.zeros(len(t))

u[0:100] = 0.1
u[100:200] = -0.1
u[200:] = 0.0

x0 = np.zeros(2)
x0[0] = 0
x0[1] = 0

#imulation pendulum
y = odeint(pendulum_ode,x0,t)
print y	
#for i in range(len(t)-1):        
#	y = pendulum(x0, 0, u[i+1])
#	x0[0] += y[0]*t[1] 
#	Xout1[i] = x0[0]	
#	x0[1] += y[1]*t[1]
#	Xout2[i] = x0[1]

#for i in range(len(t)-1):        
#	y = pendulum_k(x0, 0, u[i+1])
#	x0[0] = y[0] 
#	Xout1[i] = x0[0]	
#	x0[1] = y[1]
#	Xout2[i] = x0[1]


plt.figure()
#plt.plot(t, u, 'b--', linewidth = 3)
plt.plot(t, y[:,1], 'r--', linewidth = 3)
plt.plot(t, y[:,0], 'g--', linewidth = 3)
#plt.plot(t, Xout1, 'r--', linewidth = 3)
#plt.plot(t, Xout2, 'g--', linewidth = 3)
 
plt.show()	
