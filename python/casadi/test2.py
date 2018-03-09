# -*- coding: utf-8 -*-
"""
Created on Wed Mar  7 15:22:00 2018

@author: weller
"""

from casadi import *
from numpy import *
from matplotlib import pyplot as plt

dt = 0.01
N = 20
lf = 0.9 
lr = 0.640  
lb = 1.440    
#dt = 0.05
max_lat_acc = 20
vmax = 100 # m/s
vmin = 0.001  # no driving backwards
psimin = -(30.0/180)*math.pi #in rad
psimax = (30.0/180)*math.pi #in rad

p = np.ndarray(shape=(N, 4,2))
p1 = [9,12]
p2 = [14,10]
p3 = [10,10]
p4 = [20,11]

for i in range(N):
    p[i][0] = p1
    p[i][1] = p2
    p[i][2] = p3
    p[i][3] = p4
     

x = SX.sym("x",(N+1) *6)
G = vertcat(x[0], x[1], x[2], x[3])

for i in range (N):

    beta = arctan((lr/(lf +lr)) * tan(x[i*6 + 5]))
    max_beta =  arctan(1.0/2 * (lf + lr) * max_lat_acc / x[(i*6) +2]**2)    
    ineq1 = -beta + max_beta
    ineq2 =  beta + max_beta
    G = vertcat(G, x[(i+1)*6   ] - (x[i*6   ] + x[(i)*6 +2] * dt * cos(x[i*6 +3] + beta)))
    G = vertcat(G, x[(i+1)*6 +1] - (x[i*6 +1] + x[(i)*6 +2] * dt * sin(x[i*6 +3] + beta)))
    G = vertcat(G, x[(i+1)*6 +2] - (x[i*6 +2] + x[(i)*6 +4] * dt))
    G = vertcat(G, x[(i+1)*6 +3] - (x[i*6 +3] + x[(i)*6 +2] * dt /lr * sin(beta)))
    G = vertcat(G, ineq1, ineq2)
 


ineq = vertcat()
for i in range(N):
    p1 = p[i][0]
    p2 = p[i][1]
    ineq1 = (x[i*6]-p1[0]) * (p2[1] - p1[1]) - (x[i*6 + 1] - p1[1]) * (p2[0]- p1[0])            
    p1 = p[i][2]
    p2 = p[i][3]
    ineq2 =((x[i*6]-p1[0]) * (p2[1] - p1[1]) - (x[i*6 + 1] - p1[1]) * (p2[0]- p1[0])) * -1
    ineq = vertcat(ineq, ineq1, ineq2)

G = vertcat(G, ineq)

J = 1/x[N*6 +2]
#J = sqrt((20 - x[N*6])**2 + (20 - x[N*6 + 1])**2)
#J = sqrt(x[0]**2 + x[1]**2)
#J = 1/(x[0] + x[1])
#for i in range(N):
    #J = J + 1/x[(i+1)* 6 + 2]
    #J = J + sqrt((0 - x[(i+1)*6])**2 + (20 - x[(i+1)* 6 +1])**2)
    #J = J + sqrt(x[(i+1)*6]**2 + x[(i+1)*6 + 1]**2)
    #J = J + 1/(x[0] + x[1])
    
nlp = {'x':x, 'f':J, 'g': G}

# Allocate an NLP solver
opts = {"ipopt.tol":1e-10, "ipopt.print_level": 3}
solver = nlpsol("solver", "ipopt", nlp, opts)


# Bounds on u and initial condition
lbg = np.array([10, 10, 10, 1.54])
ubg = np.array([10, 10, 10, 1.54])
 
lbx = np.array([-inf, -inf, vmin, -inf, -9,  psimin])
ubx = np.array([ inf,  inf, vmax,  inf,  9,  psimax])

x0 =  np.array([10, 10, 10, 0.01, 0.01 ,0.01])
#set bounds for values x is possible to use and 0 for the vehicle model 
for i in range(N):
    x0  = np.append(x0,  np.array([0.01, 0.01, 0.01, 0.01,   0.01,   0.01]))
    lbg = np.append(lbg, np.array([0, 0, 0, 0,   0,   0]))
    ubg = np.append(ubg, np.array([0, 0, 0, 0, inf, inf]))
    lbx = np.append(lbx, np.array([-inf, -inf, vmin, -inf,  -9, psimin]))
    ubx = np.append(ubx, np.array([ inf,  inf, vmax,  inf,   9, psimax]))

#add ineq 
for i in range(N):
    lbg = np.append(lbg, np.array([0,0]))
    ubg = np.append(ubg, np.array([inf,inf]))

arg = {}
arg["lbx"] = lbx
arg["ubx"] = ubx

# Bounds on g
arg["lbg"] = lbg
arg["ubg"] = ubg
arg["x0" ] = x0

N= 200
tmp = np.ndarray(shape=(N,6))
tmp2 = np.arange(N)

for i in range(N):    
    sol = solver(**arg)
    res = sol["x"].full().flatten()
    tmp[i][0:6] = res[0:6]
        
    x0[0:-6] = res[6:] 
    x0[-6:] = res[-6:]

    #update lower and upper bound    
    lbg[0:4] = res[6:10]
    ubg[0:4] = res[6:10]

plotXY = True
if plotXY:
    plt.plot(tmp[:,0], tmp[:,1])
    axes = plt.gca()
    axes.set_xlim([0,22])
    axes.set_ylim([0,22])
    plt.show()
else:
    plt.plot(tmp2, tmp[:,2])
    axes = plt.gca()
    axes.set_xlim([0,N])
    axes.set_ylim([0,20])
    #plt.plot(tmp2, tmp[:,2])
    plt.show()
    
#==============================================================================
# #update points
# p = np.ndarray(shape=(N, 4,2))
# p1 = [0,2]
# p2 = [0.1,5]
# p3 = [3.2,2]
# p4 = [3.3,6]
# 
# for i in range(N):
#     p[i][0] = p1
#     p[i][1] = p2
#     p[i][2] = p3
#     p[i][3] = p4
#  
# 
# print G
# for i in range(N):
#     p1 = p[i][0]
#     p2 = p[i][1]
#     G[4 + N*6 + i*2] = (x[i*6]-p1[0]) * (p2[1] - p1[1]) - (x[i*6 + 1] - p1[1]) * (p2[0]- p1[0])            
#     p1 = p[i][2]
#     p2 = p[i][3]
#     G[4 + N*6 + i*2 +1] =((x[i*6]-p1[0]) * (p2[1] - p1[1]) - (x[i*6 + 1] - p1[1]) * (p2[0]- p1[0])) * -1
# print "\n after :", G
# 
# res = solver(**arg)
# tmp = res['x'].full().flatten()
# 
# 
#==============================================================================

#==============================================================================
# tmp = np.arange(res["x"].size()[0])
# print tmp
# for i in range(res["x"].size()[0]):
#     if(i%6 == 0):
#         print "\n"
#     print res["x"][i]
#     tmp[i] = float( res["x"][i])
# 
# print tmp
#==============================================================================


    
    
    
    
    
    
    
    
    
    
    
    
    