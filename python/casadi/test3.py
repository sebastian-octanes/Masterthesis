# -*- coding: utf-8 -*-
"""
Created on Fri Mar  9 10:21:20 2018

@author: weller
"""


from casadi import *
from numpy import *

dt = 0.1

#[0        1    2      3     4  ]
#[speed, acc, speed, acc, speed]
x = SX.sym("x",5)

G = vertcat(x[0], x[2] - (x[0] + x[1]*dt))
G = vertcat(G, x[4] - (x[2] + x[3]*dt))

J =1 / x[2] + 1/x[4]#speed after one step

nlp = {'x':x, 'f':J, 'g': G}

# Allocate an NLP solver
opts = {"ipopt.tol":1e-10, "ipopt.print_level": 3}
solver = nlpsol("solver", "ipopt", nlp, opts)

arg = {}
arg["lbx"] = [0, -9, 0, -9, 0]
arg["ubx"] = [60, 9, 60, 9, 60]

# Bounds on g
arg["lbg"] = [1, 0, 0]
arg["ubg"] = [1, 0, 0]
arg["x0" ] = [1, 0, 1, 0, 0.01]

res = solver(**arg)
print res["x"]




#[0   1      2   3   4      5    6   7  ]
#[x, speed, acc, x, speed, acc, x, speed]
x = SX.sym("x", 8)
x_coord = x[3] - x[0] + x[1] * dt
speed = x[4] - x[1] + x[2] * dt
G = vertcat(x[0], x[1])
G = vertcat(G, x_coord, speed)
x_coord = x[6] - x[3] + x[4] * dt
speed = x[7] - x[4] + x[5] * dt
G = vertcat(G, x_coord, speed)

J =1 / x[4] + 1/x[7]  #speed after one step

#G = vertcat(x[0], x[1], x[2], x[3])

nlp = {'x':x, 'f':J, 'g': G}

# Allocate an NLP solver
opts = {"ipopt.tol":1e-10, "ipopt.print_level": 2}
solver = nlpsol("solver", "ipopt", nlp, opts)

arg = {}
arg["lbx"] = [-inf, 0, -9, -inf,  0, -9, -inf, 0]
arg["ubx"] = [inf, 60,  9,  inf, 60,  9,  inf, 60]

# Bounds on g
arg["lbg"] = [0, 0.01, 0, 0, 0, 0]
arg["ubg"] = [0, 0.01, 0, 0, 0, 0]
arg["x0" ] = [0, 0.01, 0, 0, 0.01, 0, 0, 0.01]

res = solver(**arg)
print res["x"]
















