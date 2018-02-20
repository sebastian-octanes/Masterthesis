from race_track import RaceTrack
from cost_function import CostFunction
from scipy_advanced_bicycle import Optimization
from vehicle_model import VehicleModel
import numpy as np
import math
from scipy import interpolate
from matplotlib import pyplot as plt



from casadi import *





opti = casadi.Opti()
x = opti.variable(2)
#const = lambda x : x[2] + (1-x[0])**2 - x[1]

const = lambda x : x[0]+x[1]-10
opti.subject_to(const(x) >= 2)

opti.subject_to(x[0] >= 9)

min_f = lambda x: x[0]**2 + x[1]**2
#min_f = lambda x: x[0]**2 + 100*x[2]**2
opti.minimize(min_f(x))
opti.solver('ipopt')
sol = opti.solve()
print sol.value(x)


#==============================================================================
# x = SX.sym('x')
# y = SX.sym('y')
# z = SX.sym('z')
# 
# 
# nlp = {'x': vertcat(x,y,z), 'f': x**2+100*z**2, 'g': z+(1-x)**2-y }
# S=nlpsol('S', 'ipopt', nlp)
# 
# r = S(x0 = [0, 0.0, 0.75], lbg=0, ubg=0)
# x_opt = r['x']
# print('x_opt: ', x_opt)
# 
#==============================================================================
