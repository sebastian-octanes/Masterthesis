from race_track import RaceTrack
from cost_function import CostFunction
from scipy_advanced_bicycle import Optimization
import numpy as np
import math



track = RaceTrack()
track.complex_track()
track.plot_track()
#cost = CostFunction(track)

#==============================================================================
# p1 = np.array([2,1])
# p2 = np.array([2,3])
# sub = p2 - p1
# r_l = [-sub[1], sub[0]]
# r_r = [sub[1], -sub[0]]
# 
# r_no = 1/math.sqrt(r_l[0]**2 + r_l[1]**2) 
# r_no_l = np.multiply(r_no, r_l)
# 
# p = p1 + r_no_l*track.get_track_width()/2.0
# print p
#==============================================================================
#print left_direct_vec

#opt = Optimization()
#opt.optimize(5,30)
#opt.track.plot_track()


