from race_track import RaceTrack
from cost_function import CostFunction
from scipy_advanced_bicycle import Optimization
from vehicle_model import VehicleModel
import numpy as np
import math



track = RaceTrack()
track.complex_track()
track.plot_track()
cost = CostFunction(track)
cost.print_cost_func(2)
#==============================================================================
# 
# vehicleModel = VehicleModel(0.1)
# 
# opt = Optimization(track, cost, vehicleModel)
# opt.optimize(5,30)
# #opt.track.plot_track()
#==============================================================================


