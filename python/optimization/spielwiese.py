from race_track import RaceTrack
from cost_function import CostFunction
from scipy_advanced_bicycle import Optimization
from vehicle_model import VehicleModel
import numpy as np
import math
from scipy import interpolate
from matplotlib import pyplot as plt

track = RaceTrack()

tck, u = track.get_spline_tck()
ti = np.linspace(0,1, 400)
dxdt, dydt = interpolate.splev(ti, tck, der=1)

#track.plot_track_spline()
#plt.plot(dxdt, ti)
#plt.plot(dydt, ti)
#plt.plot(dxdt , ti)
#plt.show()

pos = (0.7, 25)
print track.distance_to_track_spline(pos)

p1 = (-1, 1)
p2 = (2, 5)
x = (-3, 2)

d = (x[0]-p1[0]) * (p2[1] - p1[1]) - (x[1] - p1[1])*(p2[0]- p1[0])
print d