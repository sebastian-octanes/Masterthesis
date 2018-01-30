from race_track import RaceTrack
from cost_function import CostFunction
from scipy_advanced_bicycle import Optimization

track = RaceTrack()
track.simple_track()
cost = CostFunction(track)

opt = Optimization()
opt.optimize(10,10)
#opt.track.plot_track()


