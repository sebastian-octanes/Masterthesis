from race_track import RaceTrack
from cost_function import CostFunction
from scipy_advanced_bicycle import Optimization

track = RaceTrack()
cost = CostFunction(track)


opt = Optimization()
opt.optimize(20)
#opt.track.plot_track()


