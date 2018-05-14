from cost_function import CostFunction
from race_track import RaceTrack
from vehicle_model import VehicleModel
import numpy as np
from matplotlib import pyplot as plt

track = RaceTrack()
cost = CostFunction(track)
cost.print_cost_func(1)


