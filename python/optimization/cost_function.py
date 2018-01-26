from race_track import RaceTrack
import math
import numpy as np
from matplotlib import pyplot as plt

class CostFunction:
    
    def __init__(self, track_ = RaceTrack()):  
        self.track = track_
    
    def cost_func0(self, x):
        alpha = 2/(8*(self.track.track_width/2.0)**7)
        return alpha *(x)**8
    
    def cost_func1(self, x):
        alpha = 10000
        k1 = -2.5
        k2 = 2.5
        return math.e**(alpha*(k1 + x)) + math.e**(-alpha*(k2 + x))

    def cost_func2(self, x):
        alpha = 1
        k1 = -2.5
        k2 = 2.5
        return math.fabs(alpha/(k1 - x) + alpha/(k2 - x))
        
    def cost_dist_line(self, X):
        """Take the vehicle poition and compute the distance to a vertical line at (3,0)"""
        p1 = np.array([3,0])
        p2 = np.array([3,20])
        N = X.size/6
        cost = 0.0
        for i in range(0,N,1):
            p3 = np.array([X[i*6], X[i*6 +1]])		
            cost = cost + math.fabs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))
        return cost
    
    
    def cost_dist_origin(self, X):
        """Take the vehicle position and compute the distance to the origin"""
        N = X.size/6
        cost = 0.0
        for i in range(0,N,1):
            cost = cost + math.sqrt(X[i*6]**2 + X[i*6 +1]**2)
        return cost

    def print_cost_func(self, function):
        space = np.linspace(-3.0, 3.0, 100)
        cost =  np.zeros(space.size)
        for i in range(0, space.size, 1):
            if(function == 0):
                cost[i] = self.cost_func0(space[i])
            if(function == 1):
                cost[i] = self.cost_func1(space[i])
            if(function == 2):
                cost[i] = self.cost_func2(space[i])
        plt.plot(space, cost)
        plt.show()
