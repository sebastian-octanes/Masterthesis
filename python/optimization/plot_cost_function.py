
from matplotlib import pyplot as plt
import numpy as np
from cost_function import CostFunction
from race_track import RaceTrack

raceTrack = RaceTrack()
costFunction = CostFunction(raceTrack)
#tmp = np.linspace(-2, 2, 200)
#costFunction.print_cost_func(0)
#costFunction.print_cost_func(1)
#costFunction.print_cost_func(2)

space = np.linspace(-3.0, 3.0, 100)
cost0 =  np.zeros(space.size)
cost1 =  np.zeros(space.size)
cost2 =  np.zeros(space.size)
cost3 =  np.zeros(space.size)
for i in range(0, space.size, 1):
	cost0[i] = costFunction.cost_func0(space[i])
	cost1[i] = costFunction.cost_func1(space[i])
	cost2[i] = costFunction.cost_func2(space[i])
	cost3[i] = costFunction.cost_func3(space[i])

plt.ylim(-2, 40)
plt.xlim(-3, 3)
plt.ylabel('Kosten')
plt.xlabel('Distanz zur Mitte')

plt.plot(space,cost0, label = "alpha * (x)**2")
plt.plot(space,cost1, label = "e**(alpha*(k1 + x)) + e**(-alpha*(k2 + x))")
plt.plot(space,cost2, label = "|alpha/(k1 - x) + alpha/(k2 - x)|")
plt.plot(space,cost3, label = "alpha * |x|")
plt.legend()
plt.show()


