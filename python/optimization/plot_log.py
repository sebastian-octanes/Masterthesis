
from matplotlib import pyplot as plt
import numpy as np


tmp = np.linspace(0, 5, 100)
tmp2 = []
mu = [0, 0.1, 1.0, 2.0, 5.0, 10.0]
for j in mu:
	for i in range(0,100,1):
	    tmp2.append(- np.log(tmp[i]) * j)
	l = "u = " + str(j)
	print l
	plt.plot(tmp[:], tmp2[:], label = l)
	tmp2 = []

plt.legend()
plt.ylim(-10, 20)
plt.ylabel('Barrierefunktion, -u*ln(x)')
plt.xlabel('Variable(x)')
plt.show()

