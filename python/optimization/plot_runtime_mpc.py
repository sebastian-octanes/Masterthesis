
from matplotlib import pyplot as plt
import numpy as np



n = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12,13,14,15,16,17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,31, 35]
tim = [0.014, 0.028, 0.041, 0.043, 0.046, 0.052, 0.0577, 0.063, 0.07, 0.093, 0.11,0.129,0.133,0.20,0.239,0.27,0.37, 0.4 ,0.67, 0.7, 0.5, 0.57, 0.62, 0.7, 0.8, 0.91, 0.9, 0.96, 1.157, 2.3, 6.8, 10.6]

plt.plot(n[:], tim[:])


plt.legend()
#plt.ylim(-10, 20)
plt.ylabel('Durchschnittliche Berechnungszeit in s')
plt.xlabel('Praediktionsschritte (N)')
plt.show()

