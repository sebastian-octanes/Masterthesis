from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

def example(x):
	return x[0]**2 + x[1]**2

fig = plt.figure()
ax = fig.gca(projection='3d')


# Make data.
X = np.arange(-5, 5, 0.25)
Y = np.arange(-5, 5, 0.25)
X, Y = np.meshgrid(X, Y)
#R = np.sqrt(X**2 + Y**2)
Z = X**2 + Y**2

optResult = minimize(example, np.array([2,2]), method='Nelder-Mead', tol=1e-10)
print optResult.x
print optResult.nfev

# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm)

#plt.show()
# Customize the z axis.
#ax.set_zlim(-1.01, 1.01)
#ax.zaxis.set_major_locator(LinearLocator(10))
#ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))


 
