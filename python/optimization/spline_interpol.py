# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 11:49:12 2018

@author: weller
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


from scipy import interpolate

from race_track import RaceTrack

raceTrack = RaceTrack()
#raceTrack.complex_track()
track = raceTrack.get_track_points()

z = np.arange(0, 60, 60.0/track.__len__())
tmp = np.array(track)

tmp = np.insert(tmp, 2, z, axis= 1)


#==============================================================================
# z = np.arange(0, np.pi * 4, np.pi/4)
# x = np.cos(z)
# y = np.sin(z)
# 
# X = np.array([x, y, z])
# 
#==============================================================================
tmp = np.array(track)
tmp = np.array([tmp[:,0], tmp[:, 1]])


tck = interpolate.splprep(tmp, s=0.1)
xnew = np.arange(0, 1, 0.01)
ynew = interpolate.splev(xnew, tck[0], der=0)

print ynew


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.plot(tmp[0], tmp[1])
ax.plot(ynew[0], ynew[1] )
plt.show()



#==============================================================================
# plt.figure()
# plt.plot(x, y, 'x', xnew, ynew,  x, y, 'b')
# plt.legend(['Linear', 'Cubic Spline', 'True'])
# plt.axis([-0.05, 6.33, -1.05, 1.05])
# plt.title('Cubic-spline interpolation')
# plt.show()
# 
#==============================================================================
