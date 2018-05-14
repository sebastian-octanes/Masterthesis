
from matplotlib import pyplot as plt
import numpy as np
from vehicle_model import VehicleModel

vehicleModel = VehicleModel(0.01)
tmp = np.linspace(-2, 2, 200)
tmp2 = []
for i in range(0,200,1):
    tmp2.append(vehicleModel.pacejka_tire_model_complex(tmp[i], front = True))
print(len(tmp))
print(len(tmp2))
plt.plot(tmp[:], tmp2[:])
plt.show()

