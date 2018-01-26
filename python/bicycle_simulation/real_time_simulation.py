# -*- coding: utf-8 -*-
"""
Created on Thu Jan  4 15:50:01 2018

@author: weller
"""
#xbox driver found https://github.com/FRC4564/Xbox

import numpy as np
import math
import xbox
import matplotlib.pyplot as plt
from time import sleep
from Bicycle import Bicycle

#make use of xbox controller
joy = xbox.Joystick()



plt.ion()

#fig = plt.gcf()
#fig.show()
#fig.canvas.draw()

bicycle = Bicycle()

while True:
    
    if(joy.B()):
        bicycle.resetBicycle()
    #fig.clf()
    #update bicycle model with xbox input
    bicycle.setThrottlePosition(joy.leftY())
    bicycle.setSteeringAngle(joy.rightX())
        
    #compute bicycle model
    ret = bicycle.computeBicycleModel()
    #plt.plot([ret[0]-1.0, ret[0]],[ret[1]-1.0, ret[1]])
    car_body = bicycle.drawBicycleModel()
#    plt.plot([car_body[0][0], car_body[1][0], car_body[3][0], car_body[2][0]], [car_body[0][1], car_body[1][1], car_body[3][1], car_body[2][1]])
    plt.plot([car_body[0][0], car_body[1][0]], [car_body[0][1], car_body[1][1]])
    plt.plot([car_body[1][0], car_body[2][0]], [car_body[1][1], car_body[2][1]])
    plt.plot([car_body[2][0], car_body[3][0]], [car_body[2][1], car_body[3][1]])
    plt.plot([car_body[3][0], car_body[0][0]], [car_body[3][1], car_body[0][1]])
    #plt.axis(-100, 200, -100, 200)
    plt.xlim(-100, 100)
    plt.ylim(-100, 100)
    plt.xlabel("X-Axis")
    plt.ylabel("Y-Axis")
    #print car_body
    sleep(0.1)
    #fig.canvas.draw()
    plt.pause(0.05)
    plt.clf()

#close when done
joy.close()

