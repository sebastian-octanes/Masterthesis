# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

#set one point every 10cm of racetrack
#define the straights
import numpy as np
import math
from matplotlib import pyplot as plt



class RaceTrack:

    def __init__(self):
        self.track = [[0, 0]]
        self.track_width = 5
        self.simple_track()
        self.trackPosition = -1

    def add_straight(self, direction): #always add 10m
        if(direction == 0): #upwards
            last_point = self.track[-1]
            for i in range(0, 100, 1):
                new_x = last_point[0]
                new_y = last_point[1] + 0.1
                self.track.append([new_x, new_y])
                last_point = self.track[-1]
        if (direction == 1):  # right
            last_point = self.track[-1]
            for i in range(0, 100, 1):
                new_point = [last_point[0] + 0.1, last_point[1]]
                self.track.append(new_point)
                last_point = new_point
        if (direction == 2):  # downward
            last_point = self.track[-1]
            for i in range(0, 100, 1):
                new_point = [last_point[0], last_point[1] - 0.1]
                self.track.append(new_point)
                last_point = new_point
        if (direction == 3):  # left
            last_point = self.track[-1]
            for i in range(0, 100, 1):
                new_point = [last_point[0] - 0.1, last_point[1]]
                self.track.append(new_point)
                last_point = new_point

    def add_right_turn(self, direction, radius):
        if(direction == 0):
            last_point= self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] + radius - radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] + radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 1):
            last_point= self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] + radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] - radius + radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 2):
            last_point= self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] - radius + radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] - radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 3):
            last_point= self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] - radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] + radius - radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])


    def add_left_turn(self, direction, radius):             
        if(direction == 0):
            last_point = self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] - radius + radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] + radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 1):
            last_point = self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] - radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] - radius + radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 2):
            last_point= self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] + radius - radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] - radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 3):
            last_point= self.track[-1]
            for i in range(0,100,1):
                new_x = last_point[0] + radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] + radius - radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])

    def simple_track(self):
        self.track = [[0, 0]]
        self.add_straight(0)
        self.add_straight(0)
        self.add_right_turn(0, 7)
        self.add_right_turn(1, 7)
        self.add_straight(2)
        self.add_straight(2)
        self.add_right_turn(2, 7)
        self.add_right_turn(3, 7)

    def complex_track(self):
        self.track = [[0, 0]]
        self.add_straight(0)
        self.add_right_turn(0, 5)
        self.add_right_turn(1, 5)
        self.add_left_turn(2, 5)
        self.add_straight(1)
        self.add_straight(1)
        self.add_right_turn(1, 5)
        self.add_right_turn(2, 5)
        self.add_left_turn(1, 5)
        self.add_right_turn(2, 5)
        self.add_right_turn(3, 5)
        self.add_left_turn(0, 5)
        self.add_straight(3)
        self.add_right_turn(3, 5)

    def plot_track(self):
        tmp = np.array(self.track)
        plt.plot(tmp[:,0], tmp[:,1])
        plt.show()

    def print_track(self):
        print self.track
    
    def distance_to_track(self, pos, trackPose):
        if (trackPose == True):
            if(self.trackPosition == -1):
                    self.trackPosition = self.closest_track_point(pos)
            else:
                indx = self.closest_track_point_tracked(pos)
                a = math.sqrt((self.track[indx][0]- pos[0])**2 + (self.track[indx][1] - pos[1])**2)
        else:
            a = min (math.sqrt((x- pos[0])**2 + (y - pos[1])**2) for (x,y) in self.track)        
        return a

    def closest_track_point_tracked(self, pos):
        indx = 0
        smallest = 100
        for i in range(self.trackPosition, self.trackPosition + 200, 1):
            dist = math.sqrt((self.track[i][0]- pos[0])**2 + (self.track[i][1] - pos[1])**2)
            if(dist < smallest):
                smallest = dist
                indx = i
        self.trackPosition = indx
        return indx
        
    
    def closest_track_point(self, pos):
        indx = 0
        smallest = 100
        for i in range(0, self.track.__len__(), 1):
            dist = math.sqrt((self.track[i][0]- pos[0])**2 + (self.track[i][1] - pos[1])**2)
            if(dist < smallest):
                smallest = dist
                indx = i

        return indx
        
    def get_track_point(self, indx):
        return self.track[indx]
        

    def cost_func1(self, x):
        alpha = 2/(8*(self.track_width/2.0)**7)
        return alpha *(x)**8
    
    def cost_func2(self, x):
        alpha = 10000
        k1 = -2.5
        k2 = 2.5
        return math.e**(alpha*(k1 + x)) + math.e**(-alpha*(k2 + x))

    def cost_func3(self, x):
        alpha = 1
        k1 = -2.5
        k2 = 2.5
        return math.fabs(alpha/(k1 - x) + alpha/(k2 - x))
            
    