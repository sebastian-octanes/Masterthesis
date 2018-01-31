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
        self.track_bounds = [[0,0],[0,0]]
        self.track_width = 5
        self.simple_track()
        self.lastVehiclePosition = -1

    def add_straight(self, direction, length = 10): #always add 10m
        length = int(length) * 10
        if(direction == 0): #upwards
            last_point = self.track[-1]
            for i in range(0, length, 1):
                new_x = last_point[0]
                new_y = last_point[1] + 0.1
                self.track.append([new_x, new_y])
                last_point = self.track[-1]
        if (direction == 1):  # right
            last_point = self.track[-1]
            for i in range(0, length, 1):
                new_point = [last_point[0] + 0.1, last_point[1]]
                self.track.append(new_point)
                last_point = new_point
        if (direction == 2):  # downward
            last_point = self.track[-1]
            for i in range(0, length, 1):
                new_point = [last_point[0], last_point[1] - 0.1]
                self.track.append(new_point)
                last_point = new_point
        if (direction == 3):  # left
            last_point = self.track[-1]
            for i in range(0, length, 1):
                new_point = [last_point[0] - 0.1, last_point[1]]
                self.track.append(new_point)
                last_point = new_point

    def add_right_turn(self, direction, radius):
        if(direction == 0):
            last_point= self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] + radius - radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] + radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 1):
            last_point= self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] + radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] - radius + radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 2):
            last_point= self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] - radius + radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] - radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 3):
            last_point= self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] - radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] + radius - radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])


    def add_left_turn(self, direction, radius):             
        if(direction == 0):
            last_point = self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] - radius + radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] + radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 1):
            last_point = self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] - radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] - radius + radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 2):
            last_point= self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] + radius - radius*math.cos(math.pi*i/(2*100))
                new_y = last_point[1] - radius*math.sin(math.pi*i/(2*100))
                self.track.append([new_x, new_y])
        if(direction == 3):
            last_point= self.track[-1]
            for i in range(1,100,1):
                new_x = last_point[0] + radius*math.sin(math.pi*i/(2*100))
                new_y = last_point[1] + radius - radius*math.cos(math.pi*i/(2*100))
                self.track.append([new_x, new_y])

    def simple_track(self):
        self.track = [[0, 0]]
        self.add_straight(0, 10)     
        self.add_right_turn(0, 10)
        self.add_right_turn(1, 10)
        self.add_straight(2, 10)
        self.add_right_turn(2, 10)
        self.add_right_turn(3, 10)
        self.create_track_boundary()

    def complex_track(self):
        self.track = [[0, 0]]
        self.add_straight(0)
        self.add_right_turn(0, 10)
        self.add_right_turn(1, 10)
        self.add_left_turn(2, 10)
        self.add_straight(1,30)
        self.add_right_turn(1, 10)
        self.add_right_turn(2, 10)
        self.add_left_turn(1, 10)
        self.add_right_turn(2, 10)
        self.add_right_turn(3, 10)
        self.add_left_turn(0, 10)
        self.add_straight(3)
        self.add_right_turn(3, 10)
        self.add_straight(0)
        self.create_track_boundary()

    def create_track_boundary(self):
        print "call to create_track_boundary"
        self.track_bounds = np.empty([self.track.__len__(), 4])
        track = np.array(self.track) 
        for i in range(0, self.track.__len__() -1, 1):
            sub = track[i+1] - track[i]
            r_l = np.array([-sub[1], sub[0]])           
            r_r = np.array([sub[1], -sub[0]])
            r_no_l = np.multiply(1/math.sqrt(r_l[0]**2 + r_l[1]**2) , r_l)         
            r_no_r = np.multiply(1/math.sqrt(r_r[0]**2 + r_r[1]**2) , r_r)
            b_l = track[i] + r_no_l* self.track_width/2.0
            b_r = track[i] + r_no_r* self.track_width/2.0
            self.track_bounds[i, :2] = np.array([b_l[0], b_l[1]])
            self.track_bounds[i, 2:] = np.array([b_r[0], b_r[1]])

                        
            
    def plot_track(self):
        tmp = np.array(self.track)
        plt.plot(tmp[:,0], tmp[:,1])
        #plt.plot(self.track_bounds[:,0], self.track_bounds[:,1])
        plt.show()
    
    def plot_track_bounds(self): 
        plt.plot(self.track_bounds[:,0], self.track_bounds[:,1])
        plt.show()

    def print_track_bounds(self):
        print self.track_bounds
        
    def print_track(self):
        print self.track
   
    def get_track_points(self):
        return self.track
   
    def distance_to_track(self, pos):
        """for now this function returns the distance to the track this is done by searching for closest point and calculating the "lot" on 
        this and its next point"""
#            a = min (math.sqrt((x- pos[0])**2 + (y - pos[1])**2) for (x,y) in self.track) 
        indx = self.closest_track_point(pos, 1)
        next = indx +1
        if next == self.track.__len__():
            next = 0
        p1 = np.array([self.track[indx][0], self.track[indx][1]])
        p2 = np.array([self.track[next][0], self.track[next][1]])
        p3 = np.array([pos[0], pos[1]])
        
        dist = math.fabs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))
        
        return dist


    def closest_track_point(self, pos, tracked = -1):
        """this function returns the point on the track closest to the vehicle, if tracked >=0 it searches based of the last known position.
        To use this efficiently update the position every time the car "moves" (outer for loop) with set_new_vehicle_position"""
        indx = 0
        smallest = float("inf")
        if tracked < 0:            
            for i in range(0, self.track.__len__(), 1):
                dist = math.sqrt((self.track[i][0]- pos[0])**2 + (self.track[i][1] - pos[1])**2)
                if(dist < smallest):
                    smallest = dist
                    indx = i
        else:
            i = self.lastVehiclePosition
            if(i <0):
                i = 0
            lastValue = float("inf")
            counter = 0
            while True:
                dist = math.sqrt((self.track[i][0]- pos[0])**2 + (self.track[i][1] - pos[1])**2)
                if(dist < smallest):
                    smallest = dist
                    indx = i
                if(dist < self.track_width and dist > lastValue):
                    break
                lastValue = dist
                if(i >= self.track.__len__() -1):
                    i = 0
                else:
                    i = i+1
                if counter >= self.track.__len__():
                    break
                counter = counter +1
        return indx
        
    def get_track_point(self, indx):
        """returns koordinate of indx : (x/y)"""
        return self.track[indx]
        
    def get_track_width(self):
        return self.track_width
    
    def get_track_bounds(self):
        return self.track_bounds
    
    def set_new_vehicle_positon(self, pos):
        """this function is called every time the car position is updated in the outer loop.
        It specifies the starting point from where we search for a new closest point in the track array"""
        self.lastVehiclePosition = self.closest_track_point(pos, 1)
        
    