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
from scipy import interpolate


class RaceTrack:

    def __init__(self):
        self.track = [[0, 0]]
        self.track_bounds = [[-2.5, 0, 2.5, 0]]
        self.track_width = 5
        self.simple_track()
        self.lastVehiclePosition = -1
        self.arc_pos = 0

    def simple_track(self):
        self.track = [[0, 0]]
        self.track_bounds = [[-2.5, 0, 2.5, 0]]
        self.add_straight(0, 10)     
        self.add_right_turn(0, 10)
        self.add_right_turn(1, 10)
        self.add_straight(2, 10)
        self.add_right_turn(2, 10)
        self.add_right_turn(3, 10)
        self.create_spline_tck()
        self.create_spline_tck_bnds_left()
        self.create_spline_tck_bnds_right()

    def complex_track(self):
        self.track = [[0, 0]]
        self.track_bounds = [[-2.5, 0, 2.5, 0]]
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
        self.create_spline_tck()
        self.create_spline_tck_bnds_left()
        self.create_spline_tck_bnds_right()

    def create_spline_tck(self):
        tmp = np.array(self.track)
        tmp = np.array([tmp[:,0], tmp[:, 1]])
        self.tck = interpolate.splprep(tmp, s=0.1)      

    def create_spline_tck_bnds_left(self):
        tmp = np.array(self.track_bounds)
        tmp = np.array([tmp[:,0], tmp[:, 1]])
        self.tck_bnds_left = interpolate.splprep(tmp, s=0.1)                  
    
    def create_spline_tck_bnds_right(self):
        tmp = np.array(self.track_bounds)
        tmp = np.array([tmp[:,2], tmp[:, 3]])
        self.tck_bnds_right = interpolate.splprep(tmp, s=0.1)                  
            

    def get_spline_tck(self):
        return self.tck
        
    def get_spline_tck_bnds_left(self):
        return self.tck_bnds_left

    def get_spline_tck_bnds_right(self):
        return self.tck_bnds_right
        
        
    def plot_track(self):
        tmp = np.array(self.track)
        plt.plot(tmp[:,0], tmp[:,1])
        plt.show()
        
    def plot_track_spline(self):
        ti = np.linspace(0,1, 1000)
        x = interpolate.splev(ti, self.tck[0], der=0)
        plt.plot(x[0], x[1])
        
    def plot_track_bounds(self): 
        tmp = np.array(self.track_bounds)        
        plt.plot(tmp[:,0], tmp[:,1])
        plt.show()

    def print_track_bounds(self):
        print self.track_bounds
        
    def print_track(self):
        print self.track
   
    def get_track_points(self):
        return self.track
   
    def get_spline_arc_pos(self, pos):
        ti = np.linspace(0,1, 400)
        x,y = interpolate.splev(ti, self.tck[0], der=0)
        indx = 0
        smallest = float("inf")       
        for i in range (0, 400, 1):
            dist = math.sqrt((x[i] - pos[0])**2 + (y[i] - pos[1])**2) 
            if dist < smallest:
                smallest = dist
                indx = i
        self.arc_pos = indx/400.0
        return self.arc_pos

    def get_bnd_left_spline_arc_pos(self, pos):
        ti = np.linspace(0,1, 400)
        x,y = interpolate.splev(ti, self.tck_bnds_left[0], der=0)
        indx = 0
        smallest = float("inf")       
        for i in range (0, 400, 1):
            dist = math.sqrt((x[i] - pos[0])**2 + (y[i] - pos[1])**2) 
            if dist < smallest:
                smallest = dist
                indx = i
        self.arc_pos = indx/400.0
        return self.arc_pos

    def get_bnd_right_spline_arc_pos(self, pos):
        ti = np.linspace(0,1, 400)
        x,y = interpolate.splev(ti, self.tck_bnds_right[0], der=0)
        indx = 0
        smallest = float("inf")       
        for i in range (0, 400, 1):
            dist = math.sqrt((x[i] - pos[0])**2 + (y[i] - pos[1])**2) 
            if dist < smallest:
                smallest = dist
                indx = i
        self.arc_pos = indx/400.0
        return self.arc_pos

    def distance_to_track_spline_tracked(self, pos):
        arc = self.arc_pos 
        indx = 0
        smallest = float("inf") 
        last = float("inf")
        while arc < 1.0:
            x,y = interpolate.splev(arc, self.tck[0], der=0)    
            dist = math.sqrt((x - pos[0])**2 + (y - pos[1])**2) 
            if dist < smallest:
                smallest = dist
                indx = arc
            if dist > last:
                break
            last = dist
            arc = arc + 0.01
       
        x,y = interpolate.splev(indx -0.001, self.tck[0] ,der = 0)             
        p1 = np.array([x, y])        
        x,y = interpolate.splev(indx +0.001, self.tck[0] ,der = 0)
        p2 = np.array([x, y])    
        p3 = np.array([pos[0], pos[1]])
        dist = math.fabs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))        
        return dist
        
        
    def distance_to_track_spline(self, pos):
        ti = np.linspace(0,1, 400)
        x,y = interpolate.splev(ti, self.tck[0], der=0)
        indx = 0
        smallest = float("inf")       
        for i in range (0, 400, 1):
            dist = math.sqrt((x[i] - pos[0])**2 + (y[i] - pos[1])**2) 
            if dist < smallest:
                smallest = dist
                indx = i             
                
        x,y = interpolate.splev(indx/400.0 -0.001, self.tck[0] ,der = 0)             
        p1 = np.array([x, y])        
        x,y = interpolate.splev(indx/400.0 +0.001, self.tck[0] ,der = 0)
        p2 = np.array([x, y])    
        p3 = np.array([pos[0], pos[1]])
        dist = math.fabs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))        
        return dist
        
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
        self.get_spline_arc_pos(pos)
     
     
    def add_straight(self, direction, length = 10.0): 
        #always add 4 points per straight
        if(direction == 0): #upwards
            last_point = self.track[-1]
            for i in range(1, 5, 1):
                new_point = [last_point[0], last_point[1] + length/4.0]
                self.track.append(new_point)
                new_point_bnd_left = [last_point[0] - self.track_width/2.0, last_point[1] + length/4.0] 
                new_point_bnd_right= [last_point[0] + self.track_width/2.0, last_point[1] + length/4.0] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
                last_point = self.track[-1]
        if (direction == 1):  # right
            last_point = self.track[-1]
            for i in range(0, 4, 1):
                new_point = [last_point[0] + length/4.0, last_point[1]]
                self.track.append(new_point)
                new_point_bnd_left = [last_point[0] + length/4.0, last_point[1] + self.track_width/2.0] 
                new_point_bnd_right= [last_point[0] + length/4.0, last_point[1] - self.track_width/2.0] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
                last_point = new_point
        if (direction == 2):  # downward
            last_point = self.track[-1]
            for i in range(0, 4, 1):
                new_point = [last_point[0], last_point[1] - length/4.0]
                self.track.append(new_point)
                new_point_bnd_left = [last_point[0] + self.track_width/2.0, last_point[1] - length/4.0] 
                new_point_bnd_right= [last_point[0] - self.track_width/2.0, last_point[1] - length/4.0] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
                last_point = new_point
        if (direction == 3):  # left
            last_point = self.track[-1]
            for i in range(0, 4, 1):
                new_point = [last_point[0] - length/4.0, last_point[1]]
                self.track.append(new_point)
                new_point_bnd_left = [last_point[0] - length/4.0, last_point[1] - self.track_width/2.0] 
                new_point_bnd_right= [last_point[0] - length/4.0, last_point[1] + self.track_width/2.0] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
                last_point = new_point

    def add_right_turn(self, direction, radius):
        steps = np.arange(0.0, np.pi/2 + np.pi/8, np.pi/8)
        radius_minus = radius - self.track_width/2.0
        radius_plus = radius + self.track_width/2.0
        if(direction == 0):
            last_point= self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] + radius - radius*math.cos(steps[i])
                new_y = last_point[1] + radius*math.sin(steps[i])
                self.track.append([new_x, new_y])
                new_point_bnd_left = [last_point[0] + radius - radius_plus * math.cos(steps[i]) , last_point[1] + radius_plus*math.sin(steps[i])] 
                new_point_bnd_right= [last_point[0] + radius - radius_minus *math.cos(steps[i]) , last_point[1] + radius_minus*math.sin(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])                        
        if(direction == 1):
            last_point= self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] + radius*math.sin(steps[i])
                new_y = last_point[1] - radius + radius*math.cos(steps[i])
                self.track.append([new_x, new_y])
                new_point_bnd_left = [last_point[0] + radius_plus * math.sin(steps[i]) , last_point[1] -radius + radius_plus* math.cos(steps[i])] 
                new_point_bnd_right= [last_point[0] + radius_minus *math.sin(steps[i]) , last_point[1] -radius + radius_minus*math.cos(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])                
        if(direction == 2):
            last_point= self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] - radius + radius*math.cos(steps[i])
                new_y = last_point[1] - radius*math.sin(steps[i])
                self.track.append([new_x, new_y])
                new_point_bnd_left = [last_point[0] - radius + radius_plus * math.cos(steps[i]) , last_point[1] - radius_plus*math.sin(steps[i])] 
                new_point_bnd_right= [last_point[0] - radius + radius_minus *math.cos(steps[i]) , last_point[1] - radius_minus*math.sin(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])                
        if(direction == 3):
            last_point= self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] - radius*math.sin(steps[i])
                new_y = last_point[1] + radius - radius*math.cos(steps[i])
                self.track.append([new_x, new_y])
                new_point_bnd_left = [last_point[0] - radius_plus * math.sin(steps[i]) , last_point[1] + radius - radius_plus* math.cos(steps[i])] 
                new_point_bnd_right= [last_point[0] - radius_minus *math.sin(steps[i]) , last_point[1] + radius - radius_minus*math.cos(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])            


    def add_left_turn(self, direction, radius):
        steps = np.arange(0.0, np.pi/2 + np.pi/8, np.pi/8)
        radius_minus = radius - self.track_width/2.0
        radius_plus = radius + self.track_width/2.0
        if(direction == 0):
            last_point = self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] - radius + radius*math.cos(steps[i])
                new_y = last_point[1] + radius*math.sin(steps[i])
                self.track.append([new_x, new_y])
                new_point_bnd_left = [last_point[0] - radius + radius_minus*math.cos(steps[i]) , last_point[1] + radius_minus*math.sin(steps[i])] 
                new_point_bnd_right= [last_point[0] - radius + radius_plus *math.cos(steps[i]) , last_point[1] + radius_plus *math.sin(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
        if(direction == 1):
            last_point = self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] - radius*math.sin(steps[i])
                new_y = last_point[1] - radius + radius*math.cos(steps[i])
                self.track.append([new_x, new_y])
                new_point_bnd_left = [last_point[0] - radius_minus*math.sin(steps[i]) , last_point[1] - radius + radius_minus* math.cos(steps[i])] 
                new_point_bnd_right= [last_point[0] - radius_plus *math.sin(steps[i]) , last_point[1] - radius + radius_plus *math.cos(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])   
        if(direction == 2):
            last_point= self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] + radius - radius*math.cos(steps[i])
                new_y = last_point[1] - radius*math.sin(steps[i])
                self.track.append([new_x, new_y])                
                new_point_bnd_left = [last_point[0] + radius - radius_minus*math.cos(steps[i]) , last_point[1] - radius_minus*math.sin(steps[i])] 
                new_point_bnd_right= [last_point[0] + radius - radius_plus *math.cos(steps[i]) , last_point[1] - radius_plus *math.sin(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]]) 
        if(direction == 3):
            last_point= self.track[-1]
            for i in range(1,steps.__len__(),1):
                new_x = last_point[0] + radius*math.sin(steps[i])
                new_y = last_point[1] + radius - radius*math.cos(steps[i])
                self.track.append([new_x, new_y])
                new_point_bnd_left = [last_point[0] + radius_minus *math.sin(steps[i]) , last_point[1] + radius - radius_minus*math.cos(steps[i])] 
                new_point_bnd_right= [last_point[0] + radius_plus * math.sin(steps[i]) , last_point[1] + radius - radius_plus* math.cos(steps[i])] 
                self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])        