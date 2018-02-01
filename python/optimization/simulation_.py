# -*- coding: utf-8 -*-
"""
Created on Thu Feb  1 12:13:19 2018

@author: weller
"""
import pygame
import numpy as np
import math
from vehicle_model import VehicleModel
from race_track import RaceTrack
from cost_function import CostFunction
from constraints import Constraints
from scipy.optimize import minimize


class SimulationEnvironment():
        
    def __init__(self):
        # Initializes all pygame functionality.
        pygame.init()

        #needed to display text in a font
        pygame.font.init()
        self.myfont = pygame.font.SysFont('Comic Sans MS', 30)

        # Set the size of the window, variables can be used for positioning
        # within program.
        window_width = 1000
        window_height = 1000

        # Creates the window and puts a Surface into "screen".
        self.screen = pygame.display.set_mode((window_width, window_height))

        # Sets title of window, not needed for the game to run but
        # unless you want to try telling everyone your game is called
        # "Game Title", make sure to set the caption :)
        pygame.display.set_caption("Vehicle_Simulation")
        
        #Create object of clock used for timing within the program.
        self.clock = pygame.time.Clock()
        self.timer_key_select = pygame.time.get_ticks()
                
        self.complexTrack = False
        self.plotTrack = False
        #is the mpcActive?
        self.mpcActive = False
        #shoud the path of the car be plottet?
        self.plotCarPath = False
        #is loop done? should always be False till termination
        self.loopSimDone = False
        self.X0 = np.zeros(6)
        #init car_path
        self.carPath = []
        self.carPath.append([self.X0[0], self.X0[1], self.X0[2]])
        #init rest of the required classes for mpc
        self.init_vehicle(0.1)
        self.init_track()
        self.init_state_vector()
        self.mpc_key_counter = 0
        self.init_mpc(4)
   
    
    def init_vehicle(self, dt):
        #load race_car image 
        raceCarImage = pygame.image.load("/home/weller/Bilder/race_car.jpg").convert()
        self.surf = pygame.transform.scale(raceCarImage, (20, 17))
        self.vehicleModel = VehicleModel(dt)
    
    
    def init_track(self):
        self.raceTrack = RaceTrack()
 
       
    def init_state_vector(self, N = 1, X = [0, 0, 0, math.pi/2, 0, 0]):
        self.X0 = np.zeros(6*(N+1))       
        self.X0[0] = X[0]
        self.X0[1] = X[1]
        self.X0[2] = X[2]
        self.X0[3] = X[3]
        self.X0[4] = X[4]
        self.X0[5] = X[5]
       
        
    def init_mpc(self, N):
        self.init_vehicle(0.1)        
        self.init_track()        
        self.costFunction = CostFunction()
        self.bnds = self.vehicleModel.get_bounds(N)
        self.init_state_vector(N, [0, 0, 0, math.pi/2, 0, 0])
        self.constraints = Constraints(self.X0[:4], self.vehicleModel)
        self.cons =({'type': 'eq', 'fun': self.constraints.constraint_fix_init_state}, 
                    {'type': 'eq', 'fun': self.constraints.constraint_vehicle_model})    


    def reset_car_position(self):
            self.X0[:6] = [0, 0, 0, math.pi/2, 0, 0]
            self.carPath = []
            self.carPath.append([self.X0[0], self.X0[1], self.X0[2]])
        

    def handle_keystrokes(self):
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                        self.loopSimDone = True
                
        pressed = pygame.key.get_pressed()
        if(pygame.time.get_ticks() - self.timer_key_select >= 500):       
            if pressed[pygame.K_r]: 
                self.reset_car_position() 
                self.timer_key_select = pygame.time.get_ticks()
            if pressed[pygame.K_p]: 
                self.plotTrack = not self.plotTrack
                self.timer_key_select = pygame.time.get_ticks()
            if pressed[pygame.K_c]:
                self.timer_key_select = pygame.time.get_ticks()
                self.complexTrack = not self.complexTrack
                if(self.complexTrack):   self.raceTrack.complex_track()
                else:                   self.raceTrack.simple_track()
            if pressed[pygame.K_m]:                
                if(self.mpc_key_counter == 0):
                    self.mpc_key_counter = self.mpc_key_counter + 1
                    self.timer_key_select = pygame.time.get_ticks()                
                    self.mpcActive = not self.mpcActive
                else: self.mpc_key_counter = 0
            if pressed[pygame.K_l]:
                self.timer_key_select = pygame.time.get_ticks()
                self.plotCarPath = not self.plotCarPath

        if(not self.mpcActive):
            self.X0[4] = 0
            self.X0[5] = 0
            if pressed[pygame.K_UP]: self.X0[4] = self.vehicleModel.get_max_acc() 
            if pressed[pygame.K_DOWN]: self.X0[4] = - self.vehicleModel.get_max_dec()
            if pressed[pygame.K_LEFT]: self.X0[5] = self.vehicleModel.get_max_steer_angle()
            if pressed[pygame.K_RIGHT]: self.X0[5] = - self.vehicleModel.get_max_steer_angle()


    def plot_race_track(self):
        if(self.plotTrack):
            points = self.raceTrack.get_track_points()
            bounds = self.raceTrack.get_track_bounds()
            for i in range(0, points.__len__(), 1):
                #offset position by 250 to set track more to the center of the screen 
                #screen.set_at((int(points[i][0]*10 +150), -int(points[i][1]*10) +350), (125,125,125))                
                self.screen.set_at((int(bounds[i][0]*10 +150), -int(bounds[i][1]*10) +350), (0,125,125))
                self.screen.set_at((int(bounds[i][2]*10 +150), -int(bounds[i][3]*10) +350), (0,125,125))


    def plot_car_path(self):
        if(self.plotCarPath):
            for i in range(0, self.carPath.__len__(), 1):
                speed =int( math.fabs(self.carPath[i][2] * 2 * 5))
                if(speed > 255): speed = 255
                self.screen.set_at((int(self.carPath[i][0]*10 +150), -int(self.carPath[i][1]*10) +350), (speed, 255-speed, 0))

   
    def set_car_path(self):
        if(not self.mpcActive):
            if(self.carPath.__len__() >= 500):
                self.carPath.pop(0)
            if(math.sqrt((self.X0[0] - self.carPath[-1][0])**2 + (self.X0[1] - self.carPath[-1][1])**2) > 0.2):
                self.carPath.append([self.X0[0], self.X0[1], self.X0[2]])
        else:
            if(self.carPath.__len__() >= 500):
                self.carPath.pop(0)
            self.carPath.append([self.X0[0], self.X0[1], self.X0[2]])            
              
    def display_menu(self):
        textsurface = self.myfont.render('Reset Car: r', False, (255, 255, 255))        
        self.screen.blit(textsurface,(20,970))        
        textsurface = self.myfont.render('Plot Racetrack: p', False, (255, 255, 255))        
        self.screen.blit(textsurface,(170,970))
        textsurface = self.myfont.render('Change Track: c', False, (255, 255, 255))        
        self.screen.blit(textsurface,(380,970)) 
        textsurface = self.myfont.render('Activate MPC: m', False, (255, 255, 255))        
        self.screen.blit(textsurface,(570,970))       
        textsurface = self.myfont.render('Plot Car Path: l', False, (255, 255, 255))        
        self.screen.blit(textsurface,(810,970)) 
    
    def display_simulation_info(self):
        textsurface = self.myfont.render('Speed: {0:.3f} km/h' .format(self.X0[2]*3.6), False, (255, 255, 255))        
        self.screen.blit(textsurface,(020,930))
        
    def simulate(self):
        while not self.loopSimDone:
            self.handle_keystrokes()
            #reset canvas
            self.screen.fill((0,0,0))
            
            if(not self.mpcActive):
                self.plot_race_track()
                self.plot_car_path()
                #plot racecar        
                self.vehicleModel.set_dt(self.clock.get_time()/1000.0)
                self.X0[0:4] = self.vehicleModel.compute_next_state_(self.X0[0:6])
                self.set_car_path()                
                #rotate car picture                 
                car_pic = pygame.transform.rotate(self.surf, self.X0[3] * 180/math.pi)
                #offset position by 250 to set car on race_track if plottet
                #print "x_position: " + repr(self.X0[0]) + "  y_position: " + repr(self.X0[1])
                self.screen.blit(car_pic, (self.X0[0]*10 +150, - self.X0[1] *10 +350))
            else:
                self.plot_race_track()
                self.plot_car_path()
                res = minimize(self.costFunction.cost_dist_track, self.X0, method ='SLSQP', bounds = self.bnds, constraints= self.cons)
                #compute movement of car later on real hardware this is piped to the actuators 
                self.X0[0:4] = self.vehicleModel.compute_next_state(res.x[0:6])
                self.raceTrack.set_new_vehicle_positon(self.X0[0:2])
                #self.X0[6:] = res.x[12:]                
                #set new init_state for constraint                
                self.constraints.set_initial_state(self.X0[0:4])                
                self.set_car_path()              
                car_pic = pygame.transform.rotate(self.surf, self.X0[3] * 180/math.pi)
                #offset position by 250 to set car on race_track if plottet
                #print "x_position: " + repr(self.X0[0]) + "  y_position: " + repr(self.X0[1])
                self.screen.blit(car_pic, (self.X0[0]*10 +150, - self.X0[1] *10 +350))
                self.display_simulation_info()
                
            self.display_menu()
            #render new picture
            pygame.display.flip()
            # Defines the frame rate. The number is number of frames per second.
            self.clock.tick(60)
        
simulation = SimulationEnvironment()
simulation.simulate()
        
                           
        