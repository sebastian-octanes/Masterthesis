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
from scipy import interpolate

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
        self.computation_time = pygame.time.get_ticks()
        self.simulation_time = pygame.time.get_ticks()
        self.simulation_steps = 1
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
        self.init_mpc(20)
        
  
    
    def init_vehicle(self, dt):
        #load race_car image 
        raceCarImage = pygame.image.load("/home/weller/Bilder/race_car.jpg").convert()
        self.surf = pygame.transform.scale(raceCarImage, (20, 18))
        self.surf_rect = self.surf.get_rect(center = (10,9))        
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
        #self.init_vehicle(0.1)        
        self.init_track()        
        self.costFunction = CostFunction(self.raceTrack)
        self.bnds = self.vehicleModel.get_bounds(N)
        self.init_state_vector(N, [0, 0, 0, math.pi/2, 0, 0])
        print self.vehicleModel.get_dt()
        self.constraints = Constraints(self.X0[:4], self.vehicleModel, self.raceTrack)
        self.constraints.ineq_constraint_vehicle_bounds_set_tangent_points(self.X0)
        self.cons =({'type': 'eq', 'fun': self.constraints.constraint_fix_init_state}, 
                    {'type': 'eq', 'fun': self.constraints.constraint_vehicle_model},
                    {'type': 'ineq', 'fun': self.constraints.ineq_constraint_vehicle_model},
                    {'type': 'ineq', 'fun': self.constraints.ineq_constraint_vehicle_bounds})    
        self.computation_time = pygame.time.get_ticks()
        self.simulation_time = pygame.time.get_ticks()
        self.simulation_steps = 1

    def reset_car_position(self):
            self.X0[:6] = [0, 0, 0, math.pi/2, 0, 0]
            self.X0[6:] = 0
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
                    self.vehicleModel.set_dt(0.1)
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
            tck_bounds_left = self.raceTrack.get_spline_tck_bnds_left()
            tck_bounds_right = self.raceTrack.get_spline_tck_bnds_right()
            if (self.raceTrack.get_track_points().__len__() > 50):
                steps = np.arange(0, 1, 0.0001)
            else:
                steps = np.arange(0, 1, 0.001)                
            bounds_left = interpolate.splev(steps, tck_bounds_left[0], der=0)
            bounds_right = interpolate.splev(steps, tck_bounds_right[0], der=0)
            for i in range(0, steps.__len__(), 1):              
                self.screen.set_at((int(bounds_left[0][i]*10 +150), -int(bounds_left[1][i]*10) +350), (0,125,255))
                self.screen.set_at((int(bounds_right[0][i]*10 +150), -int(bounds_right[1][i]*10) +350), (0,125,255))


    def plot_car_path(self):
        if(self.plotCarPath):
            for i in range(0, self.carPath.__len__(), 1):
                speed =int( math.fabs(self.carPath[i][2] * 2 * 5))
                if(speed > 255): speed = 255
                self.screen.set_at((int(self.carPath[i][0]*10 +150), -int(self.carPath[i][1]*10) +350), (speed, 255-speed, 0))
    
    def plot_predicted_path(self, x):
        n = x.size/6   
        for i in range (0, n, 1):
            self.screen.set_at((int(x[i*6]*10 +150), -int(x[i*6 +1]*10) +350), (255,0,255))
            
    def set_car_path(self):
        if(not self.mpcActive):
            if(self.carPath.__len__() >= 1000):
                self.carPath.pop(0)
            if(math.sqrt((self.X0[0] - self.carPath[-1][0])**2 + (self.X0[1] - self.carPath[-1][1])**2) > 0.2):
                self.carPath.append([self.X0[0], self.X0[1], self.X0[2]])
        else:
            if(self.carPath.__len__() >= 1000):
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
    
    
    def display_simulation_info(self, res):
        self.computation_time = pygame.time.get_ticks() - self.computation_time
        self.full_simulation_time = pygame.time.get_ticks() - self.simulation_time
        textsurface = self.myfont.render('Solver: feasible: ' + repr(res.success) .format(self.X0[2]*3.6), False, (255, 255, 255))        
        self.screen.blit(textsurface,(020,900))
        textsurface = self.myfont.render('Solver: iterations: ' + repr(res.nit) .format(self.X0[2]*3.6), False, (255, 255, 255))        
        self.screen.blit(textsurface,(020,870))
        textsurface = self.myfont.render('Solver: nfev: ' + repr(res.nfev) .format(self.X0[2]*3.6), False, (255, 255, 255))        
        self.screen.blit(textsurface,(020,840))
        textsurface = self.myfont.render('Solver: time to compute: ' + repr(self.computation_time) + "ms".format(self.X0[2]*3.6), False, (255, 255, 255))        
        self.screen.blit(textsurface,(020,810))
        textsurface = self.myfont.render('Solver: median time to compute: ' + repr(self.full_simulation_time / self.simulation_steps) + "ms".format(self.X0[2]*3.6), False, (255, 255, 255))        
        self.screen.blit(textsurface,(020,780)) 
 
        self.computation_time = pygame.time.get_ticks()
        

    def display_car_info(self, acc_, steer):
        textsurface = self.myfont.render('Speed: {0:.3f} km/h' .format(self.X0[2]*3.6), False, (255, 255, 255))        
        self.screen.blit(textsurface,(020,930))
 
        acc = acc_
        rect = pygame.Rect(890,499,60, 2)
        self.screen.fill((255,255,255), rect)
        rect = pygame.Rect(890,649,60, 2)
        self.screen.fill((255,255,255), rect)
        rect = pygame.Rect(890,349,60, 2)
        self.screen.fill((255,255,255), rect)
        if(acc > 0):
            #norm acceleration to 1            
            tmp = acc / self.vehicleModel.get_max_acc()
            size_max = 150
            rect = pygame.Rect(900, 500 - int(size_max * tmp),40, int(size_max * tmp))
            self.screen.fill((int(tmp*255), 255 - int(255*tmp) , 0), rect)
        else:
            #norm acceleration to 1            
            tmp = - acc / self.vehicleModel.get_max_acc()
            size_max = 150
            rect = pygame.Rect(900,500 ,40, int(size_max * tmp))
            self.screen.fill((int(tmp*255), 255 - int(255*tmp), 0), rect)
            
        rect = pygame.Rect(700,700,2, 50)
        self.screen.fill((255,255,255), rect)
        rect = pygame.Rect(800,700,2, 50)
        self.screen.fill((255,255,255), rect)
        rect = pygame.Rect(900,700,2, 50)
        self.screen.fill((255,255,255), rect)
        if(steer < 0):
            #norm acceleration to 1            
            tmp = - steer / self.vehicleModel.get_max_steer_angle()
            size_max = 100
            rect = pygame.Rect(800, 710 , int(size_max * tmp) ,30)
            self.screen.fill((int(tmp*255), 255 - int(255*tmp) , 0), rect)
        else:
            #norm acceleration to 1            
            tmp =  steer / self.vehicleModel.get_max_steer_angle()
            size_max = 100
            rect = pygame.Rect(800 - int(size_max * tmp), 710, int(size_max * tmp), 30)
            self.screen.fill((int(tmp*255), 255 - int(255*tmp), 0), rect)
            
     
       
    def plot_track_bound_constraint(self, X):
        N = X.size/6
        for i in range(0, N, 3):
            arc = self.raceTrack.get_bnd_left_spline_arc_pos(X[i*6:i*6 + 2])
            #left bound line
            tck, u = self.raceTrack.get_spline_tck_bnds_left()
            x,y = interpolate.splev(arc -0.004, tck ,der = 0)             
            p1 = np.array([x, y])        
            x,y = interpolate.splev(arc +0.004, tck ,der = 0)
            p2 = np.array([x, y]) 
            angle = np.arctan((p2[1] - p1[1])/(p2[0] - p1[0]))   
            p3 = np.array([p1[0] + 7*np.cos(angle), p1[1] + 7 *np.sin(angle)])
            p4 = np.array([p1[0] - 7*np.cos(angle), p1[1] - 7 *np.sin(angle)])
            p4 = np.array([p4[0]*10 +150, -p4[1]*10 + 350])
            p3 = np.array([p3[0]*10 +150, -p3[1]*10 + 350])
            pygame.draw.line(self.screen, (255,0,0), p3 , p4, 1)
    
            #right bound line
            arc = self.raceTrack.get_bnd_right_spline_arc_pos(X[i*6: i*6 + 2])
            tck, u = self.raceTrack.get_spline_tck_bnds_right()
            x,y = interpolate.splev(arc -0.004, tck ,der = 0)             
            p1 = np.array([x, y])        
            x,y = interpolate.splev(arc +0.004, tck ,der = 0)
            p2 = np.array([x, y]) 
            angle = np.arctan((p2[1] - p1[1])/(p2[0] - p1[0]))   
            p3 = np.array([p1[0] + 7*np.cos(angle), p1[1] + 7 *np.sin(angle)])
            p4 = np.array([p1[0] - 7*np.cos(angle), p1[1] - 7 *np.sin(angle)])
            p4 = np.array([p4[0]*10 +150, -p4[1]*10 + 350])
            p3 = np.array([p3[0]*10 +150, -p3[1]*10 + 350])
            pygame.draw.line(self.screen, (255,0,0), p3 , p4, 1)

        
    def rotate(self, image, rect, angle):
        """Rotate the image while keeping its center."""
        # Rotate the original image without modifying it.
        new_image = pygame.transform.rotate(image, angle)
        # Get a new rect with the center of the old rect.
        rect = new_image.get_rect(center=rect.center)
        return new_image, rect    
        
        
    def simulate(self):
        while not self.loopSimDone:
            self.handle_keystrokes()
            #reset canvas
            self.screen.fill((0,0,0))
            rect = self.surf_rect
            if(not self.mpcActive):
                self.plot_race_track()
                self.plot_car_path()
                #plot racecar        
                self.vehicleModel.set_dt(self.clock.get_time()/1000.0)
                self.X0[0:4] = self.vehicleModel.compute_next_state_(self.X0[0:6])
                self.display_car_info(self.X0[4], self.X0[5])
                self.set_car_path()                
                #rotate car picture                 
                #car_pic = pygame.transform.rotate(self.surf, self.X0[3] * 180/math.pi)
                pic, rect = self.rotate(self.surf, rect, self.X0[3] * 180/math.pi)
                rect.center = [self.X0[0]*10 +150, - self.X0[1] *10 +350]
                self.screen.blit(pic, rect)
                

            else:
                self.plot_race_track()
                self.plot_car_path()
                res = minimize(self.costFunction.cost_dist_track_speed, self.X0, method ='SLSQP', bounds = self.bnds, constraints= self.cons)               
                #compute movement of car later on real hardware this is piped to the actuators 
                self.X0[0:4] = self.vehicleModel.compute_next_state_(res.x[0:6])
                #shift the state vector one step to the left and predict last step for constraint handling
                self.X0[4:-6] = res.x[10:]
                self.X0[-6:-2] = self.vehicleModel.compute_next_state_(self.X0[-12:-6])
                self.constraints.ineq_constraint_vehicle_bounds_set_tangent_points(self.X0)
                #print self.constraints.ineq_constraint_vehicle_bounds(self.X0)
                                               
                #self.raceTrack.set_new_vehicle_positon(self.X0[0:2])
                #set new init_state for constraint                
                self.constraints.set_initial_state(self.X0[0:4])                
                self.set_car_path()
                self.display_car_info(res.x[4], res.x[5])
                self.plot_predicted_path(res.x)
                self.plot_track_bound_constraint(res.x)
                self.display_simulation_info(res)
                #used for calculation of median value for solver
                self.simulation_steps = self.simulation_steps + 1
                #plot racecar
                pic, rect = self.rotate(self.surf, rect, self.X0[3] * 180/math.pi)
                rect.center = [self.X0[0]*10 +150, - self.X0[1] *10 +350]
                self.screen.blit(pic, rect)
                
            self.display_menu()
            #render new picture
            pygame.display.flip()
            # Defines the frame rate. The number is number of frames per second.
            self.clock.tick(60)
        
simulation = SimulationEnvironment()
simulation.simulate()
        
                           
        