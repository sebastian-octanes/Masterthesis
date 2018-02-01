# Needed to use any and all python resources.
import pygame
import numpy as np
import math
from vehicle_model import VehicleModel
from race_track import RaceTrack
from cost_function import CostFunction
from constraints import Constraints
from scipy.optimize import minimize


# Initializes all pygame functionality.
pygame.init()

#needed to display text in a font
pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)

# Set the size of the window, variables can be used for positioning
# within program.
window_width = 1000
window_height = 1000

# Creates the window and puts a Surface into "screen".
screen = pygame.display.set_mode((window_width, window_height))

# Sets title of window, not needed for the game to run but
# unless you want to try telling everyone your game is called
# "Game Title", make sure to set the caption :)
pygame.display.set_caption("Vehicle_Simulation")


#load race_car image 
image = pygame.image.load("/home/weller/Bilder/race_car.jpg").convert()
image_surf = pygame.transform.scale(image, (20, 17))
#image_surf = pygame.transform.rotate(image_surf, 90)
surf = image_surf
imagerect = image.get_rect() 
screen.blit(image,(160, 120))


# Used for timing within the program.
clock = pygame.time.Clock()
#object of vehicle_model
vehicleModel = VehicleModel(0.1)
#object of race_track
raceTrack = RaceTrack()

#CostFunction for optimization
costFunction = CostFunction()
N = 3

bnds = vehicleModel.get_bounds(N) 

X0 = np.zeros(6*(N+1))
X0[1] = 7.0
X0[2] = 3.0
X0[3] = math.pi/2
constraints = Constraints(X0[:4], vehicleModel)
cons =({'type': 'eq', 'fun': constraints.constraint_fix_init_state}, 
       {'type': 'eq', 'fun': constraints.constraint_vehicle_model})    


# define state vector [x, y, v, orient, acceleration, steer_angle]
stateVector = np.zeros(6)
stateVector[:] = [0,7,0,math.pi/2,0,0] 

done = False
complexTrack = False
plotTrack = False
mpcActive = False
plotCarPath = False
block_k_key = False
block_k_key_counter = 0
car_path = []
car_path.append([stateVector[0], stateVector[1], stateVector[2]])
last = clock.get_time()
timer_key_select = clock.get_time()
while not done:

        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                        done = True
                
        pressed = pygame.key.get_pressed()

        if pressed[pygame.K_r]: 
            stateVector[:] = [0,0,0,math.pi/2,0,0]
            car_path = []
            car_path.append([stateVector[0], stateVector[1], stateVector[2]])
        if pressed[pygame.K_p]:
            if(pygame.time.get_ticks() - timer_key_select >= 500):
                plotTrack = not plotTrack
                timer_key_select = pygame.time.get_ticks()
        if pressed[pygame.K_c]:
            if(pygame.time.get_ticks() - timer_key_select >= 500):
                timer_key_select = pygame.time.get_ticks()
                complexTrack = not complexTrack
            if(complexTrack):   raceTrack.complex_track()
            else: raceTrack.simple_track()
        if pressed[pygame.K_m]:
            if(pygame.time.get_ticks() - timer_key_select >= 500 and (not block_k_key)):       
                mpcActive = not mpcActive
                if(mpcActive):
                    block_k_key = True
                X0 = np.zeros(6*(N+1))
                X0[1] = 7.0                
                X0[2] = 3.0
                X0[3] = math.pi/2
                timer_key_select = pygame.time.get_ticks()
        if pressed[pygame.K_l]:
            if(pygame.time.get_ticks() - timer_key_select >= 500):
                timer_key_select = pygame.time.get_ticks()
                plotCarPath = not plotCarPath
        #reset canvas
        screen.fill((0,0,0))
        
        if(block_k_key_counter >= 2):
            block_k_key = False
            block_k_key_counter = 0
        
        if(mpcActive):
            if(block_k_key):
                block_k_key_counter = block_k_key_counter + 1
             #plot race_track
            if(plotTrack):
                points = raceTrack.get_track_points()
                bounds = raceTrack.get_track_bounds()
                for i in range(0, points.__len__(), 1):
                    #offset position by 250 to set track more to the center of the screen 
                    #screen.set_at((int(points[i][0]*10 +150), -int(points[i][1]*10) +350), (125,125,125))
                    screen.set_at((int(bounds[i][0]*10 +150), -int(bounds[i][1]*10) +350), (0,125,125))
                    screen.set_at((int(bounds[i][2]*10 +150), -int(bounds[i][3]*10) +350), (0,125,125))
            #plot racecar path
            if(plotCarPath):
                for i in range(0, car_path.__len__(), 1):
                    speed =int( math.fabs(car_path[i][2] * 2 * 5))
                    if(speed > 255): speed = 255
                    screen.set_at((int(car_path[i][0]*10 +150), -int(car_path[i][1]*10) +350), (speed, 255-speed, 0))
            
            #optimization
            res = minimize(costFunction.cost_dist_track_speed, X0, method ='SLSQP', bounds = bnds, constraints=cons)
            x_new = vehicleModel.compute_next_state(res.x[0:6]) 
            X0[0:4] = x_new
            constraints.set_initial_state(X0[0:4])            
            #add car positioins to plot
            if(car_path.__len__() >= 500):
                car_path.pop(0)
            #if(math.sqrt((stateVector[0] - car_path[-1][0])**2 + (stateVector[1] - car_path[-1][1])**2) > 0.2):
            car_path.append([x_new[0], x_new[1], x_new[2]])
            
            surf = pygame.transform.rotate(image_surf, x_new[3] * 180/math.pi)
            #offset position by 250 to set car on race_track if plottet
            screen.blit(surf, (x_new[0]*10 +150, - x_new[1] *10 +350))
        
            textsurface = myfont.render('Simulation active', False, (255, 0, 0))        
            screen.blit(textsurface,(570, 930))
            textsurface = myfont.render('Speed: {0:.3f} km/h' .format(x_new[2]*3.6), False, (255, 255, 255))        
            screen.blit(textsurface,(810,970))
        
        #control car with keyboard
        else:
            stateVector[4:] = [0, 0]
            if pressed[pygame.K_UP]: stateVector[4] = vehicleModel.get_max_acc() 
            if pressed[pygame.K_DOWN]: stateVector[4] = - vehicleModel.get_max_dec()
            if pressed[pygame.K_LEFT]: stateVector[5] = vehicleModel.get_max_steer_angle()
            if pressed[pygame.K_RIGHT]: stateVector[5] = - vehicleModel.get_max_steer_angle()
            #plot race_track
            if(plotTrack):
                points = raceTrack.get_track_points()
                bounds = raceTrack.get_track_bounds()
                for i in range(0, points.__len__(), 1):
                    #offset position by 250 to set track more to the center of the screen 
                    screen.set_at((int(points[i][0]*10 +150), -int(points[i][1]*10) +350), (125,125,125))                
                    screen.set_at((int(bounds[i][0]*10 +150), -int(bounds[i][1]*10) +350), (0,125,125))
                    screen.set_at((int(bounds[i][2]*10 +150), -int(bounds[i][3]*10) +350), (0,125,125))
            #plot racecar path
            if(plotCarPath):
                for i in range(0, car_path.__len__(), 1):
                    speed =int( math.fabs(car_path[i][2] * 2 * 5))
                    if(speed > 255): speed = 255
                    screen.set_at((int(car_path[i][0]*10 +150), -int(car_path[i][1]*10) +350), (speed, 255-speed, 0))
            #plot racecar        
            time = clock.get_time() - last
            print time
            vehicleModel.set_dt(time/1000.0)
            stateVector[0:4] = vehicleModel.compute_next_state_(stateVector)
            if(car_path.__len__() >= 500):
                car_path.pop(0)
            if(math.sqrt((stateVector[0] - car_path[-1][0])**2 + (stateVector[1] - car_path[-1][1])**2) > 0.2):
                car_path.append([stateVector[0], stateVector[1], stateVector[2]])
            
            surf = pygame.transform.rotate(image_surf, stateVector[3] * 180/math.pi)
            #offset position by 250 to set car on race_track if plottet
            screen.blit(surf, (stateVector[0]*10 +150, - stateVector[1] *10 +350))
            
            textsurface = myfont.render('Speed: {0:.3f} km/h' .format(stateVector[2]*3.6), False, (255, 255, 255))        
            screen.blit(textsurface,(810,970))
        textsurface = myfont.render('Reset Car: r', False, (255, 255, 255))        
        screen.blit(textsurface,(20,970))        
        textsurface = myfont.render('Plot Racetrack: p', False, (255, 255, 255))        
        screen.blit(textsurface,(170,970))
        textsurface = myfont.render('Change Track: c', False, (255, 255, 255))        
        screen.blit(textsurface,(380,970)) 
        textsurface = myfont.render('Activate MPC: m', False, (255, 255, 255))        
        screen.blit(textsurface,(570,970))       
        textsurface = myfont.render('Plot Car Path: l', False, (255, 255, 255))        
        screen.blit(textsurface,(20,930)) 
            
            
        #render new picture
        pygame.display.flip()
    
        # Defines the frame rate. The number is number of frames per second.
        clock.tick(60)
        
    
    
