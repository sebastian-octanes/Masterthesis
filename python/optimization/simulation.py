# Needed to use any and all python resources.
import pygame
import numpy as np
import math
from vehicle_model import VehicleModel
from race_track import RaceTrack


# Defines common colors
black = (0, 0, 0)  # black
white = (255, 255, 255)  # white
# red: (255, 0, 0)
# purple: (255, 0, 255)
# light salmon: (255, 160, 122)

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
vehicleModel = VehicleModel()
#object of race_track
raceTrack = RaceTrack()

# define state vector [x, y, v, orient, acceleration, steer_angle]
stateVector = np.zeros(6)
stateVector[:] = [0,0,0,math.pi/2,0,0] 

done = False
complexTrack = False
plotTrack = False
simulationActive = False
plotCarPath = False
car_path = []
last = clock.get_time()
while not done:

        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                        done = True
                
        pressed = pygame.key.get_pressed()
        stateVector[4:] = [0, 0]
        if pressed[pygame.K_UP]: stateVector[4] = vehicleModel.get_max_acc() 
        if pressed[pygame.K_DOWN]: stateVector[4] = - vehicleModel.get_max_dec()
        if pressed[pygame.K_LEFT]: stateVector[5] = vehicleModel.get_max_steer_angle()
        if pressed[pygame.K_RIGHT]: stateVector[5] = - vehicleModel.get_max_steer_angle()
        if pressed[pygame.K_r]: 
            stateVector[:] = [0,0,0,math.pi/2,0,0]
            car_path = []
        if pressed[pygame.K_p]: 
            plotTrack = not plotTrack
            print plotTrack
        if pressed[pygame.K_c]:
            print complexTrack
            complexTrack = not complexTrack
            if(complexTrack):   raceTrack.complex_track()
            else: raceTrack.simple_track()
        if pressed[pygame.K_a]:
            simulationActive = not simulationActive
        if pressed[pygame.K_l]:
            plotCarPath = not plotCarPath
        #reset canvas
        screen.fill(black)
        
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
        vehicleModel.set_dt(time/1000.0)
        stateVector[0:4] = vehicleModel.compute_next_state_(stateVector)
        if(car_path.__len__() >= 500):
            car_path.pop(0)
        car_path.append([stateVector[0], stateVector[1], stateVector[2]])
        
        surf = pygame.transform.rotate(image_surf, stateVector[3] * 180/math.pi)
        #offset position by 250 to set car on race_track if plottet
        screen.blit(surf, (stateVector[0]*10 +150, - stateVector[1] *10 +350))
        
        textsurface = myfont.render('Speed: {0:.3f} km/h' .format(stateVector[2]*3.6), False, (255, 255, 255))        
        screen.blit(textsurface,(820,970))
        textsurface = myfont.render('Reset Car: r', False, (255, 255, 255))        
        screen.blit(textsurface,(020,970))        
        textsurface = myfont.render('Plot Racetrack: p', False, (255, 255, 255))        
        screen.blit(textsurface,(170,970))
        textsurface = myfont.render('Change Track: c', False, (255, 255, 255))        
        screen.blit(textsurface,(380,970)) 
        textsurface = myfont.render('Activate Simulation: a', False, (255, 255, 255))        
        screen.blit(textsurface,(570,970))       
        textsurface = myfont.render('Plot Car Path: l', False, (255, 255, 255))        
        screen.blit(textsurface,(020,930)) 
        
        
        #render new picture
        pygame.display.flip()

        # Defines the frame rate. The number is number of frames per second.
        clock.tick(60)
    
    
    
