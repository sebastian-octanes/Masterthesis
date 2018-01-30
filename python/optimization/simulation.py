# Needed to use any and all python resources.
import pygame
import numpy as np
import math
from vehicle_model import VehicleModel



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
surf = image_surf
imagerect = image.get_rect() 
screen.blit(image,(160, 120))


# Used for timing within the program.
clock = pygame.time.Clock()
#object of vehicle_model
vehicleModel = VehicleModel()

# define state vector [x, y, v, orient, acceleration, steer_angle]
stateVector = np.zeros(6)
stateVector[3] = math.pi/2


max_steer = vehicleModel.get_max_steer_angle()
max_acc = vehicleModel.get_max_acc()
max_dec = vehicleModel.get_max_dec()

done = False
last = clock.get_time()
while not done:

        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                        done = True
                
        pressed = pygame.key.get_pressed()
        stateVector[4:] = [0, 0]
        if pressed[pygame.K_UP]: stateVector[4] = max_acc 
        if pressed[pygame.K_DOWN]: stateVector[4] = - max_dec
        if pressed[pygame.K_LEFT]: stateVector[5] = max_steer
        if pressed[pygame.K_RIGHT]: stateVector[5] = -max_steer
        if pressed[pygame.K_r]: stateVector[:] = [0,0,0,0,0,0] 
        
        time = clock.get_time() - last
        vehicleModel.set_dt(time/1000.0)
        stateVector[0:4] = vehicleModel.compute_next_state_(stateVector)
        screen.fill(black)
        surf = pygame.transform.rotate(image_surf, stateVector[3] * 180/math.pi)
        screen.blit(surf, (stateVector[0]*10, - stateVector[1] *10))
        
        textsurface = myfont.render('Speed: {0:.3f} km/h' .format(stateVector[2]*3.6), False, (255, 255, 255))        
        screen.blit(textsurface,(820,970))        
        pygame.display.flip()

        # Defines the frame rate. The number is number of frames per second.
        clock.tick(60)
    
    
    
