import pygame
import math
import sc_services as scsvc


pygame.init()
pygame.joystick.init()

print(pygame.joystick.get_count())
joystick = pygame.joystick.Joystick(0)

while True:
    pygame.event.pump()
    print("Y:", joystick.get_axis(0), " - X:", joystick.get_axis(1))


