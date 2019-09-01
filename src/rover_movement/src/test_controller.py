#!/usr/bin/env python3
import pygame


pygame.init()
pygame.joystick.init()
_joystick = pygame.joystick.Joystick(4)
_joystick.init()
while True:
    for event in pygame.event.get():
        print(event)