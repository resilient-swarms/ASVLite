import pygame
import sys
import time
import os
import threading

print ("Test")
pygame.init()
pygame.joystick.init()

print (str(pygame.joystick.get_count()))

joystick = pygame.joystick.Joystick(0)

print ("Initialization of Joystick(0)")
joystick.init()

print ("Init: ", str(joystick.get_init()))
print ("ID: ", str(joystick.get_id()))
print ("Name: ", str(joystick.get_name()))
print ("#Axes: ", str(joystick.get_numaxes()))
print ("#Balls: ", str(joystick.get_numballs()))
print ("#Buttons: ", str(joystick.get_numbuttons()))
print ("#Hats: ", str(joystick.get_numhats()))
print ("Axis: ", str(joystick.get_axis(0)))

run = True
axes = joystick.get_numaxes()

while run:
    # event = pygame.event.wait()
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            run=False # Flag that we are done so we exit this loop
        
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

        if event.type == pygame.JOYAXISMOTION:
#            print("Axis movement.")
            for i in range( axes ):
                axis = joystick.get_axis( i )
                print("Axis {} value: {:>6.3f}".format(i, axis) )

print ("Quitting...")


# QUIT             none
# ACTIVEEVENT      gain, state
# KEYDOWN          unicode, key, mod
# KEYUP            key, mod
# MOUSEMOTION      pos, rel, buttons
# MOUSEBUTTONUP    pos, button
# MOUSEBUTTONDOWN  pos, button
# JOYAXISMOTION    joy, axis, value
# JOYBALLMOTION    joy, ball, rel
# JOYHATMOTION     joy, hat, value
# JOYBUTTONUP      joy, button
# JOYBUTTONDOWN    joy, button
# VIDEORESIZE      size, w, h
# VIDEOEXPOSE      none
# USEREVENT        code
