#!/usr/bin/env python

import pygame
# allow multiple joysticks
joy = []
# handle joystick event

def handleJoyEvent(e):
	
    if e.type == pygame.JOYAXISMOTION:
        axis = "unknown"
        if (e.dict['axis'] == 0):
            axis = "X"
        if (e.dict['axis'] == 1):
            axis = "Y"
        if (e.dict['axis'] == 2):
            axis = "Throttle"
        if (e.dict['axis'] == 3):
            axis = "Z"
        if (axis != "unknown"):
            #pos = e.dict['value']
            return e.dict
            
    elif e.type == pygame.JOYBUTTONDOWN:
        str = "Button: %d" % (e.dict['button'])
        # uncomment to debug
        #output(str, e.dict['joy'])
        # Button 0 (trigger) to quit
        if (e.dict['button'] == 0):
            return e.dict
            print ("Bye!\n")
            #quit()
    else:
        pass
# print the joystick position
def output(line, stick):
    print ("Joystick: %d; %s" % (stick, line))

# main method
def main():
    # initialize pygame
    pygame.joystick.init()
    pygame.display.init()
    if not pygame.joystick.get_count():
        print ("\nPlease connect a joystick and run again.\n")
        quit()
    print ("\n%d joystick(s) detected." % pygame.joystick.get_count())
    for i in range(pygame.joystick.get_count()):
        myjoy = pygame.joystick.Joystick(i)
        myjoy.init()
        joy.append(myjoy)
        print ("Joystick %d: " % (i) + joy[i].get_name())
    print ("Depress trigger (button 0) to quit.\n")
    # run joystick listener loop
    joystickControl()
# allow use as a module or standalone script
if __name__ == "__main__":
    main()
