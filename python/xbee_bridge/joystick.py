## joystick
#

import pygame

class joystick:
    def __init__(self):
        pygame.init()
        ## list of joysticks
        self.joysticks = []
        ## rudder COMMAND
        self.rudderCmd = 0.0
        ## throttle command
        self.throttleCmd = 0.0
        # initialize joysticks
        for i in range(0, pygame.joystick.get_count()):
            # create an Joystick object in our list
            self.joysticks.append(pygame.joystick.Joystick(i))
            # initialize them all (-1 means loop forever)
            self.joysticks[-1].init()
            # print a statement telling what the name of the controller is
            print "Detected joystick '",self.joysticks[-1].get_name(),"'"
    ## read the joystick axes and set the rudder/throttle values as appropriate
    def read(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                axis_value = event.value
                if event.axis==0:
                    self.rudderCmd = axis_value
                if event.axis==3:
                    self.throttleCmd = -axis_value
                    if self.throttleCmd <= 0.1:
                        self.throttleCmd = 0.0
                    elif self.throttleCmd >= 1.0:
                        self.throttleCmd = 1.0

    def __del__(self):
        pygame.quit()

def main():
    joy = joystick()
    while (True):
        try:
            joy.read()
            print("%g,%g" % (joy.rudderCmd,joy.throttleCmd))
        except KeyboardInterrupt:
            print("Exit test joystick")
            break

if __name__=='__main__':
    main()
