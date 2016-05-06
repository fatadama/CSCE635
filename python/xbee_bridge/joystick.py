## joystick
#

import pygame

class joystick:
    ## Constructor
    #
    # @param[in] dw_interface: boolean, set to true to enable BACKSPACE and SPACEBAR keys to set synthetic waypoints and toggle control mode, respectively
    def __init__(self,dw_interface=False):
        pygame.init()
        ## list of joysticks
        self.joysticks = []
        ## rudder COMMAND
        self.rudderCmd = 0.0
        ## throttle command
        self.throttleCmd = 0.0
        ## flag to use keyboard
        self.useKeyboard = False
        ## flag: is the debugging interface on?
        self.debug_interface=dw_interface
        ## control_mode: boolean,set to True to toggle the control mode between two options
        self.control_mode = False
        ## new_waypoint: boolean, set to True when you want the main process to generate a new synthetic waypoint.
        self.new_waypoint = False
        # initialize joysticks
        num = pygame.joystick.get_count()
        if num < 1:
            # set up keyboard
            self.useKeyboard = True
            print("No joysticks present, use keyboard WASD to control")
            screen = pygame.display.set_mode((320, 240))
            pygame.display.set_caption("Active window for keyboard control")
            background = pygame.Surface(screen.get_size())
            background = background.convert()
            background.fill((255, 255, 255))
        else:
            # turn on the screen if we're debugging on joystick mode
            if self.debug_interface:
                screen = pygame.display.set_mode((320, 240))
                pygame.display.set_caption("Active window for keyboard control")
                background = pygame.Surface(screen.get_size())
                background = background.convert()
                background.fill((255, 255, 255))
            for i in range(num):
                # create an Joystick object in our list
                self.joysticks.append(pygame.joystick.Joystick(i))
                # initialize them all (-1 means loop forever)
                self.joysticks[-1].init()
                # print a statement telling what the name of the controller is
                print "Detected joystick '",self.joysticks[-1].get_name(),"'"
    ## read the joystick axes and set the rudder/throttle values as appropriate
    def read(self):
        # read joystick
        for event in pygame.event.get():
            if self.useKeyboard==False:
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
            else:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        self.throttleCmd = 0.6
                    if event.key == pygame.K_a:
                        self.rudderCmd = -0.5
                    if event.key == pygame.K_d:
                        self.rudderCmd = 0.5
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_w:
                        self.throttleCmd = 0.0
                    if event.key == pygame.K_a:
                        self.rudderCmd = 0.0
                    if event.key == pygame.K_d:
                        self.rudderCmd = 0.0
            # if using the debugging interface, check for keydown events to toggle things
            if self.debug_interface:
                if event.type == pygame.KEYDOWN:
                    # spacebar: toggle control mode
                    if event.key == pygame.K_SPACE:
                        self.control_mode = True
                    # backspace: create a new synthetic waypoint
                    if event.key == pygame.K_BACKSPACE:
                        self.new_waypoint = True


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
