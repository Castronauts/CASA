import pygame, sys, math
from time import sleep

#---------------------Initialization Section---------------------------#
pygame.init()

# Initialize the joysticks
pygame.joystick.init()

#Get count
joystick_count = pygame.joystick.get_count()

#Check count
if joystick_count == 0:
    print("Error, No found joysticks")
    pygame.quit()
    sys.exit()
	
#Assuming only one joystick will be used meaning one controller
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

#For event loop
done = False


#----------------------- Main Program Loop ----------------------------#
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
    
    get_forward_backward = joystick.get_axis(1) #Throttle analog joystick(left)
    
    get_x_axis = joystick.get_axis(3) #Directional analog joystick(right)
    get_y_axis = -1*joystick.get_axis(4) #Remember direction is inverted so multiple -1
    
    
    #Print if going forward or backward dependent on left joystick
    if get_forward_backward < 0: #Going Forward
        forward_string = "Forward Power: {:>.2f}%".format(-100*get_forward_backward)
        backward_string = "Backward Power: {:>.2f}%".format(0)
    else: #Going Backward
        forward_string = "Forward Power: {:>.2f}%".format(0)
        backward_string = "Backward Power: {:>.2f}%".format(100*get_forward_backward)
	
    print(forward_string + '  ' + backward_string + '\n') #Prints power of joystick
    
    #Now determine 360 degree direction based on right joystick
    radians = math.atan2(get_y_axis, get_x_axis)
    degree = math.degrees(radians)
    if degree < 0:
        degree = degree % 360
	
    print("Direction: {:>.2f}".format(degree) + '\n')
    
    sleep(.1)
    
#Quit everything
pygame.quit()
sys.exit()

"""Found the correct axes to use with the example code posted"""
"""The axes were Axis0/Axis1 for left and Axis3/Axis4 for right"""
