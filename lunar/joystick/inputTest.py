import pygame

xboxController = pygame.joystick.Joystick(0)
xboxController.init()
print(xboxController.get_name())
print("Axes", (xboxController.get_numaxes()))
print("Balls", xboxController.get_numballs())
print("Buttons", xboxController.get_numbuttons())
print("Hats", xboxController.get_numhats())
if xboxController.get_init() is True:
	print("Initialized properly")

while not found:
    trial += 1
    print(trial)
    pygame.event.pump()
    for a in range(0, xboxController.get_numaxes()):
        if round(xboxController.get_axis(a) * 100) != 0:
            print(xboxController.get_axis(a))
    for b in range(0, xboxController.get_numballs()):
        if xboxController.get_ball(b) != 0: 
            print(xboxController.get_ball(b))
    for c in range(0, xboxController.get_numbuttons()):
        if xboxController.get_button(c) is False: 
            print(xboxController.get_button(button(c)))
    for d in range(0, xboxController.get_numhats()):
        if xboxController.get_hat(d) != (0, 0): 
            print(xboxController.get_hat(d))
