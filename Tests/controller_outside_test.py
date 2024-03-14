
import pygame
import sys

# Initialize Pygame
pygame.init()

# Initialize joystick subsystem
pygame.joystick.init()

# Check how many joysticks are connected
joystick_count = pygame.joystick.get_count()
print("Number of joysticks:", joystick_count)

# Get the first joystick
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Main game loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if event.type == pygame.JOYAXISMOTION:
                # Handle joystick axis motion
                print("Axis:", event.axis, "Value:", event.value)

            if event.type == pygame.JOYBUTTONDOWN:
                # Handle joystick button press
                print("Button pressed:", event.button)
