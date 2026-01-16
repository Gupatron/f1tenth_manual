import pygame
import os
import time

# Initialize Pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()

def clear_screen():
    # Clears the terminal screen for a clean "dashboard" look
    os.system('cls' if os.name == 'nt' else 'clear')

try:
    while True:
        pygame.event.pump() # Internal pygame magic to keep the data fresh
        clear_screen()
        
        print(f"--- Monitoring: {controller.get_name()} ---")
        
        # 1. Print all Axes (Sticks and Triggers)
        print("\nAXES:")
        for i in range(controller.get_numaxes()):
            axis_val = controller.get_axis(i)
            # Standard PS4 Mapping: 0:LX, 1:LY, 2:RX, 3:RY, 4:L2, 5:R2
            print(f" Axis {i}: {axis_val:>8.4f}")

        # 2. Print all Buttons
        print("\nBUTTONS:")
        buttons = []
        for i in range(controller.get_numbuttons()):
            button_val = controller.get_button(i)
            buttons.append(f"B{i}: {button_val}")
        
        # Print buttons in rows of 4 for readability
        for i in range(0, len(buttons), 4):
            print("  ".join(buttons[i:i+4]))

        # 3. Print D-Pad (Hats)
        print("\nD-PAD / HATS:")
        for i in range(controller.get_numhats()):
            print(f" Hat {i}: {controller.get_hat(i)}")

        print("\n(Press Ctrl+C to stop)")
        time.sleep(0.05) # Update ~20 times per second

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    pygame.quit()