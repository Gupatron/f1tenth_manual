import pygame
import os
import time
import config

def map_range(x, in_min, in_max, out_min, out_max):
    """Linear interpolation to map a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Initialize Pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()

try:
    while True:
        pygame.event.pump()
        os.system('cls' if os.name == 'nt' else 'clear')
        
        # Read Raw Values
        raw_steer = controller.get_axis(config.AXIS_STEER)
        raw_l2 = controller.get_axis(config.AXIS_L2)
        raw_r2 = controller.get_axis(config.AXIS_R2)

        # 1. Map Steering (Axis 0)
        # Input: -1 to 1 -> Output: 120 to 880
        steer_mapped = int(map_range(raw_steer, -1.0, 1.0, config.STEER_MIN, config.STEER_MAX))

        # 2. Map Triggers (Axis 4 and 5)
        # Check if both are pressed (using -0.9 as a threshold since raw is -1.0 when untouched)
        # If both are significantly pressed, output 1048
        if raw_l2 > -0.9 and raw_r2 > -0.9:
            trigger_output = config.NEUTRAL_VALUE
        elif raw_l2 > -1.0:
            # Map L2: -1 to 1 -> 48 to 1047
            trigger_output = int(map_range(raw_l2, -1.0, 1.0, config.L2_MIN, config.L2_MAX))
        elif raw_r2 > -1.0:
            # Map R2: -1 to 1 -> 1049 to 2047
            trigger_output = int(map_range(raw_r2, -1.0, 1.0, config.R2_MIN, config.R2_MAX))
        else:
            # Nothing pressed
            trigger_output = config.NEUTRAL_VALUE

        print(f"--- {controller.get_name()} Mapped Output ---")
        print(f"Steering (Axis 0): {steer_mapped}")
        print(f"Trigger Output:    {trigger_output}")
        print("\n(Press Ctrl+C to stop)")
        
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    pygame.quit()