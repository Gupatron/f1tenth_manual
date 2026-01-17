import pygame
import os
import time
import config

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()

print(f"Using {config.ACTIVE_PROFILE} profile for {controller.get_name()}")

try:
    while True:
        pygame.event.pump()
        
        # Read Raw Values using the configured MAP
        raw_steer = controller.get_axis(config.MAP["AXIS_STEER"])
        raw_l2 = controller.get_axis(config.MAP["AXIS_L2"])
        raw_r2 = controller.get_axis(config.MAP["AXIS_R2"])

        # 1. Map Steering
        steer_mapped = int(map_range(raw_steer, -1.0, 1.0, config.STEER_MIN, config.STEER_MAX))

        # 2. Map Triggers with a Deadzone
        # Pygame triggers usually rest at -1.0. We use -0.9 to ensure it's "pressed"
        l2_pressed = raw_l2 > -0.9
        r2_pressed = raw_r2 > -0.9

        if l2_pressed and r2_pressed:
            trigger_output = config.NEUTRAL_VALUE
        elif l2_pressed:
            trigger_output = int(map_range(raw_l2, -1.0, 1.0, config.L2_MIN, config.L2_MAX))
        elif r2_pressed:
            trigger_output = int(map_range(raw_r2, -1.0, 1.0, config.R2_MIN, config.R2_MAX))
        else:
            trigger_output = config.NEUTRAL_VALUE

        # Dashboard Display
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"--- {controller.get_name()} ({config.ACTIVE_PROFILE} Mode) ---")
        print(f"Steering: {steer_mapped}")
        print(f"Trigger:  {trigger_output}")
        print(f"\nRaw L2 (Axis {config.MAP['AXIS_L2']}): {raw_l2:.3f}")
        print(f"Raw R2 (Axis {config.MAP['AXIS_R2']}): {raw_r2:.3f}")
        
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    pygame.quit()
