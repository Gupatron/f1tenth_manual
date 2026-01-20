import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller detected")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Controller connected: {joystick.get_name()}")

# Constants
MAX_GEAR = 5

# State variables
gear = 0
direction = "FORWARD"  # default state

# Track previous button states for edge detection
prev_buttons = [0] * joystick.get_numbuttons()

clock = pygame.time.Clock()
running = True

while running:
    pygame.event.pump()

    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    # ---------- Gear Logic ----------
    # Button 5 → gear up (edge-triggered)
    if buttons[5] and not prev_buttons[5]:
        gear = min(MAX_GEAR, gear + 1)

    # Button 4 → gear down (edge-triggered)
    if buttons[4] and not prev_buttons[4]:
        gear = max(0, gear - 1)

    # ---------- Direction Logic (latched) ----------
    # Button 0 → FORWARD
    if buttons[0] and not prev_buttons[0]:
        direction = "FORWARD"

    # Button 1 → BACKWARD
    if buttons[1] and not prev_buttons[1]:
        direction = "BACKWARD"

    # Print state on change
    if buttons != prev_buttons:
        print(f"Gear: {gear}, Direction: {direction}")

    prev_buttons = buttons.copy()
    clock.tick(60)

pygame.quit()
