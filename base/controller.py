import pygame
import time
import evdev
import os
from evdev import InputDevice, ecodes
from base_databuffer import DataBuffer
import controller_config
from expo import DualExpoMapper

# --- CONFIGURATION ---
USE_PS4 = False  # Set to False to use the Steering Wheel + Gear Selector
# ---------------------

class BaseController:
    def __init__(self, databuffer: DataBuffer, frequency: float = 20.0):
        self.buffer = databuffer
        self.loop_period = 1.0 / frequency
        self.look_theta = 0.0
        self.expo = DualExpoMapper(steering_expo=0.3, throttle_expo=0.5)
        
        # Shared Target Ranges
        self.STEER_MIN, self.STEER_MAX = 120, 880
        self.NEUTRAL_RPM = 1048
        self.MAX_LOOK_RATE = 180.0
        
        self._init_pygame_headless()

    def _init_pygame_headless(self):
        """Initializes pygame with a dummy video driver to prevent errors."""
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.display.init()
        pygame.joystick.init()

    def _map(self, val, in_min, in_max, out_min, out_max):
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def get_gear_limits(self, gear, direction):
        """Returns (min_rpm, max_rpm) based on gear and direction logic."""
        if gear == 0:
            return self.NEUTRAL_RPM, self.NEUTRAL_RPM
        
        if direction == "FORWARD":
            gear_map = {
                1: (1049, 1150),
                2: (1049, 1300),
                3: (1049, 1450),
                4: (1049, 1600),
                5: (1600, 1750)
            }
        else: # BACKWARD
            gear_map = {
                1: (48, 250),
                2: (48, 400),
                3: (48, 550),
                4: (48, 700),
                5: (48, 850)
            }
        return gear_map.get(gear, (self.NEUTRAL_RPM, self.NEUTRAL_RPM))

class PS4Controller(BaseController):
    def __init__(self, databuffer, frequency=20.0):
        super().__init__(databuffer, frequency)
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No PS4 controller detected!")
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.current_gear = 2 

    def run(self):
        try:
            while True:
                pygame.event.pump()
                raw_steer = self.joy.get_axis(0)
                expo_steer = self.expo.apply_steering(raw_steer)
                steering_angle = int(self._map(expo_steer, -1.0, 1.0, self.STEER_MIN, self.STEER_MAX))

                raw_l2 = self.joy.get_axis(2) 
                raw_r2 = self.joy.get_axis(5) 
                
                if raw_r2 > -0.9:
                    g_min, g_max = self.get_gear_limits(self.current_gear, "FORWARD")
                    rpm = int(self._map(self.expo.apply_throttle(raw_r2), -1.0, 1.0, g_min, g_max))
                elif raw_l2 > -0.9:
                    g_min, g_max = self.get_gear_limits(self.current_gear, "BACKWARD")
                    rpm = int(self._map(self.expo.apply_throttle(raw_l2), 1.0, -1.0, g_max, g_min))
                else:
                    rpm = self.NEUTRAL_RPM

                self.update_buffer(rpm, steering_angle, self.look_theta)
                time.sleep(self.loop_period)
        except KeyboardInterrupt: pass

    def update_buffer(self, rpm, steer, look):
        with self.buffer.lock:
            self.buffer.Omega.append(float(rpm))
            self.buffer.Theta.append(float(steer))
            self.buffer.look_theta.append(float(look))

class WheelController(BaseController):
    def __init__(self, databuffer, frequency=20.0, device_path="/dev/input/event7"):
        super().__init__(databuffer, frequency)
        
        # 1. Initialize Pygame for Buttons
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected by Pygame! Check USB connection.")
        
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        print(f"[Pygame] Linked to: {self.joy.get_name()}") # Verify this is the wheel

        # 2. Initialize Evdev for Axes
        try:
            self.wheel_dev = InputDevice(device_path)
            print(f"[Evdev] Linked to: {self.wheel_dev.name} at {device_path}")
        except PermissionError:
            print("\n!!! PERMISSION DENIED: Try running with 'sudo python3 controller.py' !!!\n")
            raise
        except Exception as e:
            print(f"Could not open {device_path}: {e}")
            print("Available devices:")
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            for d in devices: print(f"  {d.path} - {d.name}")
            raise

        # Logic States
        self.gear = 0
        self.direction = "FORWARD"
        self.prev_buttons = [0] * self.joy.get_numbuttons()
        self.raw_steer = 32768
        self.raw_throttle = 0

    def poll_wheel(self):
        """Non-blocking read of evdev events."""
        try:
            # We must exhaust the event queue each loop
            while True:
                event = self.wheel_dev.read_one()
                if event is None: break 
                
                if event.type == ecodes.EV_ABS:
                    # Map axes based on your steeringwheelread.py logic
                    if event.code == 2:   # ABS_Z: Steering
                        self.raw_steer = event.value
                    elif event.code == 5: # ABS_RZ: Throttle
                        self.raw_throttle = event.value
        except Exception as e:
            print(f"Error reading wheel axes: {e}")

    def update_gears(self):
        """Edge-triggered gear logic from gearselector.py"""
        pygame.event.pump()
        buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        
        # Button 5: Gear Up, Button 4: Gear Down
        if buttons[5] and not self.prev_buttons[5]:
            self.gear = min(5, self.gear + 1)
            print(f"\nGear Shifted Up: {self.gear}")
            
        if buttons[4] and not self.prev_buttons[4]:
            self.gear = max(0, self.gear - 1)
            print(f"\nGear Shifted Down: {self.gear}")

        # Button 0: Forward, Button 1: Backward
        if buttons[0] and not self.prev_buttons[0]:
            self.direction = "FORWARD"
            print("\nDirection: FORWARD")
            
        if buttons[1] and not self.prev_buttons[1]:
            self.direction = "BACKWARD"
            print("\nDirection: BACKWARD")
        
        self.prev_buttons = buttons.copy()

    def run(self):
        try:
            print(f"Wheel Controller Started. Device: {self.wheel_dev.name}")
            while True:
                self.poll_wheel()
                self.update_gears()

                # 1. Map Steering (0-65535 -> 120-880)
                normalized_steer = (self.raw_steer / 32767.5) - 1.0
                expo_steer = self.expo.apply_steering(normalized_steer)
                steering_angle = int(self._map(expo_steer, -1.0, 1.0, self.STEER_MIN, self.STEER_MAX))

                # 2. Map RPM based on Gear/Direction
                g_min, g_max = self.get_gear_limits(self.gear, self.direction)
                
                # Normalize throttle (assumed 0-1023)
                throttle_norm = min(self.raw_throttle / 1023.0, 1.0)
                
                if self.direction == "FORWARD":
                    rpm = int(self._map(throttle_norm, 0, 1.0, g_min, g_max))
                else:
                    # Reverse: Throttle 0 is neutral, Throttle 1.0 is max reverse (e.g., 48)
                    rpm = int(self._map(throttle_norm, 0, 1.0, self.NEUTRAL_RPM, g_min))

                # 3. Write to Buffer (now includes look_theta)
                with self.buffer.lock:
                    self.buffer.Omega.append(float(rpm))
                    self.buffer.Theta.append(float(steering_angle))
                    self.buffer.look_theta.append(float(self.look_theta))
                
                time.sleep(self.loop_period)
        except KeyboardInterrupt:
            print("\nShutting down controller...")

if __name__ == "__main__":
    from base_databuffer import DataBuffer
    db = DataBuffer()
    
    # Selection based on top-level flag
    if USE_PS4:
        ctrl = PS4Controller(db)
    else:
        ctrl = WheelController(db)
    ctrl.run()