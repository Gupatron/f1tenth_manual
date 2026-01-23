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
BRAKE_MODE = "NEUTRAL"  # Options: "ACTIVE", "NEUTRAL", "PROGRESSIVE"
# ---------------------

class BaseController:
    def __init__(self, databuffer: DataBuffer, frequency: float = 20.0):
        self.buffer = databuffer
        self.loop_period = 1.0 / frequency
        self.look_theta = 0.0
        self.expo = DualExpoMapper(steering_expo=0.3, throttle_expo=0.5)
        
        # Shared Target Ranges
        self.STEER_MIN, self.STEER_MAX = 120, 880
        self.NEUTRAL_RPM = 0  # Changed back to 0
        self.MAX_LOOK_RATE = 180.0
        
        # Braking ranges
        self.BRAKE_MIN = 10   # Active braking range: 1-47
        self.BRAKE_MAX = 47
        
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
                5: (1049, 1750)
            }
        else: # BACKWARD
            # Range adjusted for DShot 3D mode logic
            gear_map = {
                1: (1047, 49),
                2: (49, 1047),
                3: (49, 1047),
                4: (49, 1047),
                5: (49, 1047)
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
                    rpm = int(self._map(self.expo.apply_throttle(raw_r2), -1.0, 1.0, self.NEUTRAL_RPM + 1, g_max))
                elif raw_l2 > -0.9:
                    # PS4 Reverse: Map L2 trigger to 1047 -> 49
                    rpm = int(self._map(self.expo.apply_throttle(raw_l2), -1.0, 1.0, self.NEUTRAL_RPM - 1, 49))
                else:
                    rpm = self.NEUTRAL_RPM

                self.update_buffer(rpm, steering_angle, self.look_theta)
                time.sleep(self.loop_period)
        except KeyboardInterrupt: 
            pass

    def update_buffer(self, rpm, steer, look):
        with self.buffer.lock:
            self.buffer.Omega.append(float(rpm))
            self.buffer.Theta.append(float(steer))
            self.buffer.look_theta.append(float(look))


class WheelController(BaseController):
    def __init__(self, databuffer, frequency=20.0, device_path=None):
        super().__init__(databuffer, frequency)
        
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected by Pygame!")
        
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        if device_path is None:
            device_path = self._find_wheel_device()
        
        self.wheel_dev = InputDevice(device_path)

        # Logic States
        self.gear = 0
        self.direction = "FORWARD"
        self.prev_buttons = [0] * self.joy.get_numbuttons()
        self.raw_steer = 32768
        self.raw_throttle = 0
        self.raw_brake = 0  # New: Brake pedal input
        self.reverse_pedal_pressed = False
        self.last_rpm = self.NEUTRAL_RPM  # Track last RPM for progressive braking

    def _find_wheel_device(self):
        """Auto-detect the steering wheel device."""
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            caps = device.capabilities(verbose=False)
            if ecodes.EV_ABS in caps:
                abs_events = caps[ecodes.EV_ABS]
                if any(event[0] == ecodes.ABS_Z for event in abs_events):
                    return device.path
        raise RuntimeError("Could not find steering wheel device.")

    def poll_wheel(self):
        try:
            while True:
                event = self.wheel_dev.read_one()
                if event is None: 
                    break 
                if event.type == ecodes.EV_ABS:
                    if event.code == 2:   # ABS_Z: Steering
                        self.raw_steer = event.value
                    elif event.code == 5: # ABS_RZ: Throttle
                        self.raw_throttle = event.value
                    elif event.code == 4: # ABS_Y: Brake (adjust code if needed)
                        self.raw_brake = event.value
        except Exception as e:
            print(f"Error reading wheel axes: {e}")

    def update_gears(self):
        """Edge-triggered gear logic with direction-change safety."""
        pygame.event.pump()
        buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        
        # Gear Shift
        if buttons[5] and not self.prev_buttons[5]:
            self.gear = min(5, self.gear + 1)
        if buttons[4] and not self.prev_buttons[4]:
            self.gear = max(0, self.gear - 1)

        # Direction Selection
        new_direction = self.direction
        if buttons[0] and not self.prev_buttons[0]:
            new_direction = "FORWARD"
        if buttons[1] and not self.prev_buttons[1]:
            new_direction = "BACKWARD"

        # Handle direction change
        if new_direction != self.direction:
            self.direction = new_direction
            if self.direction == "BACKWARD":
                # Reset reverse pedal flag when entering reverse
                self.reverse_pedal_pressed = False
                # Send 0 signal on entering reverse
                with self.buffer.lock:
                    self.buffer.Omega.append(0.0)
            else:
                # Send neutral when switching to forward
                with self.buffer.lock:
                    self.buffer.Omega.append(float(self.NEUTRAL_RPM))

        self.prev_buttons = buttons.copy()

    def calculate_brake_rpm(self, brake_norm):
        """Calculate RPM based on brake input and selected brake mode."""
        if BRAKE_MODE == "ACTIVE":
            # Active braking: 1-47 range (ESC must support this)
            return int(self._map(brake_norm, 0, 1.0, self.BRAKE_MIN, self.BRAKE_MAX))
        
        elif BRAKE_MODE == "NEUTRAL":
            # Simple: Just return 0 to stop
            return 0
        
        elif BRAKE_MODE == "PROGRESSIVE":
            # Progressive: Apply opposite throttle based on brake pressure
            if self.direction == "FORWARD":
                # Apply reverse throttle to brake
                return int(self._map(brake_norm, 0, 1.0, 0, 200))
            else:
                # Apply forward throttle to brake
                return int(self._map(brake_norm, 0, 1.0, 0, 1200))
        
        return 0

    def run(self):
        try:
            print(f"Wheel Controller Started. Device: {self.wheel_dev.name}")
            print(f"Brake Mode: {BRAKE_MODE}")
            while True:
                self.poll_wheel()
                self.update_gears()

                # 1. Map Steering
                normalized_steer = (self.raw_steer / 32767.5) - 1.0
                expo_steer = self.expo.apply_steering(normalized_steer)
                steering_angle = int(self._map(expo_steer, -1.0, 1.0, self.STEER_MIN, self.STEER_MAX))

                # 2. Check Brake Input
                brake_norm = min(self.raw_brake / 1023.0, 1.0)
                brake_active = brake_norm > 0.05  # Brake threshold

                # 3. Map RPM
                if brake_active:
                    # Brake is pressed - override throttle
                    rpm = self.calculate_brake_rpm(brake_norm)
                else:
                    # Normal throttle control
                    g_min, g_max = self.get_gear_limits(self.gear, self.direction)
                    throttle_norm = min(self.raw_throttle / 1023.0, 1.0)
                    
                    if self.gear == 0 or throttle_norm < 0.02:
                        rpm = 0  # Send 0 when no throttle
                    else:
                        if self.direction == "FORWARD":
                            # Forward: 1049 up to Gear Max
                            rpm = int(self._map(throttle_norm, 0, 1.0, 1049, g_max))
                        else:
                            # Reverse: Send 0 until pedal is pressed, then map 49-1047
                            if not self.reverse_pedal_pressed:
                                if throttle_norm >= 0.02:
                                    self.reverse_pedal_pressed = True
                                    rpm = int(self._map(throttle_norm, 0, 1.0, 49, 1047))
                                else:
                                    rpm = 0
                            else:
                                if throttle_norm >= 0.02:
                                    rpm = int(self._map(throttle_norm, 0, 1.0, 49, 1047))
                                else:
                                    rpm = 0  # Send 0 when pedal released

                # Store last RPM for reference
                self.last_rpm = rpm

                # 4. Write to Buffer
                with self.buffer.lock:
                    self.buffer.Omega.append(float(rpm))
                    self.buffer.Theta.append(float(steering_angle))
                    self.buffer.look_theta.append(float(self.look_theta))
                
                time.sleep(self.loop_period)
        except KeyboardInterrupt:
            print("\nShutting down controller...")