# controller.py
import pygame
import time
from base_databuffer import DataBuffer
import controller_config

class Controller:
    def __init__(self, databuffer: DataBuffer, frequency: float = 20.0):
        self.buffer = databuffer
        self.loop_period = 1.0 / frequency
        self.look_theta = 0.0

        # Target Ranges for the new mapping
        self.STEER_MIN, self.STEER_MAX = 120, 880
        self.RPM_L2_MIN, self.RPM_L2_MAX = 48, 1047
        self.RPM_R2_MIN, self.RPM_R2_MAX = 1049, 2047
        self.NEUTRAL_RPM = 1048
        
        self.MAX_LOOK_RATE = 180.0
        self.STICK_DEADZONE = 0.12

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected!")

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        # Axis mapping profiles
        if controller_config.IS_PS4:
            self.AX_STEER = 0
            self.AX_L2    = 2
            self.AX_R2    = 5
            self.AX_LOOK  = 3
        else: # Xbox
            self.AX_STEER = 0
            self.AX_L2    = 2 # Note: Xbox LT/RT behavior can vary by OS/driver
            self.AX_R2    = 5
            self.AX_LOOK  = 4

    def _map(self, val, in_min, in_max, out_min, out_max):
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def run(self):
        try:
            while True:
                pygame.event.pump()

                # 1. Steering Angle (120 to 880)
                raw_steer = self.joy.get_axis(self.AX_STEER)
                steering_angle = int(self._map(raw_steer, -1.0, 1.0, self.STEER_MIN, self.STEER_MAX))

                # 2. RPM (Triggers)
                # Pygame triggers rest at -1.0 and go to 1.0
                raw_l2 = self.joy.get_axis(self.AX_L2)
                raw_r2 = self.joy.get_axis(self.AX_R2)
                
                l2_active = raw_l2 > -0.9
                r2_active = raw_r2 > -0.9

                if l2_active and r2_active:
                    rpm = self.NEUTRAL_RPM
                elif l2_active:
                    rpm = int(self._map(raw_l2, -1.0, 1.0, self.RPM_L2_MIN, self.RPM_L2_MAX))
                elif r2_active:
                    rpm = int(self._map(raw_r2, -1.0, 1.0, self.RPM_R2_MIN, self.RPM_R2_MAX))
                else:
                    rpm = self.NEUTRAL_RPM

                # 3. Look Theta (Keep existing incremental logic)
                raw_look = self.joy.get_axis(self.AX_LOOK)
                if abs(raw_look) < self.STICK_DEADZONE: raw_look = 0
                self.look_theta = (self.look_theta + (raw_look * self.MAX_LOOK_RATE * self.loop_period)) % 360.0

                # Write to buffer
                with self.buffer.lock:
                    # We map Omega to your RPM range and Theta to steering_angle
                    self.buffer.Omega.append(float(rpm))
                    self.buffer.Theta.append(float(steering_angle))
                    self.buffer.look_theta.append(self.look_theta)

                time.sleep(self.loop_period)
        except KeyboardInterrupt:
            pass
        finally:
            self.joy.quit()
