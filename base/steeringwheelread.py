import evdev
import sys
from evdev import InputDevice, ecodes

def main():
    # Device path
    device_path = "/dev/input/event7"
    
    if len(sys.argv) > 1:
        device_path = sys.argv[1]
    
    try:
        wheel = InputDevice(device_path)
    except FileNotFoundError:
        print(f"Device {device_path} not found.")
        print("\nAvailable devices:")
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            print(f"  {device.path} - {device.name}")
        sys.exit(1)
    except PermissionError:
        print(f"\nPermission denied. Run with sudo:")
        print(f"  sudo python3 {sys.argv[0]} {device_path}")
        sys.exit(1)
    
    print(f"Reading from: {wheel.path} - {wheel.name}")
    print("\nMapping:")
    print("  Steering (ABS_Z, code 2): 0-65535 → -1.0 to 1.0")
    print("  Brake (ABS_X, code 0): Binary/pressure value")
    print("  Throttle (ABS_RZ, code 5): Binary/pressure value")
    print("\nPress Ctrl+C to exit\n")
    
    # Store current values
    steering_value = 0.0
    brake_value = 0.0
    throttle_value = 0.0
    
    try:
        # Read events in a loop
        for event in wheel.read_loop():
            # Only process absolute axis events
            if event.type == ecodes.EV_ABS:
                axis_code = event.code
                
                # ABS_Z (code 2) = Steering
                if axis_code == 2:  # ABS_Z
                    # Map 0-65535 to -1.0 to 1.0
                    steering_value = (event.value / 32767.5) - 1.0
                
                # ABS_X (code 0) = Brake
                elif axis_code == 0:  # ABS_X
                    brake_value = event.value
                
                # ABS_RZ (code 5) = Throttle
                elif axis_code == 5:  # ABS_RZ
                    throttle_value = event.value
                
                # Create visual bars
                bar_length = 40
                
                # Steering bar (-1 to 1)
                steer_pos = int((steering_value + 1) / 2 * bar_length)
                steer_pos = max(0, min(bar_length, steer_pos))
                steer_bar = "█" * steer_pos + "░" * (bar_length - steer_pos)
                
                # Brake bar (normalize 0-1023 to 0.0-1.0)
                brake_normalized = min(brake_value / 1023.0, 1.0)
                brake_pos = int(brake_normalized * bar_length)
                brake_pos = max(0, min(bar_length, brake_pos))
                brake_bar = "█" * brake_pos + "░" * (bar_length - brake_pos)
                
                # Throttle bar (normalize 0-1023 to 0.0-1.0)
                throttle_normalized = min(throttle_value / 1023.0, 1.0)
                throttle_pos = int(throttle_normalized * bar_length)
                throttle_pos = max(0, min(bar_length, throttle_pos))
                throttle_bar = "█" * throttle_pos + "░" * (bar_length - throttle_pos)
                
                # Clear and display
                print("\033[2K\033[1G", end="")
                print(f"Steering: [{steer_bar}] {steering_value:+.3f}")
                print(f"Brake:    [{brake_bar}] {brake_value:.1f}")
                print(f"Throttle: [{throttle_bar}] {throttle_value:.1f}")
                print("\033[3A", end="", flush=True)
    
    except KeyboardInterrupt:
        print("\n\n\nExiting...")

if __name__ == "__main__":
    main()