import evdev
from evdev import InputDevice, ecodes

def find_wheel_device():
    """Find the steering wheel device."""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    print("Available devices:")
    for i, device in enumerate(devices):
        print(f"{i}: {device.name} - {device.path}")
    
    # Try to auto-detect wheel
    for device in devices:
        caps = device.capabilities(verbose=False)
        if ecodes.EV_ABS in caps:
            abs_events = caps[ecodes.EV_ABS]
            if any(event[0] == ecodes.ABS_Z for event in abs_events):
                print(f"\nAuto-detected wheel: {device.name}")
                return device.path
    
    return None

def monitor_events(device_path):
    """Monitor and display all events from the device."""
    device = InputDevice(device_path)
    print(f"\nMonitoring: {device.name}")
    print("=" * 60)
    print("Press your brake pedal, throttle, and steering to see events")
    print("Press Ctrl+C to exit")
    print("=" * 60)
    
    # Dictionary to store event names
    event_names = {
        0: "ABS_X",
        1: "ABS_Y", 
        2: "ABS_Z",
        3: "ABS_RX",
        4: "ABS_RY",
        5: "ABS_RZ",
        6: "ABS_THROTTLE",
        9: "ABS_BRAKE",
        16: "ABS_HAT0X",
        17: "ABS_HAT0Y"
    }
    
    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_ABS:
                event_name = event_names.get(event.code, f"UNKNOWN_{event.code}")
                print(f"Type: ABS  |  Code: {event.code:2d} ({event_name:15s})  |  Value: {event.value:5d}")
            elif event.type == ecodes.EV_KEY:
                print(f"Type: KEY  |  Code: {event.code:2d}  |  Value: {event.value}")
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")

if __name__ == "__main__":
    device_path = find_wheel_device()
    
    if device_path is None:
        print("\nCould not auto-detect wheel. Please enter device path manually:")
        device_path = input("Device path (e.g., /dev/input/event5): ").strip()
    
    monitor_events(device_path)