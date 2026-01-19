# rover.py (Fixed with Working Serial Pattern)
import zenoh
import time
import struct
import threading
from rover_databuffer import DataBuffer
from dshot_serial import DShotSerial

# Global DataBuffer and Serial Driver
rover_buffer = DataBuffer()
# Initialize Serial Driver at the specified baud rate
stm32 = DShotSerial(port='/dev/ttyUSB0', baud=921600)

# Global flag for thread management
running = True
armed = False

class Message:
    def __init__(self, omega: float, theta: float, look_theta: float):
        self.omega = omega
        self.theta = theta
        self.look_theta = look_theta

    @classmethod
    def from_binary(cls, data: bytes) -> 'Message':
        # Expects 24 bytes (3 doubles)
        return cls(*struct.unpack('ddd', data))

def message_callback(sample):
    """
    Triggered when a packet arrives from the Base.
    Updates the buffer ONLY to avoid blocking the Zenoh thread with Serial I/O.
    """
    try:
        msg = Message.from_binary(sample.payload.to_bytes())
        
        # Update Buffer with latest commands
        with rover_buffer.lock:
            rover_buffer.Omega.append(msg.omega)
            rover_buffer.Theta.append(msg.theta)
            rover_buffer.look_theta.append(msg.look_theta)
            
    except Exception as e:
        print(f"Error decoding Zenoh message: {e}")

def arm_esc_sequence():
    """
    Sends the proper arming sequence to the ESC.
    Based on working dshot_communication.py pattern:
    - Send 0 throttle (not command 47) for 5 seconds
    - This allows ESC to recognize signal and arm
    """
    global armed
    print("Starting ESC arming sequence (5 seconds at zero throttle)...")
    
    # Send zero throttle with neutral steering for 5 seconds
    # This matches the working script's disarm/idle pattern
    start_time = time.time()
    while time.time() - start_time < 5:
        stm32.send_motors_and_pwm(0, 523)  # 0 = zero throttle, 523 = center steering
        time.sleep(0.02)  # 50Hz
    
    armed = True
    print("ESC arming sequence complete. ESC should be armed now.")

def serial_sender_thread():
    """
    Reads the latest values from the buffer and sends them to the STM32 
    at a fixed frequency to prevent Serial Overrun Errors.
    
    CRITICAL: Sends continuous signals even when no commands are received
    to maintain ESC arming state.
    """
    send_frequency = 0.02  # 50Hz (20ms between sends, matches working script)
    print(f"Serial Sender Thread started at {1/send_frequency}Hz")
    
    # Default safe values (zero throttle, center steering)
    # These are THROTTLE values (0-2047), NOT DShot commands
    current_omega = 0    # Zero throttle (idle)
    current_theta = 523  # Center steering (neutral)
    
    while running:
        # Get the most recent values from the buffer if available
        with rover_buffer.lock:
            if rover_buffer.Omega:
                current_omega = rover_buffer.Omega[-1]
            if rover_buffer.Theta:
                current_theta = rover_buffer.Theta[-1]
        
        # ALWAYS send to hardware to maintain signal continuity
        # This is critical for ESC operation
        try:
            stm32.send_motors_and_pwm(current_omega, current_theta)
        except Exception as e:
            print(f"Serial transmission error: {e}")
        
        time.sleep(send_frequency)

def main():
    global running
    
    # Small delay to ensure port is ready after any previous script
    print("Waiting for serial port to be ready...")
    time.sleep(0.5)
    
    # Check if serial connection was successful
    if not stm32.ser:
        print("ERROR: Failed to connect to STM32. Exiting.")
        return
    
    # Initialize Zenoh
    conf = zenoh.Config()
    session = zenoh.open(conf)
    
    to_rover_key = 'base/to/rover'
    from_rover_key = 'rover/to/base'
    
    # Subscriber for incoming commands
    sub = session.declare_subscriber(to_rover_key, message_callback)
    # Publisher for telemetry
    pub = session.declare_publisher(from_rover_key)

    # Start the dedicated serial sender thread FIRST
    # This ensures continuous signal output before arming
    sender_thread = threading.Thread(target=serial_sender_thread, daemon=True)
    sender_thread.start()
    
    # Wait for sender thread to stabilize
    time.sleep(0.1)
    
    # Now run the arming sequence (5 seconds at zero throttle)
    arm_esc_sequence()

    print(f"Rover Online. Decoupled Zenoh -> Serial (50Hz).")
    print(f"Listening on: {to_rover_key}")

    try:
        while True:
            # Optional: Add telemetry publishing here
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping Rover...")
    finally:
        running = False
        sender_thread.join(timeout=1.0)
        
        # Disarm using zero throttle (matches working script)
        print("Sending disarm signal (zero throttle)...")
        for _ in range(50):  # 1 second of zero throttle
            stm32.send_motors_and_pwm(0, 523)
            time.sleep(0.02)
        
        sub.undeclare()
        pub.undeclare()
        session.close()
        stm32.close()

if __name__ == "__main__":
    main()
