# rover.py (Modified)
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

def serial_sender_thread():
    """
    Reads the latest values from the buffer and sends them to the STM32 
    at a fixed frequency to prevent Serial Overrun Errors.
    """
    send_frequency = 0.02  # 50Hz (50 times per second)
    print(f"Serial Sender Thread started at {1/send_frequency}Hz")
    
    while running:
        omega = None
        theta = None
        
        # Get the most recent values from the buffer
        with rover_buffer.lock:
            if rover_buffer.Omega:
                omega = rover_buffer.Omega[-1]
                theta = rover_buffer.Theta[-1]
        
        # Send to hardware if we have data
        if omega is not None and theta is not None:
            try:
                stm32.send_motors_and_pwm(omega, theta)
            except Exception as e:
                print(f"Serial transmission error: {e}")
        
        time.sleep(send_frequency)

def main():
    global running
    # Initialize Zenoh
    conf = zenoh.Config()
    session = zenoh.open(conf)
    
    to_rover_key = 'base/to/rover'
    from_rover_key = 'rover/to/base'
    
    # Subscriber for incoming commands
    sub = session.declare_subscriber(to_rover_key, message_callback)
    # Publisher for telemetry
    pub = session.declare_publisher(from_rover_key)

    # Start the dedicated serial sender thread
    sender_thread = threading.Thread(target=serial_sender_thread, daemon=True)
    sender_thread.start()

    print(f"Rover Online. Decoupled Zenoh -> Serial (50Hz).")
    print(f"Listening on: {to_rover_key}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping Rover...")
    finally:
        running = False
        sender_thread.join(timeout=1.0)
        stm32.disarm() # Send safety stop signal
        sub.undeclare()
        pub.undeclare()
        session.close()
        stm32.close()

if __name__ == "__main__":
    main()
