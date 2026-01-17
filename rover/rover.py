# rover.py
import zenoh
import time
import struct
from rover_databuffer import DataBuffer
from dshot_serial import DShotSerial

# Global DataBuffer
rover_buffer = DataBuffer()
# Initialize Serial Driver
stm32 = DShotSerial(port='/dev/ttyUSB0', baud=921600)

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
    Triggered whenever a packet arrives from the Base.
    Immediately forwards data to the STM32 for lowest latency.
    """
    try:
        msg = Message.from_binary(sample.payload.to_bytes())
        
        # 1. Update Buffer (for logging/other modules)
        with rover_buffer.lock:
            rover_buffer.Omega.append(msg.omega)
            rover_buffer.Theta.append(msg.theta)
            rover_buffer.look_theta.append(msg.look_theta)

        # 2. Forward to STM32 Hardware
        # Omega = DShot Motor values (48-2047)
        # Theta = Steering PWM values (120-880)
        stm32.send_motors_and_pwm(msg.omega, msg.theta)
        
    except Exception as e:
        print(f"Error processing incoming Zenoh message: {e}")

def main():
    # Initialize Zenoh
    conf = zenoh.Config()
    session = zenoh.open(conf)
    
    to_rover_key = 'base/to/rover'
    from_rover_key = 'rover/to/base'
    
    # Subscriber for incoming commands
    sub = session.declare_subscriber(to_rover_key, message_callback)
    # Publisher for telemetry (if needed)
    pub = session.declare_publisher(from_rover_key)

    print(f"Rover Online. Mapping Zenoh -> DShot Serial.")
    print(f"Listening on: {to_rover_key}")

    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping Rover...")
    finally:
        stm32.disarm()
        sub.undeclare()
        pub.undeclare()
        session.close()
        stm32.close()

if __name__ == "__main__":
    main()