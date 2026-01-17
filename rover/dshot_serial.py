# dshot_serial.py
import serial
import struct

class DShotSerial:
    def __init__(self, port='/dev/ttyUSB0', baud=921600):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            print(f"Connected to STM32 on {port}")
        except Exception as e:
            print(f"Serial connection failed: {e}")
            self.ser = None

    def crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else (crc << 1)
        return crc & 0xFFFF

    def send_motors_and_pwm(self, m_val, pwm_val):
        """
        Sends the motor value (DShot) to all 4 motors and 
        the steering value (PWM) to the servo.
        """
        if not self.ser:
            return

        # Clamp values to safe hardware limits
        m1 = m2 = m3 = m4 = max(0, min(2047, int(m_val)))
        pwm = max(0, min(1023, int(pwm_val)))

        # Pack: sync(1) + m1-m4(2 each) + pwm(2) = 11 bytes
        data = struct.pack('<BHHHHH', 0xAA, m1, m2, m3, m4, pwm)
        crc = self.crc16(data)
        packet = data + struct.pack('<H', crc)  # Total 13 bytes
        
        self.ser.write(packet)

    def disarm(self):
        """Sends the disarm signal (47) and neutral PWM."""
        print("Sending disarm signal...")
        for _ in range(10):
            self.send_motors_and_pwm(47, 523) # 47 is DShot disarm/stop

    def close(self):
        if self.ser:
            self.ser.close()
