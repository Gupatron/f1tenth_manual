# dshot_serial.py (Fixed)
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
        Sends the motor value (throttle 0-2047) to all 4 motors and 
        the steering value (PWM 0-1023) to the servo.
        
        NOTE: m_val is a THROTTLE value (0-2047), NOT a DShot command number
        """
        if not self.ser:
            return

        # Clamp values to safe hardware limits
        # All motors get the same throttle value
        m1 = m2 = m3 = m4 = max(0, min(2047, int(m_val)))
        pwm = max(0, min(1023, int(pwm_val)))

        # Pack: sync(1) + m1-m4(2 each) + pwm(2) = 11 bytes
        data = struct.pack('<BHHHHH', 0xAA, m1, m2, m3, m4, pwm)
        crc = self.crc16(data)
        packet = data + struct.pack('<H', crc)  # Total 13 bytes
        
        self.ser.write(packet)

    def disarm(self):
        """
        Sends zero throttle and neutral PWM.
        Based on working dshot_communication.py pattern.
        """
        print("Sending disarm signal (zero throttle)...")
        for _ in range(50):  # 1 second at 50Hz
            self.send_motors_and_pwm(0, 523)  # 0 = zero throttle, 523 = neutral

    def close(self):
        if self.ser:
            self.ser.close()
