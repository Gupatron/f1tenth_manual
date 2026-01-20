// Message.hpp
#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <cstdint>
#include <cstring>
#include <array>

/**
 * Message structure for rover communication
 * Matches Python struct.pack('ddd', omega, theta, look_theta)
 */
struct Message {
    double motor_speed;      // RPM or throttle value
    double steering_angle;   // Steering angle in degrees or PWM value
    double look_theta;       // Camera/turret orientation (optional for now)
    
    // Default constructor
    Message() : motor_speed(0.0), steering_angle(0.0), look_theta(0.0) {}
    
    // Constructor with values
    Message(double speed, double angle, double look = 0.0)
        : motor_speed(speed), steering_angle(angle), look_theta(look) {}
    
    // Serialize to binary (24 bytes - 3 doubles)
    std::array<uint8_t, 24> toBinary() const {
        std::array<uint8_t, 24> buffer;
        std::memcpy(buffer.data(), &motor_speed, sizeof(double));
        std::memcpy(buffer.data() + 8, &steering_angle, sizeof(double));
        std::memcpy(buffer.data() + 16, &look_theta, sizeof(double));
        return buffer;
    }
    
    // Deserialize from binary
    static Message fromBinary(const uint8_t* data, size_t size) {
        if (size < 24) {
            return Message(); // Return default if insufficient data
        }
        
        Message msg;
        std::memcpy(&msg.motor_speed, data, sizeof(double));
        std::memcpy(&msg.steering_angle, data + 8, sizeof(double));
        std::memcpy(&msg.look_theta, data + 16, sizeof(double));
        return msg;
    }
};

#endif // MESSAGE_HPP