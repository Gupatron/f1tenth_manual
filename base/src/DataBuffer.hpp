// DataBuffer.hpp
#ifndef DATABUFFER_HPP
#define DATABUFFER_HPP

#include <mutex>
#include <vector>
#include <optional>
#include "Message.hpp"

/**
 * Thread-safe data buffer for storing command values
 * Mirrors the Python DataBuffer class
 */
class DataBuffer {
private:
    mutable std::mutex mutex_;
    std::vector<double> motor_speed_;
    std::vector<double> steering_angle_;
    std::vector<double> look_theta_;
    
public:
    DataBuffer() = default;
    
    // Append new values to the buffer
    void append(double motor_speed, double steering_angle, double look_theta = 0.0) {
        std::lock_guard<std::mutex> lock(mutex_);
        motor_speed_.push_back(motor_speed);
        steering_angle_.push_back(steering_angle);
        look_theta_.push_back(look_theta);
    }
    
    // Get the latest message (if available)
    std::optional<Message> getLatest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (motor_speed_.empty() || steering_angle_.empty()) {
            return std::nullopt;
        }
        
        return Message(
            motor_speed_.back(),
            steering_angle_.back(),
            look_theta_.empty() ? 0.0 : look_theta_.back()
        );
    }
    
    // Clear all buffers (useful for reset)
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        motor_speed_.clear();
        steering_angle_.clear();
        look_theta_.clear();
    }
    
    // Get buffer sizes (for debugging)
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return motor_speed_.size();
    }
    
    // Print latest values (for debugging)
    void printLatest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!motor_speed_.empty() && !steering_angle_.empty()) {
            std::cout << "{" 
                      << motor_speed_.back() << ", "
                      << steering_angle_.back() << ", "
                      << (look_theta_.empty() ? 0.0 : look_theta_.back())
                      << "}" << std::endl;
        }
    }
};

#endif // DATABUFFER_HPP