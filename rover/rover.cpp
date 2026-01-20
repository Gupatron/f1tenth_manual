// rover.cpp
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include "DataBuffer.hpp"
#include "ZenohCommunicator.hpp"
#include "Message.hpp"

// Global flag for clean shutdown
std::atomic<bool> running(true);

void signalHandler(int signal) {
    std::cout << "\n[Rover] Received interrupt signal. Shutting down..." << std::endl;
    running = false;
}

/**
 * Process commands from buffer and send to hardware
 * This is where you'll integrate your serial communication
 */
void hardwareThread(DataBuffer& buffer, double frequency_hz = 50.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[Hardware] Thread started at " << frequency_hz << "Hz" << std::endl;
    std::cout << "[Hardware] TODO: Initialize serial communication to STM32" << std::endl;
    
    // Default safe values
    double current_motor_speed = 0.0;      // Zero throttle
    double current_steering_angle = 523.0;  // Center steering
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // Get latest commands from buffer
        auto msg_opt = buffer.getLatest();
        
        if (msg_opt) {
            current_motor_speed = msg_opt->motor_speed;
            current_steering_angle = msg_opt->steering_angle;
        }
        
        // TODO: Send to hardware via serial
        // Example: stm32.send_motors_and_pwm(current_motor_speed, current_steering_angle);
        
        // For now, just print (replace with actual hardware control)
        // std::cout << "[Hardware] Motor: " << current_motor_speed 
        //           << ", Steering: " << current_steering_angle << std::endl;
        
        // Sleep to maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[Hardware] Thread stopped" << std::endl;
}

int main(int argc, char* argv[]) {
    // Register signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "=== Rover Starting ===" << std::endl;
    
    // Create shared data buffer
    DataBuffer buffer;
    
    // Create Zenoh communicator
    std::string to_rover_key = "base/to/rover";
    std::string from_rover_key = "rover/to/base";
    
    ZenohCommunicator comm(from_rover_key, to_rover_key);
    
    // Set callback for messages from base
    comm.setMessageCallback([&buffer](const Message& msg) {
        // Store received commands in buffer
        buffer.append(msg.motor_speed, msg.steering_angle, msg.look_theta);
        
        // Optional: print received commands
        std::cout << "[Received] Motor: " << msg.motor_speed 
                  << ", Steering: " << msg.steering_angle << std::endl;
    });
    
    // Verify communicator is ready
    if (!comm.isReady()) {
        std::cerr << "[Rover] Failed to initialize Zenoh communicator" << std::endl;
        return 1;
    }
    
    std::cout << "[Rover] Communication initialized successfully" << std::endl;
    std::cout << "[Rover] Listening for commands..." << std::endl;
    std::cout << "[Rover] Press Ctrl+C to stop" << std::endl;
    
    // Start hardware control thread
    std::thread hardware_thread(hardwareThread, std::ref(buffer), 50.0);
    
    // Main thread waits for shutdown signal
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Clean shutdown
    std::cout << "[Rover] Waiting for threads to finish..." << std::endl;
    
    if (hardware_thread.joinable()) {
        hardware_thread.join();
    }
    
    // TODO: Send disarm signal to hardware
    std::cout << "[Rover] TODO: Send disarm signal (zero throttle)" << std::endl;
    
    std::cout << "[Rover] Shutdown complete" << std::endl;
    
    return 0;
}