// base.cpp
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
    std::cout << "\n[Base] Received interrupt signal. Shutting down..." << std::endl;
    running = false;
}

/**
 * Sender thread - reads from buffer and sends over Zenoh
 * Mirrors the Python sender_thread function
 */
void senderThread(DataBuffer& buffer, ZenohCommunicator& comm, double frequency_hz = 20.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[Sender] Thread started at " << frequency_hz << "Hz" << std::endl;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // Get latest message from buffer
        auto msg_opt = buffer.getLatest();
        
        if (msg_opt) {
            // Send message
            comm.publish(*msg_opt);
            
            // Print for debugging (optional)
            buffer.printLatest();
        }
        
        // Sleep to maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[Sender] Thread stopped" << std::endl;
}

/**
 * Example controller thread - generates dummy commands
 * Replace this with your actual controller logic
 */
void controllerThread(DataBuffer& buffer, double frequency_hz = 20.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[Controller] Thread started (dummy mode)" << std::endl;
    std::cout << "[Controller] Replace this with your actual controller!" << std::endl;
    
    double counter = 0.0;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // Generate dummy values (replace with real controller logic)
        double motor_speed = 1048.0;      // Neutral RPM
        double steering_angle = 500.0;     // Center steering
        double look_theta = 0.0;
        
        // For testing: ramp values
        // motor_speed = 1048.0 + (counter * 10.0);
        // steering_angle = 500.0 + (counter * 5.0);
        
        // Write to buffer
        buffer.append(motor_speed, steering_angle, look_theta);
        
        counter += 0.1;
        
        // Sleep to maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[Controller] Thread stopped" << std::endl;
}

int main(int argc, char* argv[]) {
    // Register signal handler for clean shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "=== Base Station Starting ===" << std::endl;
    
    // Create shared data buffer
    DataBuffer buffer;
    
    // Create Zenoh communicator
    std::string to_rover_key = "base/to/rover";
    std::string from_rover_key = "rover/to/base";
    
    ZenohCommunicator comm(to_rover_key, from_rover_key);
    
    // Set callback for messages from rover (telemetry, etc.)
    comm.setMessageCallback([](const Message& msg) {
        std::cout << "[Received] Motor: " << msg.motor_speed 
                  << ", Steering: " << msg.steering_angle 
                  << ", Look: " << msg.look_theta << std::endl;
    });
    
    // Verify communicator is ready
    if (!comm.isReady()) {
        std::cerr << "[Base] Failed to initialize Zenoh communicator" << std::endl;
        return 1;
    }
    
    std::cout << "[Base] Communication initialized successfully" << std::endl;
    std::cout << "[Base] Press Ctrl+C to stop" << std::endl;
    
    // Start threads
    std::thread controller_thread(controllerThread, std::ref(buffer), 20.0);
    std::thread sender_thread(senderThread, std::ref(buffer), std::ref(comm), 20.0);
    
    // Main thread waits for shutdown signal
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Clean shutdown
    std::cout << "[Base] Waiting for threads to finish..." << std::endl;
    
    if (controller_thread.joinable()) {
        controller_thread.join();
    }
    
    if (sender_thread.joinable()) {
        sender_thread.join();
    }
    
    std::cout << "[Base] Shutdown complete" << std::endl;
    
    return 0;
}