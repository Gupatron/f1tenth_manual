// base_purepursuit.cpp
// Base station with Pure Pursuit controller for autonomous trajectory following
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <string>
#include "DataBuffer.hpp"
#include "ZenohCommunicator.hpp"
#include "Message.hpp"
#include "PurePursuit.hpp"

// Global flag for clean shutdown
std::atomic<bool> running(true);

void signalHandler(int signal) {
    std::cout << "\n[Base] Received interrupt signal. Shutting down..." << std::endl;
    running = false;
}

// Global variables for robot state (normally from localization system)
std::atomic<double> robot_x(0.0);
std::atomic<double> robot_y(0.0);
std::atomic<double> robot_theta(0.0);

/**
 * Pure Pursuit controller thread
 * Reads robot pose and computes steering/speed commands
 */
void purePursuitThread(DataBuffer& buffer, PurePursuit& controller, double frequency_hz = 20.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[PurePursuit] Thread started at " << frequency_hz << "Hz" << std::endl;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // Get current robot pose (from localization system)
        Pose current_pose(robot_x.load(), robot_y.load(), robot_theta.load());
        
        // Compute control command
        ControlCommand cmd = controller.computeControl(current_pose);
        
        // Convert steering angle to your system's units
        // Example: Convert radians to PWM (0-1023)
        double steering_pwm = 512 + (cmd.steering_angle / controller.max_steering_angle_) * 400;
        steering_pwm = std::clamp(steering_pwm, 0.0, 1023.0);
        
        // Write to buffer
        buffer.append(cmd.motor_speed, steering_pwm, 0.0);
        
        // Check if goal reached
        if (controller.isGoalReached(current_pose)) {
            std::cout << "[PurePursuit] Goal reached!" << std::endl;
            // Send stop command
            buffer.append(1048.0, 512.0, 0.0);  // Neutral
            running = false;
        }
        
        // Print status every second
        static int counter = 0;
        if (++counter >= frequency_hz) {
            std::cout << "[PurePursuit] Progress: " 
                      << (controller.getProgress() * 100.0) << "% | "
                      << "Pose: (" << current_pose.x << ", " << current_pose.y 
                      << ", " << current_pose.theta << ") | "
                      << "Speed: " << cmd.motor_speed << " | "
                      << "Steering: " << cmd.steering_angle << " rad"
                      << std::endl;
            counter = 0;
        }
        
        // Sleep to maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[PurePursuit] Thread stopped" << std::endl;
}

/**
 * Sender thread - reads from buffer and sends over Zenoh
 */
void senderThread(DataBuffer& buffer, ZenohCommunicator& comm, double frequency_hz = 20.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[Sender] Thread started at " << frequency_hz << "Hz" << std::endl;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // Get latest message from buffer
        auto msg_opt = buffer.getLatest();
        
        if (msg_opt) {
            comm.publish(*msg_opt);
        }
        
        // Sleep to maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[Sender] Thread stopped" << std::endl;
}

int main(int argc, char* argv[]) {
    // Register signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "=== Pure Pursuit Base Station ===" << std::endl;
    
    // Check for trajectory file argument
    std::string trajectory_file = "trajectory.csv";
    if (argc > 1) {
        trajectory_file = argv[1];
    }
    
    // Create Pure Pursuit controller
    PurePursuit controller(
        0.5,    // lookahead_distance (meters)
        0.3,    // wheelbase (meters)
        0.524   // max_steering_angle (30 degrees in radians)
    );
    
    // Configure controller
    controller.setBaseSpeed(1100.0);        // Base RPM
    controller.setSpeedRange(1048.0, 1300.0);  // Speed limits
    controller.setAdaptiveSpeed(true);      // Slow down in turns
    controller.setGoalTolerance(0.15);      // 15cm tolerance
    
    // Load trajectory
    if (!controller.loadTrajectoryCSV(trajectory_file)) {
        std::cerr << "[Base] Failed to load trajectory from: " << trajectory_file << std::endl;
        std::cerr << "[Base] Usage: " << argv[0] << " <trajectory.csv>" << std::endl;
        return 1;
    }
    
    // TODO: Initialize localization system
    // For now, we'll simulate starting at origin
    std::cout << "[Base] WARNING: Using simulated pose at origin (0, 0, 0)" << std::endl;
    std::cout << "[Base] TODO: Integrate with your localization system!" << std::endl;
    
    // Create shared data buffer
    DataBuffer buffer;
    
    // Create Zenoh communicator
    std::string to_rover_key = "base/to/rover";
    std::string from_rover_key = "rover/to/base";
    
    ZenohCommunicator comm(to_rover_key, from_rover_key);
    
    // Set callback for telemetry from rover (could include pose updates)
    comm.setMessageCallback([](const Message& msg) {
        // TODO: Update robot pose from telemetry if available
        // robot_x = msg.motor_speed;  // Example - replace with actual pose data
        // robot_y = msg.steering_angle;
        // robot_theta = msg.look_theta;
    });
    
    if (!comm.isReady()) {
        std::cerr << "[Base] Failed to initialize Zenoh communicator" << std::endl;
        return 1;
    }
    
    std::cout << "[Base] Pure Pursuit controller initialized" << std::endl;
    std::cout << "[Base] Press Ctrl+C to stop" << std::endl;
    
    // Start threads
    std::thread controller_thread(purePursuitThread, std::ref(buffer), std::ref(controller), 20.0);
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