// base_vicon_purepursuit.cpp
// Complete trajectory following with Vicon feedback and PID correction
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <string>
#include <iomanip>
#include "DataBuffer.hpp"
#include "ZenohCommunicator.hpp"
#include "Message.hpp"
#include "PurePursuit.hpp"
#include "PIDController.hpp"
#include "ViconClient.hpp"
#include "TrajectoryPIDController.hpp"

// Global flag for clean shutdown
std::atomic<bool> running(true);

void signalHandler(int signal) {
    std::cout << "\n[Base] Received interrupt signal. Shutting down..." << std::endl;
    running = false;
}

/**
 * Vicon update thread - maintains connection and updates pose
 */
void viconThread(ViconClient& vicon, double frequency_hz = 100.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[Vicon] Thread started at " << frequency_hz << "Hz" << std::endl;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // Update VRPN - this receives data from Vicon
        vicon.update();
        
        // Sleep to maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[Vicon] Thread stopped" << std::endl;
}

/**
 * Control thread - computes commands using Pure Pursuit + PID
 */
void controlThread(DataBuffer& buffer, 
                   ViconClient& vicon,
                   TrajectoryPIDController& controller, 
                   double frequency_hz = 20.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[Control] Thread started at " << frequency_hz << "Hz" << std::endl;
    
    // Wait for valid Vicon data
    std::cout << "[Control] Waiting for Vicon data..." << std::endl;
    while (running && !vicon.isPoseValid()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (!running) return;
    
    std::cout << "[Control] Vicon data received. Starting control loop." << std::endl;
    
    int iteration = 0;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // Get current pose from Vicon
        ViconPose current_pose = vicon.getLatestPose();
        
        if (!current_pose.valid) {
            std::cerr << "[Control] WARNING: Invalid Vicon pose!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        
        // Compute control command with PID correction
        ControlCommand cmd;
        controller.computeControlWithPID(current_pose, cmd);
        
        // Convert steering angle to PWM (0-1023)
        double steering_pwm = 512 + (cmd.steering_angle / 0.524) * 400;
        steering_pwm = std::clamp(steering_pwm, 120.0, 880.0);
        
        // Write to buffer
        buffer.append(cmd.motor_speed, steering_pwm, 0.0);
        
        // Print status every second
        if (++iteration >= frequency_hz) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "[Control] "
                      << "Pos: (" << current_pose.x << ", " << current_pose.y << ") | "
                      << "Yaw: " << (current_pose.yaw * 180.0 / M_PI) << "° | "
                      << "Cross-track: " << controller.getCrossTrackError() << "m | "
                      << "Heading err: " << (controller.getHeadingError() * 180.0 / M_PI) << "° | "
                      << "Progress: " << (controller.getProgress() * 100.0) << "% | "
                      << "Speed: " << cmd.motor_speed << " | "
                      << "Steer: " << (cmd.steering_angle * 180.0 / M_PI) << "°"
                      << std::endl;
            iteration = 0;
        }
        
        // Check if goal reached
        if (controller.isGoalReached(current_pose)) {
            std::cout << "[Control] *** GOAL REACHED ***" << std::endl;
            // Send stop command
            buffer.append(1048.0, 512.0, 0.0);
            running = false;
        }
        
        // Sleep to maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[Control] Thread stopped" << std::endl;
}

/**
 * Sender thread - reads from buffer and sends over Zenoh
 */
void senderThread(DataBuffer& buffer, ZenohCommunicator& comm, double frequency_hz = 20.0) {
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    
    std::cout << "[Sender] Thread started at " << frequency_hz << "Hz" << std::endl;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        auto msg_opt = buffer.getLatest();
        if (msg_opt) {
            comm.publish(*msg_opt);
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
    
    std::cout << "[Sender] Thread stopped" << std::endl;
}

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <trajectory.csv> <vicon_server> <object_name>" << std::endl;
    std::cout << "\nArguments:" << std::endl;
    std::cout << "  trajectory.csv : Path to trajectory file" << std::endl;
    std::cout << "  vicon_server   : Vicon server address (e.g., 192.168.1.100:801)" << std::endl;
    std::cout << "  object_name    : Name of tracked object in Vicon (e.g., Rover)" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << program_name << " trajectory.csv 192.168.1.100:801 Rover" << std::endl;
}

int main(int argc, char* argv[]) {
    // Register signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "=== Vicon-Based Pure Pursuit with PID Correction ===" << std::endl;
    
    // Parse arguments
    if (argc < 4) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string trajectory_file = argv[1];
    std::string vicon_server = argv[2];
    std::string object_name = argv[3];
    
    // ========================================================================
    // CONFIGURATION - Modify these for your system
    // ========================================================================
    
    // Pure Pursuit parameters
    double lookahead_distance = 0.5;  // meters
    double wheelbase = 0.3;            // meters
    double max_steering_angle = 0.524; // radians (30 degrees)
    double base_speed = 1100.0;        // RPM
    
    // PID gains for position correction
    double pid_kp_x = 2.0;
    double pid_ki_x = 0.1;
    double pid_kd_x = 0.5;
    double pid_kp_y = 2.0;
    double pid_ki_y = 0.1;
    double pid_kd_y = 0.5;
    
    // PID gains for heading correction
    double heading_kp = 1.5;
    double heading_ki = 0.0;
    double heading_kd = 0.2;
    
    // Cross-track error gain
    double cross_track_kp = 1.0;
    
    // ========================================================================
    
    // Initialize Pure Pursuit
    PurePursuit pure_pursuit(lookahead_distance, wheelbase, max_steering_angle);
    pure_pursuit.setBaseSpeed(base_speed);
    pure_pursuit.setSpeedRange(1048.0, 1300.0);
    pure_pursuit.setAdaptiveSpeed(true);
    pure_pursuit.setGoalTolerance(0.15);
    
    // Load trajectory
    if (!pure_pursuit.loadTrajectoryCSV(trajectory_file)) {
        std::cerr << "[Main] Failed to load trajectory from: " << trajectory_file << std::endl;
        return 1;
    }
    
    // Initialize Trajectory + PID Controller
    TrajectoryPIDController controller(
        pure_pursuit,
        pid_kp_x, pid_ki_x, pid_kd_x,
        pid_kp_y, pid_ki_y, pid_kd_y,
        heading_kp, heading_ki, heading_kd
    );
    
    controller.setCrossTrackGain(cross_track_kp);
    controller.setMaxSteeringCorrection(0.3);  // ~17 degrees max correction
    
    std::cout << "[Main] Controller initialized with PID correction" << std::endl;
    
    // Initialize Vicon Client
    std::cout << "[Main] Connecting to Vicon: " << object_name << "@" << vicon_server << std::endl;
    ViconClient vicon(vicon_server, object_name);
    
    // Optional: Set origin if calibrated
    // controller.setOrigin(0.0, 0.0, 0.0);
    
    // Create data buffer
    DataBuffer buffer;
    
    // Create Zenoh communicator
    std::string to_rover_key = "base/to/rover";
    std::string from_rover_key = "rover/to/base";
    
    ZenohCommunicator comm(to_rover_key, from_rover_key);
    
    // Set callback for telemetry from rover
    comm.setMessageCallback([](const Message& msg) {
        // Handle telemetry if needed
    });
    
    if (!comm.isReady()) {
        std::cerr << "[Main] Failed to initialize Zenoh communicator" << std::endl;
        return 1;
    }
    
    std::cout << "[Main] System initialized. Press Ctrl+C to stop" << std::endl;
    std::cout << "\n=== PID Tuning Parameters ===" << std::endl;
    std::cout << "Position PID (X): Kp=" << pid_kp_x << ", Ki=" << pid_ki_x << ", Kd=" << pid_kd_x << std::endl;
    std::cout << "Position PID (Y): Kp=" << pid_kp_y << ", Ki=" << pid_ki_y << ", Kd=" << pid_kd_y << std::endl;
    std::cout << "Heading PID: Kp=" << heading_kp << ", Ki=" << heading_ki << ", Kd=" << heading_kd << std::endl;
    std::cout << "Cross-track gain: " << cross_track_kp << std::endl;
    std::cout << "============================\n" << std::endl;
    
    // Start threads
    std::thread vicon_thread(viconThread, std::ref(vicon), 100.0);
    std::thread control_thread(controlThread, std::ref(buffer), std::ref(vicon), 
                               std::ref(controller), 20.0);
    std::thread sender_thread(senderThread, std::ref(buffer), std::ref(comm), 20.0);
    
    // Main thread waits for shutdown signal
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Clean shutdown
    std::cout << "[Main] Waiting for threads to finish..." << std::endl;
    
    if (vicon_thread.joinable()) {
        vicon_thread.join();
    }
    
    if (control_thread.joinable()) {
        control_thread.join();
    }
    
    if (sender_thread.joinable()) {
        sender_thread.join();
    }
    
    std::cout << "[Main] Shutdown complete" << std::endl;
    
    return 0;
}