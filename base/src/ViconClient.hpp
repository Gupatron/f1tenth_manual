// ViconClient.hpp
#ifndef VICON_CLIENT_HPP
#define VICON_CLIENT_HPP

#include <vrpn_Tracker.h>
#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <functional>

/**
 * Vicon pose data structure
 */
struct ViconPose {
    double x;        // meters
    double y;        // meters
    double z;        // meters
    double roll;     // radians
    double pitch;    // radians
    double yaw;      // radians (heading)
    
    double timestamp;  // seconds
    bool valid;
    
    ViconPose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0), 
                  timestamp(0), valid(false) {}
};

/**
 * Vicon Client using VRPN
 * Based on fdcl-vicon/ManeeshW implementation
 */
class ViconClient {
private:
    vrpn_Tracker_Remote* tracker_;
    std::string server_address_;
    std::string object_name_;
    
    // Latest pose data (thread-safe)
    mutable std::mutex pose_mutex_;
    ViconPose latest_pose_;
    
    // Callback function for external use
    std::function<void(const ViconPose&)> user_callback_;
    
    /**
     * VRPN callback - must be static
     */
    static void VRPN_CALLBACK handle_pose(void* user_data, const vrpn_TRACKERCB info) {
        ViconClient* client = static_cast<ViconClient*>(user_data);
        
        // Extract position
        double x = info.pos[0];
        double y = info.pos[1];
        double z = info.pos[2];
        
        // Convert quaternion to Euler angles (ZYX convention - matches Vicon)
        double qw = info.quat[3];
        double qx = info.quat[0];
        double qy = info.quat[1];
        double qz = info.quat[2];
        
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2.0 * (qw * qy - qz * qx);
        double pitch;
        if (std::abs(sinp) >= 1.0) {
            pitch = std::copysign(M_PI / 2.0, sinp);  // Use 90 degrees if out of range
        } else {
            pitch = std::asin(sinp);
        }
        
        // Yaw (z-axis rotation) - This is the heading angle
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // Update latest pose
        {
            std::lock_guard<std::mutex> lock(client->pose_mutex_);
            client->latest_pose_.x = x;
            client->latest_pose_.y = y;
            client->latest_pose_.z = z;
            client->latest_pose_.roll = roll;
            client->latest_pose_.pitch = pitch;
            client->latest_pose_.yaw = yaw;
            client->latest_pose_.timestamp = info.msg_time.tv_sec + 
                                            info.msg_time.tv_usec / 1000000.0;
            client->latest_pose_.valid = true;
        }
        
        // Call user callback if set
        if (client->user_callback_) {
            client->user_callback_(client->latest_pose_);
        }
    }
    
public:
    /**
     * Constructor
     * @param server_address Vicon server address (e.g., "192.168.1.100:801")
     * @param object_name Name of tracked object in Vicon (e.g., "Rover")
     */
    ViconClient(const std::string& server_address, const std::string& object_name)
        : server_address_(server_address), object_name_(object_name), tracker_(nullptr) {
        
        // Create VRPN tracker connection
        std::string connection_string = object_name + "@" + server_address;
        tracker_ = new vrpn_Tracker_Remote(connection_string.c_str());
        
        // Register callback
        if (tracker_) {
            tracker_->register_change_handler(this, handle_pose);
            std::cout << "[Vicon] Connected to: " << connection_string << std::endl;
        } else {
            std::cerr << "[Vicon] Failed to create tracker for: " << connection_string << std::endl;
        }
    }
    
    /**
     * Destructor
     */
    ~ViconClient() {
        if (tracker_) {
            delete tracker_;
            tracker_ = nullptr;
        }
    }
    
    /**
     * Update - must be called regularly to receive data
     * Call this in a loop at ~100Hz or higher
     */
    void update() {
        if (tracker_) {
            tracker_->mainloop();
        }
    }
    
    /**
     * Get latest pose (thread-safe)
     */
    ViconPose getLatestPose() const {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return latest_pose_;
    }
    
    /**
     * Check if pose data is valid
     */
    bool isPoseValid() const {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return latest_pose_.valid;
    }
    
    /**
     * Set callback for pose updates
     */
    void setPoseCallback(std::function<void(const ViconPose&)> callback) {
        user_callback_ = callback;
    }
    
    /**
     * Reset pose validity (useful for detecting connection loss)
     */
    void resetValidity() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_.valid = false;
    }
    
    /**
     * Get object name
     */
    std::string getObjectName() const {
        return object_name_;
    }
};

#endif // VICON_CLIENT_HPP