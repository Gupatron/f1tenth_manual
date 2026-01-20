// PurePursuit.hpp
#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>

/**
 * Waypoint structure for trajectory following
 */
struct Waypoint {
    double x;          // meters
    double y;          // meters
    double curvature;  // 1/meters (optional, for speed profiling)
    
    Waypoint() : x(0.0), y(0.0), curvature(0.0) {}
    Waypoint(double x_, double y_, double c_ = 0.0) 
        : x(x_), y(y_), curvature(c_) {}
};

/**
 * Robot pose structure
 */
struct Pose {
    double x;       // meters
    double y;       // meters
    double theta;   // radians (heading angle)
    
    Pose() : x(0.0), y(0.0), theta(0.0) {}
    Pose(double x_, double y_, double theta_) 
        : x(x_), y(y_), theta(theta_) {}
};

/**
 * Control command output
 */
struct ControlCommand {
    double steering_angle;  // radians or PWM value
    double motor_speed;     // RPM or throttle value
    
    ControlCommand() : steering_angle(0.0), motor_speed(0.0) {}
    ControlCommand(double angle, double speed) 
        : steering_angle(angle), motor_speed(speed) {}
};

/**
 * Pure Pursuit Path Following Controller
 * 
 * Implements the classic pure pursuit algorithm for path tracking.
 * The algorithm finds a lookahead point on the path and calculates
 * the steering angle needed to reach it.
 */
class PurePursuit {
private:
    std::vector<Waypoint> path_;
    size_t closest_waypoint_idx_;
    
    // Configuration parameters
    double lookahead_distance_;    // meters
    double wheelbase_;             // meters (distance between front and rear axles)
    double max_steering_angle_;    // radians
    double base_speed_;            // base speed (RPM or m/s)
    double max_speed_;             // maximum speed
    double min_speed_;             // minimum speed
    
    // Speed profiling (slow down in turns)
    bool adaptive_speed_enabled_;
    double curvature_speed_factor_;  // How much to slow down based on curvature
    
    // Path completion
    double goal_tolerance_;        // meters - distance to consider goal reached
    
public:
    /**
     * Constructor
     * @param lookahead_distance Distance to look ahead on path (meters)
     * @param wheelbase Vehicle wheelbase (meters)
     * @param max_steering_angle Maximum steering angle (radians)
     */
    PurePursuit(double lookahead_distance = 0.5, 
                double wheelbase = 0.3,
                double max_steering_angle = 0.523599)  // 30 degrees
        : lookahead_distance_(lookahead_distance),
          wheelbase_(wheelbase),
          max_steering_angle_(max_steering_angle),
          base_speed_(1100.0),  // Default RPM
          max_speed_(1300.0),
          min_speed_(1048.0),  // Neutral
          adaptive_speed_enabled_(true),
          curvature_speed_factor_(0.5),
          goal_tolerance_(0.1),
          closest_waypoint_idx_(0) {
    }
    
    /**
     * Load trajectory from CSV file
     */
    bool loadTrajectoryCSV(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[PurePursuit] Failed to open: " << filename << std::endl;
            return false;
        }
        
        path_.clear();
        std::string line;
        
        // Skip header
        std::getline(file, line);
        
        // Read waypoints
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            
            std::getline(ss, item, ',');  // index
            
            double x, y, curv = 0.0;
            std::getline(ss, item, ',');
            x = std::stod(item);
            
            std::getline(ss, item, ',');
            y = std::stod(item);
            
            if (std::getline(ss, item, ',')) {
                curv = std::stod(item);
            }
            
            path_.emplace_back(x, y, curv);
        }
        
        file.close();
        
        std::cout << "[PurePursuit] Loaded " << path_.size() 
                  << " waypoints from " << filename << std::endl;
        
        return !path_.empty();
    }
    
    /**
     * Set trajectory programmatically
     */
    void setTrajectory(const std::vector<Waypoint>& waypoints) {
        path_ = waypoints;
        closest_waypoint_idx_ = 0;
        std::cout << "[PurePursuit] Trajectory set with " 
                  << path_.size() << " waypoints" << std::endl;
    }
    
    /**
     * Compute control command based on current robot pose
     */
    ControlCommand computeControl(const Pose& current_pose) {
        if (path_.empty()) {
            std::cerr << "[PurePursuit] No trajectory loaded!" << std::endl;
            return ControlCommand(0.0, min_speed_);
        }
        
        // Find closest waypoint
        updateClosestWaypoint(current_pose);
        
        // Find lookahead point
        Waypoint lookahead_point = findLookaheadPoint(current_pose);
        
        // Calculate steering angle
        double steering_angle = calculateSteeringAngle(current_pose, lookahead_point);
        
        // Calculate speed (adaptive based on curvature)
        double speed = calculateSpeed(lookahead_point);
        
        return ControlCommand(steering_angle, speed);
    }
    
    /**
     * Check if goal has been reached
     */
    bool isGoalReached(const Pose& current_pose) const {
        if (path_.empty()) return true;
        
        const Waypoint& goal = path_.back();
        double distance = std::sqrt(
            std::pow(goal.x - current_pose.x, 2) + 
            std::pow(goal.y - current_pose.y, 2)
        );
        
        return distance < goal_tolerance_;
    }
    
    /**
     * Get progress along path (0.0 to 1.0)
     */
    double getProgress() const {
        if (path_.empty()) return 1.0;
        return static_cast<double>(closest_waypoint_idx_) / path_.size();
    }
    
    /**
     * Configuration setters
     */
    void setLookaheadDistance(double distance) { lookahead_distance_ = distance; }
    void setBaseSpeed(double speed) { base_speed_ = speed; }
    void setSpeedRange(double min_speed, double max_speed) {
        min_speed_ = min_speed;
        max_speed_ = max_speed;
    }
    void setAdaptiveSpeed(bool enabled) { adaptive_speed_enabled_ = enabled; }
    void setGoalTolerance(double tolerance) { goal_tolerance_ = tolerance; }
    
    /**
     * Get current lookahead point (for debugging/visualization)
     */
    Waypoint getCurrentLookaheadPoint(const Pose& pose) const {
        return findLookaheadPoint(pose);
    }
    
    size_t getClosestWaypointIndex() const { return closest_waypoint_idx_; }
    
private:
    /**
     * Update closest waypoint index
     */
    void updateClosestWaypoint(const Pose& pose) {
        double min_distance = std::numeric_limits<double>::max();
        
        // Search forward from current index (path following optimization)
        size_t search_start = (closest_waypoint_idx_ > 0) ? 
                              closest_waypoint_idx_ - 1 : 0;
        
        for (size_t i = search_start; i < path_.size(); ++i) {
            double dist = std::sqrt(
                std::pow(path_[i].x - pose.x, 2) + 
                std::pow(path_[i].y - pose.y, 2)
            );
            
            if (dist < min_distance) {
                min_distance = dist;
                closest_waypoint_idx_ = i;
            }
        }
    }
    
    /**
     * Find lookahead point on the path
     */
    Waypoint findLookaheadPoint(const Pose& pose) const {
        // Start searching from closest waypoint
        for (size_t i = closest_waypoint_idx_; i < path_.size(); ++i) {
            double dist = std::sqrt(
                std::pow(path_[i].x - pose.x, 2) + 
                std::pow(path_[i].y - pose.y, 2)
            );
            
            if (dist >= lookahead_distance_) {
                return path_[i];
            }
        }
        
        // If no point found, return the last waypoint (goal)
        return path_.back();
    }
    
    /**
     * Calculate steering angle using pure pursuit geometry
     */
    double calculateSteeringAngle(const Pose& pose, const Waypoint& target) const {
        // Transform target point to vehicle frame
        double dx = target.x - pose.x;
        double dy = target.y - pose.y;
        
        // Rotate to vehicle frame
        double target_x = dx * std::cos(-pose.theta) - dy * std::sin(-pose.theta);
        double target_y = dx * std::sin(-pose.theta) + dy * std::cos(-pose.theta);
        
        // Pure pursuit formula: delta = atan(2 * L * target_y / ld^2)
        // where L = wheelbase, ld = lookahead distance
        double ld_sq = target_x * target_x + target_y * target_y;
        
        if (ld_sq < 1e-6) {
            return 0.0;  // At target, no steering needed
        }
        
        double curvature = 2.0 * target_y / ld_sq;
        double steering_angle = std::atan(curvature * wheelbase_);
        
        // Clamp to max steering angle
        steering_angle = std::clamp(steering_angle, 
                                    -max_steering_angle_, 
                                    max_steering_angle_);
        
        return steering_angle;
    }
    
    /**
     * Calculate speed based on path curvature
     */
    double calculateSpeed(const Waypoint& target) const {
        if (!adaptive_speed_enabled_) {
            return base_speed_;
        }
        
        // Slow down based on curvature
        // Higher curvature = tighter turn = slower speed
        double speed_reduction = target.curvature * curvature_speed_factor_;
        double speed = base_speed_ * (1.0 - speed_reduction);
        
        // Clamp to speed limits
        speed = std::clamp(speed, min_speed_, max_speed_);
        
        return speed;
    }
};

#endif // PURE_PURSUIT_HPP