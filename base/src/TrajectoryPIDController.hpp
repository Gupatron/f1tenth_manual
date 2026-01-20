// TrajectoryPIDController.hpp
#ifndef TRAJECTORY_PID_CONTROLLER_HPP
#define TRAJECTORY_PID_CONTROLLER_HPP

#include "PurePursuit.hpp"
#include "PIDController.hpp"
#include "ViconClient.hpp"
#include <cmath>
#include <iostream>

/**
 * Integrated Trajectory Following Controller
 * Combines Pure Pursuit with PID correction based on Vicon feedback
 * 
 * Architecture:
 * 1. Pure Pursuit computes nominal steering/speed from trajectory
 * 2. Vicon provides actual robot position
 * 3. PID corrects cross-track and heading errors
 */
class TrajectoryPIDController {
private:
    PurePursuit pure_pursuit_;
    DualPIDController position_pid_;  // X and Y position correction
    PIDController heading_pid_;       // Heading correction
    
    // Coordinate transformation
    double origin_x_;  // Vicon to world coordinate offset
    double origin_y_;
    double origin_yaw_;
    
    // Control limits
    double max_steering_correction_;  // Max PID steering correction (radians)
    double max_speed_correction_;     // Max PID speed correction
    
    // Cross-track error parameters
    double cross_track_kp_;  // Convert cross-track error to steering correction
    
    // Statistics
    double current_cross_track_error_;
    double current_heading_error_;
    
public:
    /**
     * Constructor
     * @param pure_pursuit Pure pursuit controller
     * @param pid_kp_x PID proportional gain for X
     * @param pid_ki_x PID integral gain for X
     * @param pid_kd_x PID derivative gain for X
     * @param pid_kp_y PID proportional gain for Y
     * @param pid_ki_y PID integral gain for Y
     * @param pid_kd_y PID derivative gain for Y
     * @param heading_kp Heading correction proportional gain
     * @param heading_ki Heading correction integral gain
     * @param heading_kd Heading correction derivative gain
     */
    TrajectoryPIDController(
        const PurePursuit& pure_pursuit,
        double pid_kp_x = 2.0, double pid_ki_x = 0.1, double pid_kd_x = 0.5,
        double pid_kp_y = 2.0, double pid_ki_y = 0.1, double pid_kd_y = 0.5,
        double heading_kp = 1.5, double heading_ki = 0.0, double heading_kd = 0.2)
        : pure_pursuit_(pure_pursuit),
          position_pid_(pid_kp_x, pid_ki_x, pid_kd_x, pid_kp_y, pid_ki_y, pid_kd_y),
          heading_pid_(heading_kp, heading_ki, heading_kd),
          origin_x_(0.0), origin_y_(0.0), origin_yaw_(0.0),
          max_steering_correction_(0.3),  // ~17 degrees
          max_speed_correction_(200.0),   // RPM correction
          cross_track_kp_(1.0),
          current_cross_track_error_(0.0),
          current_heading_error_(0.0) {
        
        // Configure PID limits
        position_pid_.getXController().setOutputLimits(-1.0, 1.0);
        position_pid_.getYController().setOutputLimits(-1.0, 1.0);
        heading_pid_.setOutputLimits(-max_steering_correction_, max_steering_correction_);
    }
    
    /**
     * Set coordinate origin (Vicon to world frame)
     * Call this during calibration
     */
    void setOrigin(double x, double y, double yaw = 0.0) {
        origin_x_ = x;
        origin_y_ = y;
        origin_yaw_ = yaw;
    }
    
    /**
     * Transform Vicon coordinates to world coordinates
     */
    Pose viconToWorld(const ViconPose& vicon_pose) const {
        // Rotate and translate from Vicon frame to world frame
        double dx = vicon_pose.x - origin_x_;
        double dy = vicon_pose.y - origin_y_;
        
        double cos_yaw = std::cos(-origin_yaw_);
        double sin_yaw = std::sin(-origin_yaw_);
        
        Pose world_pose;
        world_pose.x = dx * cos_yaw - dy * sin_yaw;
        world_pose.y = dx * sin_yaw + dy * cos_yaw;
        world_pose.theta = normalizeAngle(vicon_pose.yaw - origin_yaw_);
        
        return world_pose;
    }
    
    /**
     * Compute control command with PID correction
     * @param vicon_pose Current pose from Vicon
     * @param cmd Output command (modified by reference)
     */
    void computeControlWithPID(const ViconPose& vicon_pose, ControlCommand& cmd) {
        // Convert Vicon pose to world frame
        Pose current_pose = viconToWorld(vicon_pose);
        
        // Get pure pursuit nominal command
        cmd = pure_pursuit_.computeControl(current_pose);
        
        // Get desired waypoint from pure pursuit
        Waypoint target = pure_pursuit_.getCurrentLookaheadPoint(current_pose);
        
        // Calculate cross-track error (perpendicular distance to path)
        current_cross_track_error_ = calculateCrossTrackError(current_pose, target);
        
        // Calculate heading error (difference between desired and actual heading)
        double desired_heading = std::atan2(target.y - current_pose.y, 
                                           target.x - current_pose.x);
        current_heading_error_ = normalizeAngle(desired_heading - current_pose.theta);
        
        // PID correction for cross-track error
        // Convert cross-track error to steering correction
        double cross_track_correction = cross_track_kp_ * current_cross_track_error_;
        
        // PID correction for heading error
        double heading_correction = heading_pid_.compute(0.0, current_heading_error_);
        
        // Apply corrections
        cmd.steering_angle += (cross_track_correction + heading_correction);
        
        // Clamp steering angle
        double max_steering = 0.524;  // 30 degrees
        cmd.steering_angle = std::clamp(cmd.steering_angle, -max_steering, max_steering);
        
        // Speed correction based on error magnitude
        // Slow down if error is large
        double error_magnitude = std::abs(current_cross_track_error_) + 
                                std::abs(current_heading_error_);
        
        if (error_magnitude > 0.2) {  // 20cm or ~11 degrees combined error
            double speed_factor = 1.0 - std::min(error_magnitude / 1.0, 0.3);
            cmd.motor_speed *= speed_factor;
        }
    }
    
    /**
     * Calculate cross-track error (shortest distance to path)
     */
    double calculateCrossTrackError(const Pose& current, const Waypoint& target) const {
        // Vector from current position to target
        double dx = target.x - current.x;
        double dy = target.y - current.y;
        
        // Cross-track error is perpendicular distance
        // Positive = right of path, Negative = left of path
        double cross_track = dx * std::sin(current.theta) - dy * std::cos(current.theta);
        
        return cross_track;
    }
    
    /**
     * Normalize angle to [-pi, pi]
     */
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * Reset PID controllers
     */
    void reset() {
        position_pid_.reset();
        heading_pid_.reset();
    }
    
    /**
     * Configuration methods
     */
    void setCrossTrackGain(double kp) {
        cross_track_kp_ = kp;
    }
    
    void setMaxSteeringCorrection(double max_rad) {
        max_steering_correction_ = max_rad;
        heading_pid_.setOutputLimits(-max_rad, max_rad);
    }
    
    /**
     * Get error statistics (for logging/debugging)
     */
    double getCrossTrackError() const { return current_cross_track_error_; }
    double getHeadingError() const { return current_heading_error_; }
    double getProgress() const { return pure_pursuit_.getProgress(); }
    
    /**
     * Check if goal reached
     */
    bool isGoalReached(const ViconPose& vicon_pose) const {
        Pose current_pose = viconToWorld(vicon_pose);
        return pure_pursuit_.isGoalReached(current_pose);
    }
    
    /**
     * Access to underlying controllers
     */
    PurePursuit& getPurePursuit() { return pure_pursuit_; }
    DualPIDController& getPositionPID() { return position_pid_; }
    PIDController& getHeadingPID() { return heading_pid_; }
};

#endif // TRAJECTORY_PID_CONTROLLER_HPP