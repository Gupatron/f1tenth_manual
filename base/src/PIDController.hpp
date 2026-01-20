// PIDController.hpp
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>
#include <cmath>
#include <algorithm>

/**
 * PID Controller for trajectory tracking
 * 
 * Implements a standard PID controller with anti-windup
 * Used to correct deviations from the desired trajectory
 */
class PIDController {
private:
    // PID gains
    double kp_;  // Proportional gain
    double ki_;  // Integral gain
    double kd_;  // Derivative gain
    
    // State variables
    double integral_;
    double previous_error_;
    std::chrono::steady_clock::time_point last_time_;
    
    // Anti-windup
    double integral_max_;
    double integral_min_;
    
    // Output limits
    double output_max_;
    double output_min_;
    
    // Deadzone
    double deadzone_;
    
    bool initialized_;
    
public:
    /**
     * Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0)
        : kp_(kp), ki_(ki), kd_(kd),
          integral_(0.0),
          previous_error_(0.0),
          integral_max_(1000.0),
          integral_min_(-1000.0),
          output_max_(1000.0),
          output_min_(-1000.0),
          deadzone_(0.0),
          initialized_(false) {
        
        last_time_ = std::chrono::steady_clock::now();
    }
    
    /**
     * Compute PID output
     * @param setpoint Desired value
     * @param measured Current measured value
     * @return Control output
     */
    double compute(double setpoint, double measured) {
        // Calculate error
        double error = setpoint - measured;
        
        // Apply deadzone
        if (std::abs(error) < deadzone_) {
            error = 0.0;
        }
        
        // Calculate dt
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time_).count();
        last_time_ = current_time;
        
        // Prevent division by zero or negative dt
        if (dt <= 0.0) {
            dt = 0.001;  // Default to 1ms
        }
        
        // Proportional term
        double p_term = kp_ * error;
        
        // Integral term with anti-windup
        integral_ += error * dt;
        integral_ = std::clamp(integral_, integral_min_, integral_max_);
        double i_term = ki_ * integral_;
        
        // Derivative term
        double d_term = 0.0;
        if (initialized_) {
            double derivative = (error - previous_error_) / dt;
            d_term = kd_ * derivative;
        }
        
        // Calculate output
        double output = p_term + i_term + d_term;
        
        // Clamp output
        output = std::clamp(output, output_min_, output_max_);
        
        // Update state
        previous_error_ = error;
        initialized_ = true;
        
        return output;
    }
    
    /**
     * Reset controller state
     */
    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
        initialized_ = false;
        last_time_ = std::chrono::steady_clock::now();
    }
    
    /**
     * Configuration methods
     */
    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    
    void setOutputLimits(double min, double max) {
        output_min_ = min;
        output_max_ = max;
    }
    
    void setIntegralLimits(double min, double max) {
        integral_min_ = min;
        integral_max_ = max;
    }
    
    void setDeadzone(double deadzone) {
        deadzone_ = std::abs(deadzone);
    }
    
    /**
     * Getters for debugging
     */
    double getProportionalGain() const { return kp_; }
    double getIntegralGain() const { return ki_; }
    double getDerivativeGain() const { return kd_; }
    double getIntegral() const { return integral_; }
    double getLastError() const { return previous_error_; }
};

/**
 * Dual PID Controller for X and Y tracking
 */
class DualPIDController {
private:
    PIDController x_controller_;
    PIDController y_controller_;
    
public:
    DualPIDController(double kp_x = 1.0, double ki_x = 0.0, double kd_x = 0.0,
                     double kp_y = 1.0, double ki_y = 0.0, double kd_y = 0.0)
        : x_controller_(kp_x, ki_x, kd_x),
          y_controller_(kp_y, ki_y, kd_y) {}
    
    /**
     * Compute control for both axes
     */
    void compute(double setpoint_x, double setpoint_y,
                 double measured_x, double measured_y,
                 double& control_x, double& control_y) {
        control_x = x_controller_.compute(setpoint_x, measured_x);
        control_y = y_controller_.compute(setpoint_y, measured_y);
    }
    
    void reset() {
        x_controller_.reset();
        y_controller_.reset();
    }
    
    PIDController& getXController() { return x_controller_; }
    PIDController& getYController() { return y_controller_; }
};

#endif // PID_CONTROLLER_HPP