# Pure Pursuit Trajectory Following - Complete Guide

This guide explains how to create trajectories and use them with the C++ Pure Pursuit controller.

## Workflow Overview

```
┌─────────────────────┐
│  1. Draw Trajectory │  ← Python GUI (trajectory_drawer.py)
│     trajectory.csv  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  2. Load Trajectory │  ← C++ Pure Pursuit (base_purepursuit.cpp)
│     into Controller │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  3. Follow Path     │  ← Autonomous driving
│     Send Commands   │
└─────────────────────┘
```

## Step 1: Draw Your Trajectory

### Configure Your Lab Environment

Edit `trajectory_drawer.py` at the top:

```python
# Physical room dimensions (in meters)
ROOM_WIDTH = 5.0   # Your lab width
ROOM_HEIGHT = 4.0  # Your lab height

# Waypoint spacing for pure pursuit (meters)
RESAMPLE_SPACING = 0.05  # Smaller = more waypoints, smoother path
```

### Run the Trajectory Drawer

```bash
python3 trajectory_drawer.py
```

### Draw Your Path

1. Click and drag with your mouse to draw the desired trajectory
2. The path is automatically:
   - Smoothed to remove jitter
   - Resampled to uniform spacing (critical for pure pursuit)
   - Converted to physical coordinates
3. Click **Save** to export

### Output Files

Three files are generated (e.g., `trajectory_20260120_143022`):

1. **trajectory_*.json** - Full metadata + waypoints
2. **trajectory_*.csv** - Simple format for C++ loading
3. **trajectory_*.hpp** - C++ header (can directly #include)

Example CSV output:
```csv
index,x_m,y_m,curvature
0,0.1250,3.8750,0.0000
1,0.1500,3.8500,0.0234
2,0.1750,3.8250,0.0189
...
```

## Step 2: Configure Pure Pursuit Controller

The Pure Pursuit algorithm has several tunable parameters:

### Key Parameters

```cpp
PurePursuit controller(
    0.5,    // lookahead_distance (meters) - HOW FAR TO LOOK AHEAD
    0.3,    // wheelbase (meters) - YOUR ROVER'S WHEELBASE
    0.524   // max_steering_angle (radians) - MAX STEERING (30°)
);

// Speed settings
controller.setBaseSpeed(1100.0);        // Base speed (RPM)
controller.setSpeedRange(1048.0, 1300.0);  // Min/max speed
controller.setAdaptiveSpeed(true);      // Slow down in tight turns
controller.setGoalTolerance(0.15);      // Goal reached tolerance (meters)
```

### Parameter Tuning Guide

| Parameter | Effect | Recommended Starting Value |
|-----------|--------|---------------------------|
| `lookahead_distance` | Larger = smoother but less accurate tracking | 0.3 - 0.8 meters |
| `wheelbase` | Must match your rover | Measure your rover |
| `max_steering_angle` | Physical servo limit | 0.524 rad (30°) |
| `base_speed` | How fast to drive | Start slow: 1100 RPM |
| `adaptive_speed` | Slow down in turns | true (recommended) |

**Tuning Tips:**
- **Too much overshoot?** → Decrease lookahead_distance
- **Oscillating/unstable?** → Increase lookahead_distance
- **Cutting corners?** → Decrease lookahead_distance, increase speed in turns
- **Not following sharp turns?** → Decrease lookahead_distance, enable adaptive_speed

## Step 3: Run Pure Pursuit Controller

### Build the System

```bash
mkdir build && cd build
cmake ..
make
```

This creates three executables:
- `base` - Manual control base station
- `base_purepursuit` - Autonomous pure pursuit controller
- `rover` - Rover receiver

### Run the Rover

On your Jetson/Rover:
```bash
./rover
```

The rover waits for commands and forwards them to the STM32.

### Run Pure Pursuit Controller

On your laptop/base station:
```bash
./base_purepursuit trajectory_20260120_143022.csv
```

The controller will:
1. Load the trajectory
2. Start computing steering and speed commands
3. Send commands to the rover at 20Hz
4. Print progress updates
5. Stop when goal is reached

### Example Output

```
=== Pure Pursuit Base Station ===
[PurePursuit] Loaded 256 waypoints from trajectory_20260120_143022.csv
[Base] Pure Pursuit controller initialized
[Base] Press Ctrl+C to stop

[PurePursuit] Progress: 5.5% | Pose: (0.25, 0.12, 0.15) | Speed: 1080 | Steering: 0.12 rad
[PurePursuit] Progress: 12.1% | Pose: (0.68, 0.45, 0.34) | Speed: 1095 | Steering: -0.08 rad
...
[PurePursuit] Goal reached!
```

## Integration with Your System

### Adding Localization (REQUIRED)

The Pure Pursuit controller needs to know the robot's current pose. You have several options:

#### Option 1: Motion Capture System (Recommended)

```cpp
// In base_purepursuit.cpp, replace simulated pose with real data
void updatePoseFromMoCap() {
    // Get pose from your motion capture system
    auto pose = mocap_system.getRobotPose();
    robot_x = pose.x;
    robot_y = pose.y;
    robot_theta = pose.theta;
}
```

#### Option 2: Visual Odometry / SLAM

```cpp
// Integrate with ROS, ORB-SLAM, etc.
void updatePoseFromSLAM() {
    auto pose = slam_system.getCurrentPose();
    robot_x = pose.x;
    robot_y = pose.y;
    robot_theta = pose.theta;
}
```

#### Option 3: Wheel Odometry (Less accurate)

```cpp
// Use encoder feedback
void updatePoseFromOdometry() {
    // Dead reckoning from wheel encoders
    // Note: Accumulates drift over time
}
```

### Steering Angle Conversion

The Pure Pursuit outputs steering angle in radians. Convert to your system's units:

```cpp
// Example: Convert to PWM (0-1023)
double steering_pwm = 512 + (cmd.steering_angle / max_steering_angle) * 400;
steering_pwm = std::clamp(steering_pwm, 0.0, 1023.0);

// Example: Convert to servo angle (0-180 degrees)
double servo_angle = 90 + (cmd.steering_angle * 180.0 / M_PI);
servo_angle = std::clamp(servo_angle, 0.0, 180.0);
```

### Speed Control Conversion

Convert from RPM to your motor controller's units:

```cpp
// If using DShot throttle values (0-2047)
double dshot_throttle = map_rpm_to_dshot(cmd.motor_speed);

// If using PWM duty cycle
double pwm_duty = map_rpm_to_pwm(cmd.motor_speed);
```

## File Format Reference

### CSV Format (trajectory_*.csv)

Simple format for C++ loading:
```csv
index,x_m,y_m,curvature
0,0.1250,3.8750,0.0000
1,0.1500,3.8500,0.0234
```

- `index`: Waypoint number
- `x_m`: X coordinate in meters
- `y_m`: Y coordinate in meters
- `curvature`: Path curvature at this point (1/meters)

### JSON Format (trajectory_*.json)

Full metadata:
```json
{
  "metadata": {
    "room_width_m": 5.0,
    "room_height_m": 4.0,
    "num_waypoints": 256,
    "waypoint_spacing_m": 0.05,
    "timestamp": "2026-01-20T14:30:22"
  },
  "waypoints": [
    {"x": 0.1250, "y": 3.8750, "curvature": 0.0000},
    {"x": 0.1500, "y": 3.8500, "curvature": 0.0234}
  ]
}
```

### C++ Header Format (trajectory_*.hpp)

Can be directly included:
```cpp
#include "trajectory_20260120_143022.hpp"

// Use the waypoints
controller.setTrajectory(trajectory::waypoints);
```

## Advanced Usage

### Custom Trajectory Generation

You can also generate trajectories programmatically:

```cpp
std::vector<Waypoint> waypoints;

// Circle trajectory
double radius = 1.0;
for (int i = 0; i < 100; ++i) {
    double theta = 2.0 * M_PI * i / 100.0;
    waypoints.emplace_back(
        radius * cos(theta),
        radius * sin(theta),
        1.0 / radius  // Curvature of circle
    );
}

controller.setTrajectory(waypoints);
```

### Speed Profiling

Adjust speed based on path characteristics:

```cpp
// In PurePursuit.hpp, modify calculateSpeed():
double calculateSpeed(const Waypoint& target) const {
    // Slow down proportional to curvature
    double speed = base_speed_ * (1.0 - target.curvature * 2.0);
    
    // Minimum speed in tight turns
    speed = std::max(speed, min_speed_);
    
    return speed;
}
```

### Multiple Trajectories

Switch between trajectories:

```cpp
// Load racing line
controller.loadTrajectoryCSV("racing_line.csv");

// Later, load parking trajectory
controller.loadTrajectoryCSV("parking.csv");
```

## Troubleshooting

### "No trajectory loaded!"
- Make sure CSV file exists
- Check file path is correct
- Verify CSV format matches expected format

### Robot doesn't follow path
- Check localization is working (print current pose)
- Verify steering angle conversion is correct
- Try increasing lookahead_distance
- Enable debug prints in Pure Pursuit

### Robot oscillates
- Lookahead distance too small
- Increase to 0.5-1.0 meters
- Check steering angle isn't inverted

### Cuts corners
- Lookahead distance too large
- Decrease to 0.2-0.4 meters
- Increase waypoint density (decrease RESAMPLE_SPACING)

### Speed issues
- Check RPM conversion is correct
- Verify motor controller limits
- Test with lower base_speed first

## Next Steps

1. **Add Localization** - Integrate motion capture or SLAM
2. **Tune Parameters** - Adjust lookahead and speeds for your rover
3. **Add Safety** - Implement collision avoidance, emergency stop
4. **Add Telemetry** - Send pose and status back to base station
5. **Add Replanning** - Dynamically update trajectory based on obstacles

## References

- Pure Pursuit Algorithm: [R. Craig Coulter, 1992](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- Trajectory Drawing GUI: `trajectory_drawer.py`
- Pure Pursuit Implementation: `PurePursuit.hpp`
- Base Station: `base_purepursuit.cpp`