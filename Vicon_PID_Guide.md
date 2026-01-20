## Vicon-Based Trajectory Following with PID Control

Complete guide for autonomous trajectory following using Vicon motion capture feedback with PID error correction.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    BASE STATION (Laptop)                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌────────────┐    ┌──────────────┐    ┌────────────────────┐  │
│  │   Vicon    │───>│  Trajectory  │───>│  Control Commands  │  │
│  │  Position  │    │   + PID      │    │  (Steering/Speed)  │  │
│  │   Feedback │    │  Controller  │    └─────────┬──────────┘  │
│  └────────────┘    └──────────────┘              │              │
│        ▲                  │                       │              │
│        │                  │ Error Correction     │              │
│        │                  ▼                       ▼              │
│  ┌────────────────────────────────┐    ┌──────────────────┐    │
│  │    Pure Pursuit Algorithm      │    │   Zenoh Sender   │    │
│  │  (Nominal Path Following)      │    │                  │    │
│  └────────────────────────────────┘    └─────────┬────────┘    │
│                                                   │              │
└───────────────────────────────────────────────────┼──────────────┘
                                                    │ WiFi
┌───────────────────────────────────────────────────┼──────────────┐
│                      ROVER (Jetson)               │              │
├───────────────────────────────────────────────────┼──────────────┤
│                                                   ▼              │
│                                         ┌──────────────────┐    │
│                                         │  Zenoh Receiver  │    │
│                                         └─────────┬────────┘    │
│                                                   │              │
│                                                   ▼              │
│                                         ┌──────────────────┐    │
│                                         │   STM32 Motor    │    │
│                                         │    Controller    │    │
│                                         └──────────────────┘    │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
                                ▲
                                │
                          ┌─────┴─────┐
                          │   VICON   │
                          │  SYSTEM   │
                          └───────────┘
```

## Control Flow

1. **Vicon** provides real-time robot position (x, y, yaw) at 100Hz
2. **Pure Pursuit** computes nominal steering angle and speed from trajectory
3. **PID Controllers** correct errors:
   - **Cross-track PID**: Corrects lateral deviation from path
   - **Heading PID**: Corrects angular deviation from desired heading
4. **Commands** sent to rover via Zenoh at 20Hz

## Prerequisites

### 1. Install VRPN (for Vicon communication)

VRPN is the protocol Vicon uses to stream motion capture data.

```bash
# Clone VRPN repository
git clone https://github.com/vrpn/vrpn.git
cd vrpn

# Build quat library first
cd quat
# Edit Makefile and uncomment the appropriate line for your system
# For 64-bit Linux: uncomment "HW_OS := pc_linux64"
make
sudo make install

# Build VRPN
cd ..
make
sudo make install

# Build server and client
cd server_src
make
sudo make install

cd ../client_src
make
sudo make install
```

### 2. Install other dependencies

All other dependencies (Zenoh, pthread) should already be installed from the previous setup.

## Building the System

```bash
mkdir build && cd build
cmake ..
make
```

This creates four executables:
- `base` - Manual control
- `base_purepursuit` - Pure pursuit without Vicon
- **`base_vicon_purepursuit`** - Full system with Vicon + PID
- `rover` - Rover receiver

## Vicon Setup

### 1. Configure Vicon System

In Vicon Tracker or Nexus:

1. Create a rigid body object for your rover
2. Name it clearly (e.g., "Rover", "Vehicle1")
3. Enable VRPN streaming:
   - Go to Tools → VRPN
   - Enable "Start VRPN Server"
   - Note the IP address and port (default: 801)

### 2. Verify Vicon Connection

Test the connection with a simple VRPN client:

```bash
# If you have vrpn_print_devices
vrpn_print_devices Rover@192.168.1.100:801
```

You should see position and orientation data streaming.

### 3. Coordinate Frame Calibration

**IMPORTANT**: You need to align Vicon coordinates with your trajectory coordinates.

The trajectory drawer assumes:
- Origin (0,0) at bottom-left corner of room
- X-axis pointing right
- Y-axis pointing up

Vicon might have a different origin. To calibrate:

1. Place your rover at the trajectory origin (bottom-left corner)
2. Note the Vicon coordinates (x, y, yaw)
3. Set this as the origin in code (see Configuration section)

## Running the Complete System

### Step 1: Draw Trajectory

```bash
python3 trajectory_drawer.py
# Draw path, click Save
# Output: trajectory_20260120_143022.csv
```

### Step 2: Start Rover

On the Jetson:
```bash
./rover
```

### Step 3: Start Control System

On your laptop:
```bash
./base_vicon_purepursuit trajectory_20260120_143022.csv 192.168.1.100:801 Rover
```

Arguments:
- `trajectory_20260120_143022.csv` - Your trajectory file
- `192.168.1.100:801` - Vicon server IP:port
- `Rover` - Object name in Vicon

### Expected Output

```
=== Vicon-Based Pure Pursuit with PID Correction ===
[PurePursuit] Loaded 256 waypoints from trajectory_20260120_143022.csv
[Vicon] Connected to: Rover@192.168.1.100:801
[Main] Controller initialized with PID correction

=== PID Tuning Parameters ===
Position PID (X): Kp=2.0, Ki=0.1, Kd=0.5
Position PID (Y): Kp=2.0, Ki=0.1, Kd=0.5
Heading PID: Kp=1.5, Ki=0.0, Kd=0.2
Cross-track gain: 1.0
============================

[Vicon] Thread started at 100Hz
[Control] Thread started at 20Hz
[Sender] Thread started at 20Hz
[Control] Waiting for Vicon data...
[Control] Vicon data received. Starting control loop.

[Control] Pos: (0.125, 0.089) | Yaw: 15.3° | Cross-track: 0.023m | Heading err: -2.1° | Progress: 5.5% | Speed: 1095 | Steer: 12.3°
[Control] Pos: (0.456, 0.234) | Yaw: 22.7° | Cross-track: 0.012m | Heading err: -0.8° | Progress: 18.2% | Speed: 1105 | Steer: 8.1°
...
[Control] *** GOAL REACHED ***
```

## Configuration and Tuning

### In `base_vicon_purepursuit.cpp`

Find the **CONFIGURATION** section (around line 230):

```cpp
// Pure Pursuit parameters
double lookahead_distance = 0.5;  // meters - HOW FAR TO LOOK AHEAD
double wheelbase = 0.3;            // meters - YOUR ROVER'S WHEELBASE
double max_steering_angle = 0.524; // radians (30 degrees)
double base_speed = 1100.0;        // RPM - BASE DRIVING SPEED

// PID gains for position correction
double pid_kp_x = 2.0;   // X-axis proportional gain
double pid_ki_x = 0.1;   // X-axis integral gain
double pid_kd_x = 0.5;   // X-axis derivative gain
double pid_kp_y = 2.0;   // Y-axis proportional gain
double pid_ki_y = 0.1;   // Y-axis integral gain
double pid_kd_y = 0.5;   // Y-axis derivative gain

// PID gains for heading correction
double heading_kp = 1.5;  // Heading proportional gain
double heading_ki = 0.0;  // Heading integral gain (usually 0)
double heading_kd = 0.2;  // Heading derivative gain

// Cross-track error gain
double cross_track_kp = 1.0;  // Convert cross-track error to steering
```

### Coordinate Frame Calibration

To set the Vicon origin (if your Vicon frame doesn't match trajectory frame):

```cpp
// After initializing controller (around line 275)
controller.setOrigin(
    vicon_x_offset,   // X offset in meters
    vicon_y_offset,   // Y offset in meters
    vicon_yaw_offset  // Yaw offset in radians
);
```

### PID Tuning Guide

#### Cross-Track Error Correction

**Purpose**: Corrects lateral deviation from the path

**Start with**: `cross_track_kp = 1.0`

**Symptoms**:
- Rover oscillates left/right → **Decrease** to 0.5-0.8
- Rover doesn't correct deviations → **Increase** to 1.5-2.0
- Rover drifts off path → **Increase** significantly (2.0-3.0)

#### Heading PID

**Purpose**: Corrects angular deviation

**Start with**: 
- `heading_kp = 1.5`
- `heading_ki = 0.0` (usually keep at zero)
- `heading_kd = 0.2`

**Tuning Process**:

1. **Proportional (Kp)**:
   - Too low → Slow heading correction
   - Too high → Oscillation, overshoot
   - Typical range: 1.0-3.0

2. **Integral (Ki)**:
   - Usually keep at 0 for heading
   - Only add if persistent steady-state error
   - If used: 0.01-0.1

3. **Derivative (Kd)**:
   - Reduces overshoot
   - Too high → Jittery motion
   - Typical range: 0.1-0.5

#### Position PID (Advanced)

Currently not actively used but available for future enhancement.

**When to tune**:
- If you want to correct position errors directly
- For more aggressive error correction

**Recommended start**:
- Kp = 2.0
- Ki = 0.1
- Kd = 0.5

## Troubleshooting

### No Vicon Data

```
[Control] Waiting for Vicon data...
```

**Causes**:
1. Vicon server not running
2. Wrong IP address or port
3. Object not visible to Vicon cameras
4. VRPN server not enabled in Vicon

**Solutions**:
1. Check Vicon system is running and tracking
2. Verify object name matches exactly (case-sensitive)
3. Test with `vrpn_print_devices`
4. Enable VRPN in Vicon Tools menu

### Rover Oscillates

**Symptoms**: Rover weaves left and right

**Causes**:
- PID gains too high
- Lookahead distance too small

**Solutions**:
1. Decrease `cross_track_kp` (try 0.5)
2. Decrease `heading_kp` (try 1.0)
3. Increase `lookahead_distance` (try 0.7-1.0m)

### Rover Doesn't Correct Errors

**Symptoms**: Rover drifts off path and doesn't recover

**Causes**:
- PID gains too low
- Cross-track gain too low

**Solutions**:
1. Increase `cross_track_kp` (try 1.5-2.0)
2. Increase `heading_kp` (try 2.0-2.5)
3. Check Vicon data is valid

### Rover Cuts Corners

**Symptoms**: Doesn't follow sharp turns

**Causes**:
- Lookahead distance too large
- Speed too high for turns

**Solutions**:
1. Decrease `lookahead_distance` (try 0.3-0.4m)
2. Enable adaptive speed (should be on by default)
3. Decrease `base_speed`

### Coordinate Frame Mismatch

**Symptoms**: Rover goes in wrong direction

**Causes**:
- Vicon frame doesn't match trajectory frame

**Solutions**:
1. Calibrate origin using `setOrigin()`
2. Check Vicon Y-axis direction
3. Verify trajectory was drawn correctly

## Performance Optimization

### Update Frequencies

Current settings:
- Vicon: 100Hz
- Control: 20Hz
- Sender: 20Hz

**To increase responsiveness**:
```cpp
// In main()
std::thread vicon_thread(viconThread, std::ref(vicon), 120.0);  // 120Hz
std::thread control_thread(controlThread, ..., 50.0);  // 50Hz
```

**Warning**: Higher frequencies = more CPU usage

### Network Latency

If experiencing lag:

1. Use wired Ethernet for Vicon if possible
2. Reduce WiFi interference
3. Check network bandwidth usage
4. Consider increasing control frequency

## Advanced Features

### Adding Custom Logging

```cpp
// In controlThread(), add:
std::ofstream log_file("tracking_log.csv");
log_file << "time,x,y,yaw,cross_track,heading_error,speed,steering\n";

while (running) {
    // ... existing code ...
    
    // Log data
    log_file << std::chrono::system_clock::now().time_since_epoch().count() << ","
             << current_pose.x << ","
             << current_pose.y << ","
             << current_pose.yaw << ","
             << controller.getCrossTrackError() << ","
             << controller.getHeadingError() << ","
             << cmd.motor_speed << ","
             << cmd.steering_angle << "\n";
}
```

### Dynamic Gain Adjustment

For testing different gains without recompiling:

```cpp
// Read gains from config file
std::ifstream config("pid_config.txt");
config >> pid_kp_x >> pid_ki_x >> pid_kd_x;
// ... etc
```

### Visualization

Consider adding:
- RViz integration for path visualization
- Real-time plotting of errors
- Web dashboard for monitoring

## System Flow Summary

1. **Trajectory Planning** (Python GUI) → CSV file
2. **Vicon System** → Real-time position at 100Hz
3. **Pure Pursuit** → Nominal path following
4. **PID Controller** → Error correction
5. **Zenoh** → Command transmission
6. **Rover** → Executes commands
7. **Loop** → Continuous feedback and correction

## Key Files

- `trajectory_drawer.py` - Draw trajectories
- `PIDController.hpp` - PID implementation
- `ViconClient.hpp` - Vicon/VRPN interface
- `TrajectoryPIDController.hpp` - Integrated controller
- `base_vicon_purepursuit.cpp` - Main application
- `PurePursuit.hpp` - Path following algorithm

## Next Steps

1. Test with simple straight-line trajectory
2. Tune PID gains for your specific rover
3. Calibrate coordinate frames
4. Test with complex curved paths
5. Add safety features (emergency stop, collision avoidance)
6. Implement obstacle avoidance

## References

- VRPN Documentation: http://vrpn.github.io/
- PID Tuning: https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
- Pure Pursuit: R. Craig Coulter, 1992
