# config.py

# Set this to "XBOX" or "PS4"
ACTIVE_PROFILE = "PS4"

PROFILES = {
    "PS4": {
        "AXIS_STEER": 0,
        "AXIS_L2": 2,  # PS4 L2 is often Axis 2
        "AXIS_R2": 5,
    },
    "XBOX": {
        "AXIS_STEER": 0,
        "AXIS_L2": 4,  # Xbox L2 (LT) is typically Axis 4
        "AXIS_R2": 5,
    }
}

# Extract current mapping
MAP = PROFILES[ACTIVE_PROFILE]

# Target Ranges (Remains the same for both)
STEER_MIN, STEER_MAX = 120, 880
L2_MIN, L2_MAX = 48, 1047
R2_MIN, R2_MAX = 1049, 2047
NEUTRAL_VALUE = 1048
