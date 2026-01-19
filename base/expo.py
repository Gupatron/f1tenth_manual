# expo.py
"""
Exponential curve mapping for RC-style controller input.
Provides reduced sensitivity near center and increased sensitivity at extremes.
"""

class ExpoMapper:
    """
    Maps input values using an exponential curve.
    
    The expo curve formula: output = input * (expo * input² + (1 - expo))
    
    - expo = 0.0: Linear response (no expo)
    - expo = 0.5: Moderate expo (typical for RC cars)
    - expo = 1.0: Maximum expo (very soft center, aggressive ends)
    """
    
    def __init__(self, expo: float = 0.3):
        """
        Initialize the expo mapper.
        
        Args:
            expo: Exponential factor (0.0 to 1.0)
                 0.0 = linear, higher values = more exponential
        """
        self.expo = max(0.0, min(1.0, expo))  # Clamp between 0 and 1
    
    def apply(self, value: float, input_min: float = -1.0, input_max: float = 1.0) -> float:
        """
        Apply exponential curve to an input value.
        
        Args:
            value: Input value to map
            input_min: Minimum expected input value
            input_max: Maximum expected input value
            
        Returns:
            Mapped value with expo curve applied
        """
        # Normalize to -1.0 to 1.0 range
        normalized = (value - input_min) / (input_max - input_min) * 2.0 - 1.0
        
        # Apply expo curve: preserves sign, applies curve to magnitude
        sign = 1.0 if normalized >= 0 else -1.0
        abs_norm = abs(normalized)
        
        # Expo formula: output = input * (expo * input² + (1 - expo))
        expo_value = abs_norm * (self.expo * abs_norm * abs_norm + (1.0 - self.expo))
        
        # Restore sign and denormalize back to original range
        result = sign * expo_value
        return (result + 1.0) / 2.0 * (input_max - input_min) + input_min
    
    def set_expo(self, expo: float):
        """Update the expo factor."""
        self.expo = max(0.0, min(1.0, expo))


class DualExpoMapper:
    """
    Provides separate expo curves for steering and throttle.
    Common in RC applications where different axes need different feel.
    """
    
    def __init__(self, steering_expo: float = 0.3, throttle_expo: float = 0.5):
        """
        Initialize with separate expo values for steering and throttle.
        
        Args:
            steering_expo: Expo factor for steering (typically lower, 0.2-0.4)
            throttle_expo: Expo factor for throttle (typically higher, 0.4-0.6)
        """
        self.steering = ExpoMapper(steering_expo)
        self.throttle = ExpoMapper(throttle_expo)
    
    def apply_steering(self, value: float, input_min: float = -1.0, input_max: float = 1.0) -> float:
        """Apply steering expo curve."""
        return self.steering.apply(value, input_min, input_max)
    
    def apply_throttle(self, value: float, input_min: float = -1.0, input_max: float = 1.0) -> float:
        """Apply throttle expo curve."""
        return self.throttle.apply(value, input_min, input_max)


def apply_expo_simple(value: float, expo: float = 0.5, 
                      input_min: float = -1.0, input_max: float = 1.0) -> float:
    """
    Simple function-based expo application (no class needed).
    
    Args:
        value: Input value to map
        expo: Exponential factor (0.0 to 1.0)
        input_min: Minimum expected input value
        input_max: Maximum expected input value
        
    Returns:
        Mapped value with expo curve applied
    """
    expo = max(0.0, min(1.0, expo))
    
    # Normalize to -1.0 to 1.0
    normalized = (value - input_min) / (input_max - input_min) * 2.0 - 1.0
    
    # Apply expo
    sign = 1.0 if normalized >= 0 else -1.0
    abs_norm = abs(normalized)
    expo_value = abs_norm * (expo * abs_norm * abs_norm + (1.0 - expo))
    
    # Denormalize
    result = sign * expo_value
    return (result + 1.0) / 2.0 * (input_max - input_min) + input_min
