from pyb import Pin, ADC, Timer
import array

class LineFollower:
    def __init__(self, multi_sensor_read_object, black, white, bias=0.0):
        """
        Initialize line follower controller
        
        Args:
            multi_sensor_read_object: Instance of multiple_ir_readings
            black: ADC value for black calibration
            white: ADC value for white calibration
            bias: Steering bias (positive = right bias, negative = left bias)
                  Use 0.5 to 1.5 for mild bias, up to 3.0 for strong bias
        """
        self.ir = multi_sensor_read_object
        self.black = black
        self.white = white
        self.bias = bias  # Bias term to prefer one direction
        # Position weights for 7 sensors (centered at 0)
        self.weights = [-3, -2, -1, 0, 1, 2, 3]
        
    def calculate_error(self):
        """
        Calculate steering error based on centroid of sensor positions
        
        Returns:
            error: Float value from approximately -3 to +3
                Negative = line is to the left, turn left
                Positive = line is to the right, turn right
                0 = line is centered
        """
        readings = self.ir.read()
        normalized = []
        
        # Normalize each reading between 0 and 1
        for reading in readings:
            norm_value = (reading - self.white) / (self.black - self.white)
            # Clamp to [0, 1] range to handle noise
            norm_value = max(0.0, min(1.0, norm_value))
            normalized.append(norm_value)
        
        # Calculate centroid: sum(position * intensity) / sum(intensity)
        numerator = sum(w * n for w, n in zip(self.weights, normalized))
        denominator = sum(normalized)
        
        # Avoid division by zero
        if denominator < 0.01:  # Small threshold instead of exactly 0
            return 0.0  # No line detected, return neutral
        
        # Centroid position gives the error
        error = numerator / denominator
        
        # Add bias term (positive = right bias, negative = left bias)
        biased_error = error + self.bias
        
        return biased_error
    
    def set_bias(self, bias):
        """
        Set the steering bias dynamically
        
        Args:
            bias: Steering bias (positive = right bias, negative = left bias)
        """
        self.bias = bias
    
    def get_raw_readings(self):
        """Return raw sensor readings for debugging/calibration"""
        return self.ir.read()
    
    def calibrate(multi_sensor_read_object):
       ir = multi_sensor_read_object
       values = ir.read()
       avg = sum(values)/7
       return avg