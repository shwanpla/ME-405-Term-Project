"""
Line following controller using centroid-based error calculation.
Computes steering error from IR sensor array with configurable bias for directional preference.

Hardware:
    - IR Sensor Array: 7 sensors (indexed -3 to +3 from left to right)
    - ADC Resolution: 12-bit (0-4095)

Notes:
    - Positive error = line to the right, turn right
    - Negative error = line to the left, turn left
    - Bias allows preferential steering for forks or curves
"""

from pyb import Pin, ADC, Timer
import array


class LineFollower:
    """
    Line following controller with weighted centroid error calculation.
    """

    def __init__(self, multi_sensor_read_object, black, white, bias=0.0):
        """
        Initialize line follower with calibration values and steering bias.

        :param multi_sensor_read_object: IR sensor array object
        :type multi_sensor_read_object: multiple_ir_readings
        :param black: ADC value for black surface (line)
        :type black: int
        :param white: ADC value for white surface (background)
        :type white: int
        :param bias: Steering bias (positive=right, negative=left, range: -3 to +3)
        :type bias: float
        """
        self.ir = multi_sensor_read_object
        self.black = black
        self.white = white
        self.bias = bias
        self.weights = [-3, -2, -1, 0, 1, 2, 3]
        
    def calculate_error(self):
        """
        Calculate steering error using weighted centroid of normalized sensor readings.

        :return: Steering error (-3 to +3, including bias)
        :rtype: float
        """
        readings = self.ir.read()
        normalized = []

        for reading in readings:
            norm_value = (reading - self.white) / (self.black - self.white)
            norm_value = max(0.0, min(1.0, norm_value))
            normalized.append(norm_value)

        numerator = sum(w * n for w, n in zip(self.weights, normalized))
        denominator = sum(normalized)

        if denominator < 0.01:
            return 0.0

        error = numerator / denominator
        biased_error = error + self.bias

        return biased_error

    def set_bias(self, bias):
        """
        Update steering bias dynamically.

        :param bias: New steering bias value
        :type bias: float
        """
        self.bias = bias

    def get_raw_readings(self):
        """
        Get raw sensor readings for debugging or calibration.

        :return: List of raw ADC values from all sensors
        :rtype: list
        """
        return self.ir.read()

    def calibrate(multi_sensor_read_object):
        """
        Calculate average sensor reading for calibration.

        :param multi_sensor_read_object: IR sensor array object
        :type multi_sensor_read_object: multiple_ir_readings
        :return: Average ADC value across all 7 sensors
        :rtype: float
        """
        ir = multi_sensor_read_object
        values = ir.read()
        avg = sum(values) / 7
        return avg