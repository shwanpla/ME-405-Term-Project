"""
Multi-sensor IR array reader using timed ADC sampling.
Reads 7 IR reflectance sensors simultaneously using timer-triggered ADC conversions.

Hardware:
    - IR Sensor Array: 7 analog IR reflectance sensors
    - ADC Resolution: 12-bit (0-4095)
    - Sampling Timer: Timer 8 at 100Hz

Notes:
    - Uses read_timed_multi for synchronized sensor readings
    - Buffer size of 1 provides instantaneous readings without averaging
    - Sensors indexed 1-7 from left to right across the array
"""
from pyb import Pin, ADC, Timer
import array


class multiple_ir_readings:
    """
    Seven-sensor IR array interface with timer-synchronized ADC sampling.
    """

    def __init__(self, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7):
        """
        Initialize IR sensor array with 7 ADC pins.

        :param pin_1: Pin object for sensor 1 (leftmost)
        :type pin_1: Pin
        :param pin_2: Pin object for sensor 2
        :type pin_2: Pin
        :param pin_3: Pin object for sensor 3
        :type pin_3: Pin
        :param pin_4: Pin object for sensor 4 (center)
        :type pin_4: Pin
        :param pin_5: Pin object for sensor 5
        :type pin_5: Pin
        :param pin_6: Pin object for sensor 6
        :type pin_6: Pin
        :param pin_7: Pin object for sensor 7 (rightmost)
        :type pin_7: Pin
        """
        self.buf_size = 1

        self.adc1 = ADC(pin_1)
        self.adc2 = ADC(pin_2)
        self.adc3 = ADC(pin_3)
        self.adc4 = ADC(pin_4)
        self.adc5 = ADC(pin_5)
        self.adc6 = ADC(pin_6)
        self.adc7 = ADC(pin_7)

        self.tim = Timer(8, freq=100)
        self.rx1 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx2 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx3 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx4 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx5 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx6 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx7 = array.array('H', (0 for i in range(self.buf_size)))

    def read(self):
        """
        Read all 7 IR sensors simultaneously using timed ADC sampling.

        :return: List of 7 averaged ADC values [sensor1, sensor2, ..., sensor7]
        :rtype: list
        """
        ADC.read_timed_multi((self.adc1, self.adc2, self.adc3, self.adc4,
                              self.adc5, self.adc6, self.adc7),
                             (self.rx1, self.rx2, self.rx3, self.rx4,
                              self.rx5, self.rx6, self.rx7),
                             self.tim)

        avgs = [sum(rx) / self.buf_size for rx in
            (self.rx1, self.rx2, self.rx3, self.rx4, self.rx5, self.rx6, self.rx7)]
        return avgs
