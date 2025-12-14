"""
ME 405 - Mechatronics
Authors: Billy Hawkins, Tarsem Pal, Arturo Ramirez
Filename: battery_adc.py

Description:
    Battery voltage monitoring module using ADC (Analog-to-Digital Converter).
    Reads voltage directly from pin PC0 with configurable averaging and sampling.
    Provides both class-based and convenience function interfaces for battery
    voltage measurements.

Classes:
    BatteryMonitor - Configurable ADC voltage reader with averaging support

Functions:
    configure() - Optional configuration for default monitor settings
    battery_voltage() - Quick access function to read current battery voltage

Hardware:
    - ADC Pin: PC0 (default, configurable)
    - Reference Voltage: 3.3V
    - Resolution: 12-bit ADC

Dependencies:
    - pyb.Pin
    - pyb.ADC
    - time

Notes:
    - No voltage scaling applied; reads direct ADC voltage
    - For battery monitoring, ensure voltage divider if battery > 3.3V
"""

from pyb import Pin, ADC
import time

class BatteryMonitor:
    """
    ADC-based battery voltage monitor with configurable averaging.
    """

    def __init__(self, pin_name='PC0', vref=3.3, adc_bits=12,
                 samples=8, sample_delay_ms=0):
        """
        Initialize the battery monitor.

        :param pin_name: ADC pin name (default 'PC0')
        :type pin_name: str
        :param vref: Reference voltage for ADC in volts (default 3.3)
        :type vref: float
        :param adc_bits: ADC resolution in bits (default 12)
        :type adc_bits: int
        :param samples: Number of readings to average (default 8)
        :type samples: int
        :param sample_delay_ms: Optional delay between samples in ms (default 0)
        :type sample_delay_ms: int
        """
        # Initialize ADC on specified pin
        self._adc = ADC(Pin(pin_name))

        # Store configuration parameters
        self._vref = float(vref)
        self._adc_max = float((1 << adc_bits) - 1)  # Max ADC value (e.g., 4095 for 12-bit)
        self._samples = max(1, int(samples))
        self._delay = int(sample_delay_ms)

    def read_voltage(self):
        """
        Read and return averaged ADC voltage in volts.

        :return: Averaged voltage reading in volts
        :rtype: float
        """
        acc = 0

        # Accumulate multiple samples for averaging
        for _ in range(self._samples):
            acc += self._adc.read()
            if self._delay:
                time.sleep_ms(self._delay)

        # Convert averaged raw ADC value to voltage
        raw_avg = acc / self._samples
        return (raw_avg / self._adc_max) * self._vref


# ────────────────────────────────────────────────
# Convenience functions
# ────────────────────────────────────────────────

# Module-level default monitor instance (lazy initialization)
_default_monitor = None


def configure(pin_name='PC0', vref=3.3, adc_bits=12, samples=8, sample_delay_ms=0):
    """
    Configure the default battery monitor with custom settings.

    :param pin_name: ADC pin name (default 'PC0')
    :type pin_name: str
    :param vref: Reference voltage for ADC in volts (default 3.3)
    :type vref: float
    :param adc_bits: ADC resolution in bits (default 12)
    :type adc_bits: int
    :param samples: Number of readings to average (default 8)
    :type samples: int
    :param sample_delay_ms: Optional delay between samples in ms (default 0)
    :type sample_delay_ms: int
    """
    global _default_monitor
    _default_monitor = BatteryMonitor(pin_name, vref, adc_bits, samples, sample_delay_ms)


def battery_voltage():
    """
    Quick-access function to read battery voltage using default settings.

    Automatically initializes with default settings on first call.

    :return: Current ADC voltage in volts
    :rtype: float
    """
    global _default_monitor

    # Lazy initialization: create default monitor if it doesn't exist
    if _default_monitor is None:
        _default_monitor = BatteryMonitor()

    return _default_monitor.read_voltage()
