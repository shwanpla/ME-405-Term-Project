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
    def __init__(self, pin_name='PC0', vref=3.3, adc_bits=12,
                 samples=8, sample_delay_ms=0):
        """
        pin_name:          ADC pin name (default PC0)
        vref:              Reference voltage for ADC (default 3.3 V)
        adc_bits:          ADC resolution (default 12 bits)
        samples:           Number of readings to average
        sample_delay_ms:   Optional delay between samples (0 for fastest)
        """
        self._adc = ADC(Pin(pin_name))
        self._vref = float(vref)
        self._adc_max = float((1 << adc_bits) - 1)
        self._samples = max(1, int(samples))
        self._delay = int(sample_delay_ms)

    def read_voltage(self):
        """Return averaged ADC voltage in volts."""
        acc = 0
        for _ in range(self._samples):
            acc += self._adc.read()
            if self._delay:
                time.sleep_ms(self._delay)
        raw_avg = acc / self._samples
        return (raw_avg / self._adc_max) * self._vref


# ────────────────────────────────────────────────
# Convenience functions
# ────────────────────────────────────────────────
_default_monitor = None

def configure(pin_name='PC0', vref=3.3, adc_bits=12, samples=8, sample_delay_ms=0):
    """Optional one-time configuration (if you want custom settings)."""
    global _default_monitor
    _default_monitor = BatteryMonitor(pin_name, vref, adc_bits, samples, sample_delay_ms)

def battery_voltage():
    """Return the current ADC voltage in volts."""
    global _default_monitor
    if _default_monitor is None:
        _default_monitor = BatteryMonitor()
    return _default_monitor.read_voltage()
