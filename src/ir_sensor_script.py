from pyb import Pin, ADC

class ir_sensor:
    def __init__(self, pin):
        self.pin = pin
        self.sensor = ADC(self.pin)
    def read(self):
        self.value = ADC.read(self.sensor)
        return self.value


