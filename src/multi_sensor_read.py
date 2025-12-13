from pyb import Pin, ADC, Timer
import array

class multiple_ir_readings:
    def __init__(self, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7):
        self.buf_size = 1

        self.adc1 = ADC(pin_1)  # Create ADC's
        self.adc2 = ADC(pin_2)
        self.adc3 = ADC(pin_3)
        self.adc4 = ADC(pin_4)
        self.adc5 = ADC(pin_5)
        self.adc6 = ADC(pin_6)
        self.adc7 = ADC(pin_7)

        # ODD in
       #self.adc8 = ADC(pin_8)

        self.tim = Timer(8, freq=100)        # Create timer
        self.rx1 = array.array('H', (0 for i in range(self.buf_size))) # ADC buffers of
        self.rx2 = array.array('H', (0 for i in range(self.buf_size))) # 100 16-bit words
        self.rx3 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx4 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx5 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx6 = array.array('H', (0 for i in range(self.buf_size)))
        self.rx7 = array.array('H', (0 for i in range(self.buf_size)))
        # Odd pin
        #self.rx8 = array.array('H', (0 for i in range(self.buf_size)))
    
    def read(self):
        # read analog values into buffers at 100Hz (takes one second)
        ADC.read_timed_multi((self.adc1, self.adc2, self.adc3, self.adc4, 
                              self.adc5, self.adc6, self.adc7), 
                             (self.rx1, self.rx2, self.rx3, self.rx4, 
                              self.rx5, self.rx6, self.rx7), 
                             self.tim)
        
        avgs = [sum(rx) / self.buf_size for rx in
            (self.rx1, self.rx2, self.rx3, self.rx4, self.rx5, self.rx6, self.rx7)]
        return avgs
