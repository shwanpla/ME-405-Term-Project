from pyb import Pin, Timer 
 
class motor_driver: 
    def __init__(self, PWM_pin: Pin, DIR_pin: Pin, nSLP_pin: Pin, tim: Timer, chan: int): 
    # Store a copy of each input parameter as an attribute 
        self.DIR_pin  = Pin(DIR_pin,  mode=Pin.OUT_PP) 
        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP) 
        self.PWM_chan = tim.channel(chan, pin=PWM_pin, mode=Timer.PWM, pulse_width_percent=0)  
    def enable(self): 
        self.nSLP_pin.high() 
    
    def disable(self): 
        self.nSLP_pin.low() 
    
    def set_effort(self, effort: float): 
        if (effort > 0): 
            self.DIR_pin.low() 
            self.PWM_chan.pulse_width_percent(effort) 
        else: 
            self.DIR_pin.high() 
            self.PWM_chan.pulse_width_percent(-effort)