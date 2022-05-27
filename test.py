from machine import Pin, Timer, PWM, I2C, ADC
from time import sleep
from sensors import I2C_Sensor, Accelerometer, Barometer
import sensors

def detect_launch():
    
    return True

def detect_apogee():
    
    return True

def open_parachute():
    
    return True


def detect_landing():
    
    return True
 

    

state = "IDLE"
led = Pin(25, Pin.OUT)
accel = Accelerometer(i2cbus=0, scl_pin=1, sda_pin=0)
baro = Barometer(i2cbus=1, scl_pin=19, sda_pin=18)
accel.accel_conf()
baro.baro_conf()


while True:
    if state == "IDLE":
        led.off()
        state = "ARMED"
        print("State changed from IDLE to ARMED")
        
    elif state == "ARMED":
        led.on()
        xAccl_buff = 0
        yAccl_buff = 0
        zAccl_buff = 0
        for i in range(0,5):
            xAccl, yAccl, zAccl = accel.accel_read()
            xAccl_buff += xAccl
            yAccl_buff += yAccl
            zAccl_buff += zAccl
        xAccl_buff /=5
        yAccl_buff /=5
        zAccl_buff /=5
        print(zAccl_buff)
        
        if zAccl_buff < -2.0:
            state = "CLIMB"
            print("State changed from ARMED to CLIMB")
    
    elif state == "CLIMB":
        pass
    
    elif state == "APOGEE":
        pass
    
    elif state == "PARACHUTE OPENED":
        pass
    
    elif state == "LANDED":
        pass


    




