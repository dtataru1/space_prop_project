from machine import Pin, Timer, PWM, I2C, ADC
from time import sleep

#Constants
EARTH_GRAVITY = 9.81

 
 

#Addres
Baro_addr = 0x77

def detect_launch():
    
    return True

def detect_apogee():
    
    return True

def open_parachute():
    
    return True


def detect_landing():
    
    return True
 
def reg_write(i2c, addr, reg, data):
    msg = bytearray()
    msg.append(data)
    
    i2c.writeto_mem(addr, reg, msg)
    
def reg_read(i2c, addr, reg, nbytes=1):
    
    if nbytes < 1:
        return bytearray()
    
    data = i2c.readfrom_mem(addr, reg, nbytes)
    
    return data
    
def setup_accel(i2c1):
    
    DATARATE_CONFIG = (LIS3DHTR_ACCL_DR_400 | LIS3DHTR_ACCL_XAXIS | LIS3DHTR_ACCL_YAXIS | LIS3DHTR_ACCL_ZAXIS)
    reg_write(i2c1, Accel_addr, LIS3DHTR_REG_CTRL1, DATARATE_CONFIG)
        
    DATA_CONFIG = (LIS3DHTR_ACCL_RANGE_2G | LIS3DHTR_BDU_CONT | LIS3DHTR_HR_DS)
    reg_write(i2c1, Accel_addr, LIS3DHTR_REG_CTRL4, DATA_CONFIG)
        
def read_accel(i2c1):
    data0 = reg_read(i2c1, Accel_addr, LIS3DHTR_REG_OUT_X_L)
    data0 = int.from_bytes(data0, "big")
    data1 = reg_read(i2c1, Accel_addr, LIS3DHTR_REG_OUT_X_H)
    data1 = int.from_bytes(data1, "big")
    
    xAccl = data1*256+data0
    if xAccl > 32767:
        xAccl -= 65536
    xAccl /= 16000
    
    data0 = reg_read(i2c1, Accel_addr, LIS3DHTR_REG_OUT_Y_L)
    data0 = int.from_bytes(data0, "big")
    data1 = reg_read(i2c1, Accel_addr, LIS3DHTR_REG_OUT_Y_H)
    data1 = int.from_bytes(data1, "big")
    
    yAccl = data1*256+data0
    if yAccl > 32767:
        yAccl -= 65536
    yAccl /= 16000
    
    data0 = reg_read(i2c1, Accel_addr, LIS3DHTR_REG_OUT_Z_L)
    data0 = int.from_bytes(data0, "big")
    data1 = reg_read(i2c1, Accel_addr, LIS3DHTR_REG_OUT_Z_H)
    data1 = int.from_bytes(data1, "big")
    
    zAccl = data1*256+data0
    if zAccl > 32767:
        zAccl -= 65536
    zAccl /= 16000 
    
    return xAccl, yAccl, zAccl


state = "Idle"
led = Pin(25, Pin.OUT)
i2c0 = machine.I2C(0, scl=Pin(1), sda=Pin(0))
i2c1 = machine.I2C(1, scl=Pin(19), sda=Pin(18))
setup_accel(i2c1)
calibration_done = 0
while True:

    if state == "Idle":
        led.off()
        state = "Armed"
        
    elif state == "Armed":
       #if calibration_done == 0:
           
        led.on()
        xAccl_buff = 0
        yAccl_buff = 0
        zAccl_buff = 0
        for i in range(0,5):
            xAccl, yAccl, zAccl = read_accel(i2c1)
            xAccl_buff += xAccl
            yAccl_buff += yAccl
            zAccl_buff += zAccl
        
        xAccl_buff /= 5
        yAccl_buff /= 5
        zAccl_buff /= 5
        print(zAccl_buff)
        if zAccl_buff < -2.0:
            state = "Climb"
    
   # elif state == "Climb":

    print(state)
    #elif state == "Apogee":


    #elif state == "Parachute Open":



    #elif state == "Landed":
    
    




