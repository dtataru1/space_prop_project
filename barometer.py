from machine import Pin, Timer, PWM, I2C, ADC
from time import sleep

# Barometer address
BARO_DEFAULT_ADDR = 0x77

#DPS310 Barometer registers
DPS310_PRSB2 = 0x00  # Highest byte of pressure data
DPS310_PRSB1 = 0x01 #Medium byte of pressure data
DPS310_PRSB0 = 0x02 #Lowest byte of pressure data
DPS310_TMPB2 = 0x03  # Highest byte of temperature data
DPS310_TMPB1 = 0x04  # Highest byte of temperature data
DPS310_TMPB0 = 0x05  # Highest byte of temperature data
DPS310_PRSCFG = 0x06  # Pressure configuration
DPS310_TMPCFG = 0x07  # Temperature configuration
DPS310_MEASCFG = 0x08  # Sensor configuration
DPS310_CFGREG = 0x09  # Interrupt/FIFO configuration
DPS310_RESET = 0x0C  # Soft reset
DPS310_PRODREVID = 0x0D  # Register that contains the part ID
DPS310_TMPCOEFSRCE = 0x28  # Temperature calibration src


 
def reg_write(i2c, addr, reg, data):
    msg = bytearray()
    msg.append(data)
    
    i2c.writeto_mem(addr, reg, msg)

def reg_read(i2c, addr, reg, nbytes=1):
    
    if nbytes < 1:
        return bytearray()
    
    data = i2c.readfrom_mem(addr, reg, nbytes)
    
    return data
    
def setup_baro(i2c0):
    DATA_CONFIG = (1<<6) | (1<<5) |(1<<4) | (1<<1)
    reg_write(i2c0, Baro_addr, DPS310_PRSCFG, DATA_CONFIG)
    
    MEAS_CTRL = 1
    reg_write(i2c0, Baro_addr, DPS310_MEASCFG, MEAS_CTRL)
    
def read_baro(i2c0):
    
    data0 = reg_read(i2c0, Baro_addr, DPS310_PRSB0)
    data0 = int.from_bytes(data0, "big")
    data1 = reg_read(i2c0, Baro_addr, DPS310_PRSB1)
    data1 = int.from_bytes(data1, "big")
    data2 = reg_read(i2c0, Baro_addr, DPS310_PRSB2)
    data2 = int.from_bytes(data2, "big")
    
    pressure = (data2<<16) + (data1<<8) + data0
    
    return pressure


while True:
    i2c0 = machine.I2C(0, scl=Pin(1), sda=Pin(0))
    setup_baro(i2c0)
    pressure = read_baro(i2c0)
    print(pressure)
    sleep(0.1)
    
    
    
    
