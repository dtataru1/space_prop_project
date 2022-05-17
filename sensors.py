from machine import Pin, Timer, PWM, I2C, ADC
from time import sleep

#Constants
EARTH_GRAVITY = 9.81

class I2C_Sensor():
    #Abs
    
    def __init__(self):
        
        pass
        
    





#Class for the LIS3DHTR accelerometer
class Accelerometer(I2C_Sensor):
    
    #Accelerometer address
    BARO_DEFAULT_ADDR = 0x19
    
    # LIS3DHTR Register Map
    LIS3DHTR_REG_WHOAMI                 = 0x0F # Who Am I Register
    LIS3DHTR_REG_CTRL1                  = 0x20 # Control Register-1
    LIS3DHTR_REG_CTRL2                  = 0x21 # Control Register-2
    LIS3DHTR_REG_CTRL3                  = 0x22 # Control Register-3
    LIS3DHTR_REG_CTRL4                  = 0x23 # Control Register-4
    LIS3DHTR_REG_CTRL5                  = 0x24 # Control Register-5
    LIS3DHTR_REG_CTRL6                  = 0x25 # Control Register-6
    LIS3DHTR_REG_REFERENCE              = 0x26 # Reference
    LIS3DHTR_REG_STATUS                 = 0x27 # Status Register
    LIS3DHTR_REG_OUT_X_L                = 0x28 # X-Axis LSB
    LIS3DHTR_REG_OUT_X_H                = 0x29 # X-Axis MSB
    LIS3DHTR_REG_OUT_Y_L                = 0x2A # Y-Axis LSB
    LIS3DHTR_REG_OUT_Y_H                = 0x2B # Y-Axis MSB
    LIS3DHTR_REG_OUT_Z_L                = 0x2C # Z-Axis LSB
    LIS3DHTR_REG_OUT_Z_H                = 0x2D # Z-Axis MSB
     
    # Accl Datarate configuration
    LIS3DHTR_ACCL_DR_PD                 = 0x00 # Power down mode
    LIS3DHTR_ACCL_DR_1                  = 0x10 # ODR = 1 Hz
    LIS3DHTR_ACCL_DR_10                 = 0x20 # ODR = 10 Hz
    LIS3DHTR_ACCL_DR_25                 = 0x30 # ODR = 25 Hz
    LIS3DHTR_ACCL_DR_50                 = 0x40 # ODR = 50 Hz
    LIS3DHTR_ACCL_DR_100                = 0x50 # ODR = 100 Hz
    LIS3DHTR_ACCL_DR_200                = 0x60 # ODR = 200 Hz
    LIS3DHTR_ACCL_DR_400                = 0x70 # ODR = 400 Hz
    LIS3DHTR_ACCL_DR_1620               = 0x80 # ODR = 1.620 KHz
    LIS3DHTR_ACCL_DR_1344               = 0x90 # ODR = 1.344 KHz
     
    # Accl Data update & Axis configuration
    LIS3DHTR_ACCL_LPEN                  = 0x00 # Normal Mode, Axis disabled
    LIS3DHTR_ACCL_XAXIS                 = 0x04 # X-Axis enabled
    LIS3DHTR_ACCL_YAXIS                 = 0x02 # Y-Axis enabled
    LIS3DHTR_ACCL_ZAXIS                 = 0x01 # Z-Axis enabled
     
    # Acceleration Full-scale selection
    LIS3DHTR_BDU_CONT                   = 0x00 # Continuous update, Normal Mode, 4-Wire Interface
    LIS3DHTR_BDU_NOT_CONT               = 0x80 # Output registers not updated until MSB and LSB reading
    LIS3DHTR_ACCL_BLE_MSB               = 0x40 # MSB first
    LIS3DHTR_ACCL_RANGE_16G             = 0x30 # Full scale = +/-16g
    LIS3DHTR_ACCL_RANGE_8G              = 0x20 # Full scale = +/-8g
    LIS3DHTR_ACCL_RANGE_4G              = 0x10 # Full scale = +/-4g
    LIS3DHTR_ACCL_RANGE_2G              = 0x00 # Full scale = +/-2g, LSB first
    LIS3DHTR_HR_DS                      = 0x00 # High-Resolution Disabled
    LIS3DHTR_HR_EN                      = 0x08 # High-Resolution Enabled
    LIS3DHTR_ST_0                       = 0x02 # Self Test 0
    LIS3DHTR_ST_1                       = 0x04 # Self Test 1
    LIS3DHTR_SIM_3                      = 0x01 # 3-Wire Interface

    
    def __init__(self):
        
        pass
    
    def accel_conf(self):
        
        pass
        
    def accel_read(self):
        
        pass
    
    def accel_test(self):
        
        pass
    
    
#Class for the DPS310 Barometer
class Barometer(I2C_Sensor):
    
    # Barometer address
    BARO_DEFAULT_ADDR = 0x77
    
    __init__(self):
        
        pass
    
    baro_conf(self):
        
        pass
    
    baro_read(self):
        
        pass
    
    baro_test(self):
        
        pass
    
    
    
    