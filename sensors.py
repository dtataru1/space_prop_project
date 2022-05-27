from machine import Pin, Timer, PWM, I2C, ADC
from time import sleep

#Accelerometer address
LIS3DHTR_ADDR = 0x19

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


class I2C_Sensor():
    #DEFAULT SENSOR CLASS
    
    def __init__(self, i2cbus, scl_pin, sda_pin, addr):
        self.i2c = I2C(i2cbus, scl=Pin(scl_pin), sda=Pin(sda_pin))
        self.addr = addr
        
    def reg_write(self, reg, data):
        msg = bytearray()
        msg.append(data)
        self.i2c.writeto_mem(self.addr, reg, msg)
    
    def reg_read(self, reg, nbytes=1):
        if nbytes < 1:
            return bytearray()
        data = self.i2c.readfrom_mem(self.addr, reg, nbytes)
        return data
    
    
    def get_twos_complement(self, raw_val, length):

        val = raw_val

        if raw_val & (1 << (length - 1)):
            val = raw_val - (1 << length)

        return val

    
#Class for the LIS3DHTR accelerometer
class Accelerometer(I2C_Sensor):
    


    
    def __init__(self, i2cbus, scl_pin, sda_pin):

        super().__init__(i2cbus, scl_pin, sda_pin, LIS3DHTR_ADDR)
    
    def accel_conf(self):
        DATARATE_CONFIG = (LIS3DHTR_ACCL_DR_400 | LIS3DHTR_ACCL_XAXIS | LIS3DHTR_ACCL_YAXIS | LIS3DHTR_ACCL_ZAXIS) #CONSTANT, TO BE CHANGED
        self.reg_write(LIS3DHTR_REG_CTRL1, DATARATE_CONFIG)
        DATA_CONFIG = (LIS3DHTR_ACCL_RANGE_2G | LIS3DHTR_BDU_CONT | LIS3DHTR_HR_DS) #CONSTANT, TO BE CHANGED
        self.reg_write(LIS3DHTR_REG_CTRL4, DATA_CONFIG)
        
        
    def accel_read(self):
        
        xdata0 = self.reg_read(LIS3DHTR_REG_OUT_X_L)
        xdata0 = int.from_bytes(xdata0, "big")
        xdata1 = self.reg_read(LIS3DHTR_REG_OUT_X_H)
        xdata1 = int.from_bytes(xdata1, "big")
        
        xAccl = xdata1*256+xdata0
        if xAccl > 32767:
           xAccl -= 65536
        xAccl /= 16000
    
        ydata0 = self.reg_read(LIS3DHTR_REG_OUT_Y_L)
        ydata0 = int.from_bytes(ydata0, "big")
        ydata1 = self.reg_read(LIS3DHTR_REG_OUT_Y_H)
        ydata1 = int.from_bytes(ydata1, "big")
        
        yAccl = ydata1*256+ydata0
        if yAccl > 32767:
           yAccl -= 65536
        yAccl /= 16000
        
        zdata0 = self.reg_read(LIS3DHTR_REG_OUT_Z_L)
        zdata0 = int.from_bytes(zdata0, "big")
        zdata1 = self.reg_read(LIS3DHTR_REG_OUT_Z_H)
        zdata1 = int.from_bytes(zdata1, "big")
        
        zAccl = zdata1*256+zdata0
        if zAccl > 32767:
           zAccl -= 65536
        zAccl /= 16000
    
        return xAccl, yAccl, zAccl

    def accel_test(self):
        
        pass
    
    
#Class for the DPS310 Barometer
class Barometer(I2C_Sensor):

    def __init__(self, i2cbus, scl_pin, sda_pin):
        super().__init__(i2cbus, scl_pin, sda_pin, BARO_DEFAULT_ADDR)
        
    def correct_temperature(self):
        self.reg_write(0x0E, 0xA5)
        self.reg_write(0x0F, 0x96)
        self.reg_write(0x62, 0x02)
        self.reg_write(0x0E, 0x00)
        self.reg_write(0x0F, 0x00)
        
    def baro_conf(self):
        self.correct_temperature() 
        PRES_DATA_CONFIG = (1<<6) | (1<<5) |(0<<4) | (0<<3) | (0<<2) | (1<<1) | (0<<0) #CONSTANT, TO BE CHANGED
        self.reg_write(DPS310_PRSCFG, PRES_DATA_CONFIG)
        TEMP_DATA_CONFIG = (1<<6) | (1<<5) |(0<<4) | (0<<3) | (0<<2) | (1<<1) | (0<<0) #CONSTANT, TO BE CHANGED
        self.reg_write(DPS310_TMPCFG, TEMP_DATA_CONFIG)
        MEAS_CTRL = (1<<2) | (1<<1) |(1<<0) #CONSTANT, TO BE CHANGED
        self.reg_write(DPS310_MEASCFG, MEAS_CTRL) 
              
        # Oversampling Rate Configuration

        #DPS.__bus.write_byte_data(DPS.__addr, 0x09, 0x0C)


    def baro_raw_read(self):
        data0 = self.reg_read(DPS310_PRSB0)
        data0 = int.from_bytes(data0, "big")
        data1 = self.reg_read(DPS310_PRSB1)
        data1 = int.from_bytes(data1, "big")
        data2 = self.reg_read(DPS310_PRSB2)
        data2 = int.from_bytes(data2, "big")
        
        Pressure = (data2<<16) | (data1<<8) | data0
        Pressure = self.get_twos_complement(Pressure, 24)
        return Pressure
    
    def temp_raw_read(self):
        data0 = self.reg_read(DPS310_TMPB0)
        data0 = int.from_bytes(data0, "big")
        data1 = self.reg_read(DPS310_TMPB1)
        data1 = int.from_bytes(data1, "big")
        data2 = self.reg_read(DPS310_TMPB2)
        data2 = int.from_bytes(data2, "big")
        
        Temperature = (data2<<16) | (data1<<8) | data0
        Temperature = self.get_twos_complement(Temperature, 24)
        return Temperature
            
    
    def get_pressure_calib_coeffs(self):
        src13 = self.reg_read(0x13)
        src13 = int.from_bytes(src13, "big")
        src14 = self.reg_read(0x14)
        src14 = int.from_bytes(src14, "big")
        src15 = self.reg_read(0x15)
        src15 = int.from_bytes(src15, "big")
        src16 = self.reg_read(0x16)
        src16 = int.from_bytes(src16, "big")
        src17 = self.reg_read(0x17)
        src17 = int.from_bytes(src17, "big")
        src18 = self.reg_read(0x18)
        src18 = int.from_bytes(src18, "big")
        src19 = self.reg_read(0x19)
        src19 = int.from_bytes(src19, "big")
        src1A = self.reg_read(0x1A)
        src1A = int.from_bytes(src1A, "big")
        src1B = self.reg_read(0x1B)
        src1B = int.from_bytes(src1B, "big")
        src1C = self.reg_read(0x1C)
        src1C = int.from_bytes(src1C, "big")
        src1D = self.reg_read(0x1D)
        src1D = int.from_bytes(src1D, "big")
        src1E = self.reg_read(0x1E)
        src1E = int.from_bytes(src1E, "big")
        src1F = self.reg_read(0x1F)
        src1F = int.from_bytes(src1F, "big")
        src20 = self.reg_read(0x20)
        src20 = int.from_bytes(src20, "big")
        src21 = self.reg_read(0x21)
        src21 = int.from_bytes(src21, "big")

        c00 = (src13 << 12) | (src14 << 4) | (src15 >> 4)
        c00 = self.get_twos_complement(c00, 20)
        c10 = ((src15 & 0x0F) << 16) | (src16 << 8) | src17
        c10 = self.get_twos_complement(c10, 20)
        c20 = (src1C << 8) | src1D
        c20 = self.get_twos_complement(c20, 16)
        c30 = (src20 << 8) | src21
        c30 = self.get_twos_complement(c30, 16)
        c01 = (src18 << 8) | src19
        c01 = self.get_twos_complement(c01, 16)
        c11 = (src1A << 8) | src1B
        c11 = self.get_twos_complement(c11, 16)
        c21 = (src1E < 8) | src1F
        c21 = self.get_twos_complement(c21, 16)

        return c00, c10, c20, c30, c01, c11, c21
    
    def get_temperature_calib_coeffs(self):
        src10 = self.reg_read(0x10)
        src10 = int.from_bytes(src10, "big")
        src11 = self.reg_read(0x11)
        src11 = int.from_bytes(src11, "big")
        src12 = self.reg_read(0x12)
        src12 = int.from_bytes(src12, "big")

        c0 = (src10 << 4) | (src11 >> 4)
        c0 = self.get_twos_complement(c0, 12)
        c1 = ((src11 & 0x0F) << 8) | src12
        c1 = self.get_twos_complement(c1, 12)
        return c0, c1
               
    def scaled_pressure(self):
        kP = 3670016
        raw_p = self.baro_raw_read()
        scaled_p = raw_p / kP
        return scaled_p
    
    def scaled_temperature(self):
        kT = 3670016
        raw_t = self.temp_raw_read()
        scaled_t = raw_t/kT
        return scaled_t
    
    
    def comp_pressure(self):
        scaled_p = self.scaled_pressure()
        scaled_t =  self.scaled_temperature()
        c00, c10, c20, c30, c01, c11, c21 = self.get_pressure_calib_coeffs()
        comp_p = (c00 + scaled_p * (c10 + scaled_p * (c20 + scaled_p * c30))+ scaled_t * (c01 + scaled_p * (c11 + scaled_p * c21)))
        return comp_p
     
    def read_altitude(self, set_pressure):
        pressure = self.comp_pressure()
        altitude = (set_pressure-pressure)/11.3
        return altitude

    def baro_test(self):
        
        pass
   
#BARO TEST

baro = Barometer(i2cbus=1, scl_pin=19, sda_pin=18)
baro.baro_conf()
set_pressure = 0
for i in range(0,10):
    set_pressure += baro.comp_pressure()   
set_pressure /= 10
while True:
    altitude = baro.read_altitude(set_pressure)
    pressure = baro.comp_pressure()
    print(pressure)
    sleep(0.1)
    
    
   # https://github.com/Infineon/RaspberryPi_DPS/blob/master/DPS.py
