"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Github: https://github.com/Tijndagamer/mpu6050.git
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer
"""

import smbus, pigpio

class MPU6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    DLPF_CONFIG = 0x1A
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    SMPLRT_DIV = 0x19

    def __init__(self, address, vio, bus=1):
        """
        Initialize the MPU6050 class with the given parameters.
        :param pi:              (pigpio.pi object) pigpio raspberry pi reference variable
        :param address:         (hex int) MPU6050 I2C address
        :param vio:             (int) VIO pin number (must be high for the device to be visible)
        :param bus:             (int) I2C bus number
        """
        pi = pigpio.pi()
        pi.set_mode(vio, pigpio.OUTPUT)
        pi.write(vio, 1)

        # Store I2C Device Address
        self.address = address
        self.bus = smbus.SMBus(bus)

        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        # Set the filter configuration
        data = self.bus.read_byte_data(self.address, self.DLPF_CONFIG)
        data = (data >> 3 << 3) | 0x02
        self.bus.write_byte_data(self.address, self.DLPF_CONFIG, data)

        # Set Gyro Sample Rate
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 0x00)

        # Set Sensor Ranges and Scales
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        self.set_accel_range(self.ACCEL_RANGE_2G)

        # sensor biases

        self.calibrate([-1.1766396946564714,
                        -0.47155038167939273,
                        -0.46326488549618855,
                        0.077993408203125003,
                        0.017830322265625,
                        -0.081621337890625])

    # I2C communication methods
    def read_vector(self, address, count=3):
        """
        Read count number of 16-bit signed values starting from the provided
        address. Returns a tuple of the values that were read.
        :param address:     (hex int) starting register address
        :param count:       (int) Number of words (2 bytes each) to read
        """

        data = bytearray(self.bus.read_i2c_block_data(self.address, address, count*2))
        result = [0]*count
        for i in range(count):
            result[i] = ((data[i*2] << 8) | data[i*2+1]) & 0xFFFF
            if result[i] > 32767:
                result[i] -= 65536
        return result


    # MPU-6050 Methods
    def get_temp(self):
        """
        Reads the temperature from the onboard temperature sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        :return actual_temp: device temperature in degrees Celcius
        """
        raw_temp = self.read_vector(self.TEMP_OUT0, count=1)
        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53
        return actual_temp

    def set_accel_range(self, accel_range):
        """
        Sets the range of the accelerometer.
        :param accel_range:     (hex value) The range to set the accelerometer to. Using a
                                pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

        if accel_range == self.ACCEL_RANGE_2G:
            self.accel_scale = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            self.accel_scale  = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            self.accel_scale  = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            self.accel_scale  = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            self.accel_scale  = self.ACCEL_SCALE_MODIFIER_2G

    def read_accel_range(self, raw = False):
        """
        Reads the range the accelerometer is set to. If raw is True, it will
        return the raw value from the ACCEL_CONFIG register. If raw is False,
        it will return an integer: -1, 2, 4, 8 or 16. When it returns -1
        something went wrong.
        :param      raw             (bool)
        :return     accel_range     (int) range in g's
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """
        Gets and returns the X, Y and Z values from the accelerometer.
        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        :param      g:          (bool) unit selector
        :return     accel_data: (tuple) x,y,z accelerometer data in selected unit.
        """
        x, y, z = self.read_vector(self.ACCEL_XOUT0, 3)

        x = x / self.accel_scale - self.AxBias
        y = y / self.accel_scale - self.AyBias
        z = z / self.accel_scale - self.AzBias

        if g is True:
            return (x, y, z)
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return (x, y, z)

    def set_gyro_range(self, gyro_range):
        """
        Sets the range of the gyroscope to range.
        :param  gyro_range: The range to set the gyroscope to. Using a pre-defined
                            range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

        if gyro_range == self.GYRO_RANGE_250DEG:
            self.gyro_scale = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            self.gyro_scale = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            self.gyro_scale = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            self.gyro_scale = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            self.gyro_scale = self.GYRO_SCALE_MODIFIER_250DEG

    def read_gyro_range(self, raw = False):
        """
        Reads the range the gyroscope is set to.
        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        :param      raw:            (bool)
        :return     gyro_range:     (int) range in deg/s
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """
        Gets and returns the X, Y and Z values from the gyroscope.
        :return     gyro_data:  (tuple) x,y,z gyroscope data in deg/s.
        """

        x, y, z = self.read_vector(self.GYRO_XOUT0)

        x = x / self.gyro_scale - self.GxBias
        y = y / self.gyro_scale - self.GyBias
        z = z / self.gyro_scale - self.GzBias

        return (x, y, z)

    def get_all_data(self):
        """
        Reads and returns all the available data.
        :return gyro_data, temperature, accel_data (see other get functions for details)
        """
        ax, ay, az, temp, gx, gy, gz = self.read_vector(self.ACCEL_XOUT0,7)

        gx = gx / self.gyro_scale - self.GxBias
        gy = gy / self.gyro_scale - self.GyBias
        gz = gz / self.gyro_scale - self.GzBias
        ax = ax / self.accel_scale - self.AxBias
        ay = ay / self.accel_scale - self.AyBias
        az = az / self.accel_scale - self.AzBias

        return (gx, gy, gz, temp, ax, ay, az)

    def calibrate(self, biases):
        """
        Sets the gyroscope and accelerometer biases (raw offsets)
        :param      biases: (vector) sensor offsets
        """
        self.GxBias = biases[0]
        self.GyBias = biases[1]
        self.GzBias = biases[2]
        self.AxBias = biases[3]
        self.AyBias = biases[4]
        self.AzBias = biases[5]

if __name__ == "__main__":
    mpu = MPU6050(0x68)
    print(mpu.get_temp())
    accel_data = mpu.get_accel_data()
    print(accel_data[0])
    print(accel_data[1])
    print(accel_data[2])
    gyro_data = mpu.get_gyro_data()
    print(gyro_data[0])
    print(gyro_data[1])
    print(gyro_data[2])
