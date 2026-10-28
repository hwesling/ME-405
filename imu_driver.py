""" MECHA 15 - Harry """
""" IMU Driver """

from micropython import const
from struct      import calcsize, unpack_from
from pyb         import delay
from math        import pi


""" Class with device addresses as constants """
class BNO055:
    DEV_ADDR = const(0x28)

    # Values written to the OP_MODE register
    CONFIG_MODE = const(0x00)
    IMUPLUS_MODE = const(0x08)

    # Values written to the UNIT_SEL register
    UNITS = const(0x00)                                       # degrees, degrees/s, m/s^2

    # Default BNO055 axis map, written into AXIS_MAP_XX reg
    AXIS_MAP_CONFIG = const(0x24)
    AXIS_MAP_SIGN = const(0x00)

    HEADING_SIGN = const(-1)
    YAW_RATE_SIGN = const(1)

    CALIBRATION_START = const(0x55)
    CALIBRATION_LENGTH = const(22)


    """ (Inheritance?) Class with register addresses for device, also constants """
    class reg:
        CHIP_ID = (const(0x00), b"<B")
        PAGE_ID = (const(0x07), b"<B")

        EUL_HEADING_MSB = (const(0x1B), b"<B")
        EUL_HEADING_LSB = (const(0x1A), b"<B")
        EUL_HEADING = (const(0x1A), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"<hhh")
        GYR_DATA_Z = (const(0x18), b"<h")

        OP_MODE = (const(0x3D), b"<B")
        UNIT_SEL = (const(0x3B), b"<B")
        CALIB_STATUS = (const(0x35), b"<B")
        AXIS_MAP_CONFIG_REG = (const(0x41), b"<B")
        AXIS_MAP_SIGN_REG = (const(0x42), b"<B")


    """ Methods of reg class """
    def __init__(self, i2c):
        self._i2c = i2c
        self._buf = bytearray((0 for n in range(22)))


    def read_reg(self, reg):
        # Determine number of bytes to read
        length = calcsize(reg[1])

        # Create a memoryview object of the right size
        buf = memoryview(self._buf)[:length]

        # Read from the I2C bus into the memoryview
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0])

        # Unpack the bytes into a tuple
        return unpack_from(reg[1], buf)


    def euler(self):
        head, roll, pitch = self.read_reg(BNO055.reg.EUL_DATA_ALL)
        return (head / 16, roll / 16, pitch / 16)


    def write(self, register, value):
        # Writes 1 byte into BNO055 register

        self._i2c.mem_write(value, self.DEV_ADDR, register)


    def chip_id(self):
        # Returns BNO055 chip ID byte

        return self.read_reg(BNO055.reg.CHIP_ID)[0]


    def heading_rad(self):
        # Heading in rads

        heading = self.read_reg(BNO055.reg.EUL_HEADING)[0]
        return self.HEADING_SIGN * heading / 16 * pi / 180


    def yaw_rate_rad_s(self):
        # gyro angular rate in rad/s

        yaw_rate = self.read_reg(BNO055.reg.GYR_DATA_Z)[0]
        return self.YAW_RATE_SIGN * yaw_rate / 16 * pi / 180


    def set_mode(self, mode):
        # Sets operation mode for device

        self.write(BNO055.reg.OP_MODE[0], mode)

        if mode == self.CONFIG_MODE:
            delay(19)
        else:
            delay(7)


    def configure(self):
        # Sets configuration for imu (accel, gyro, NOTmag), units, and axip map

        self.set_mode(BNO055.CONFIG_MODE)

        self.write(BNO055.reg.PAGE_ID[0], 0x00)
        self.write(BNO055.reg.UNIT_SEL[0], BNO055.UNITS)
        self.write(BNO055.reg.AXIS_MAP_CONFIG_REG[0], BNO055.AXIS_MAP_CONFIG)
        self.write(BNO055.reg.AXIS_MAP_SIGN_REG[0], BNO055.AXIS_MAP_SIGN)

        self.set_mode(BNO055.IMUPLUS_MODE)


    def calibration_status(self):
        # Returns syst, gyro, & accel calib vals from 0-3

        status = self.read_reg(BNO055.reg.CALIB_STATUS)[0]

        system = (status >> 6) & 0x03
        gyro = (status >> 4) & 0x03
        accel = (status >> 2) & 0x03
        mag = status & 0x03

        return system, gyro, accel, mag


    def read_calibration_profile(self):
        # Reads the complete 22-byte calibration profile in config mode

        self.set_mode(BNO055.CONFIG_MODE)

        try:
            profile = memoryview(self._buf)[:BNO055.CALIBRATION_LENGTH]

            self._i2c.mem_read(
                profile,
                self.DEV_ADDR,
                BNO055.CALIBRATION_START
            )

            return list(profile)

        finally:
            self.set_mode(BNO055.IMUPLUS_MODE)


    def restore_calibration_profile(self, profile):
        # Restores valid calibration profile before scheduling

        if len(profile) != BNO055.CALIBRATION_LENGTH:
            raise ValueError("IMU calibration profile must contain 22 bytes")

        for value in profile:
            if type(value) is not int or not 0 <= value <= 255:
                raise ValueError("IMU calibration profile contains an invalid byte")

        self.set_mode(BNO055.CONFIG_MODE)

        try:
            self.write(BNO055.reg.PAGE_ID[0], 0x00)
            self.write(BNO055.reg.UNIT_SEL[0], BNO055.UNITS)
            self.write(
                BNO055.reg.AXIS_MAP_CONFIG_REG[0],
                BNO055.AXIS_MAP_CONFIG
            )
            self.write(
                BNO055.reg.AXIS_MAP_SIGN_REG[0],
                BNO055.AXIS_MAP_SIGN
            )

            self._i2c.mem_write(
                bytearray(profile),
                self.DEV_ADDR,
                BNO055.CALIBRATION_START
            )

        finally:
            self.set_mode(BNO055.IMUPLUS_MODE)
