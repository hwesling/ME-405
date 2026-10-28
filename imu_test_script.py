""" MECHA 15 - ROMI """
""" Romi IMU hand crancked REPL test script """
""" AKA 'Pls dont bite us in the ass' """

""" PASTE THIS INTO REPL LINE-BY-LINE TO RUN """
#import imu_test_script
#imu_test_script.main()


from math import pi
from pyb import I2C, delay
from imu_driver import BNO055
from imu_calibration import load_calibration


IMU_CALIBRATION_FILE = "imu_calibration.json"


def main():
    i2c = I2C(2, I2C.CONTROLLER, baudrate = 400_000)

    if BNO055.DEV_ADDR not in i2c.scan():
        raise RuntimeError("BNO055 not found on I2C bus")

    imu = BNO055(i2c)

    if imu.chip_id() != 0xA0:
        raise RuntimeError("Unexpected BNO055 chip ID")

    imu.configure()

    try:
        imu_calibration = load_calibration(
            IMU_CALIBRATION_FILE,
            BNO055.IMUPLUS_MODE,
            BNO055.UNITS,
            BNO055.AXIS_MAP_CONFIG,
            BNO055.AXIS_MAP_SIGN
        )

        imu.restore_calibration_profile(imu_calibration["profile"])
        print("Saved IMU calibration loaded.")

    except (OSError, ValueError, KeyError, TypeError):
        print("No valid saved IMU calibration; run main.py to calibrate first.")
        return

    heading_start = imu.heading_rad()

    print("Press Ctrl-C to stop.")
    print("Heading [rad], Relative heading [rad], Yaw rate [rad/s], System, Gyro, Accel, Mag")

    try:
        while True:
            heading = imu.heading_rad()
            relative_heading = (heading - heading_start + pi) % (2 * pi) - pi
            yaw_rate = imu.yaw_rate_rad_s()
            system, gyro, accel, mag = imu.calibration_status()

            print(
                "{:.1f},           {:.1f},          {:.1f}".format(
                    heading,
                    yaw_rate,
                    relative_heading,
                )
            )

            delay(200)

    except KeyboardInterrupt:
        print("Live IMU readout stopped.")


if __name__ == "__main__":
    main()
