import json
from pyb import delay


def validate_calibration(calibration, mode, units, axis_map_config, axis_map_sign):
    profile = calibration["profile"]

    if calibration["mode"] != mode:
        raise ValueError("Calibration does not use the current IMU mode")

    if calibration["units"] != units:
        raise ValueError("Calibration does not use the current IMU units")

    if calibration["axis_map_config"] != axis_map_config:
        raise ValueError("Calibration does not match the IMU axis map")

    if calibration["axis_map_sign"] != axis_map_sign:
        raise ValueError("Calibration does not match the IMU axis signs")

    if len(profile) != 22:
        raise ValueError("IMU profile must contain 22 bytes")

    for value in profile:
        if type(value) is not int or not 0 <= value <= 255:
            raise ValueError("IMU profile contains an invalid byte")

    return calibration


def save_calibration(filename, calibration, mode, units, axis_map_config, axis_map_sign):
    validate_calibration(
        calibration,
        mode,
        units,
        axis_map_config,
        axis_map_sign
    )

    with open(filename, "w") as file:
        json.dump(calibration, file)


def load_calibration(filename, mode, units, axis_map_config, axis_map_sign):
    with open(filename, "r") as file:
        calibration = json.load(file)

    validate_calibration(
        calibration,
        mode,
        units,
        axis_map_config,
        axis_map_sign
    )

    return calibration


def calibrate_imu(imu, filename, mode, units, axis_map_config, axis_map_sign):
    """Guide a fresh BNO055 IMUPLUS calibration before scheduling."""

    print("Keep Romi still for gyro calibration.")
    input("Place Romi level and still, then press Enter: ")
    delay(3000)

    for orientation in (
        "+X up", "-X up", "+Y up", "-Y up", "+Z up", "-Z up"
    ):
        input("Place the IMU with " + orientation + ", then press Enter: ")
        delay(2500)

    print("Waiting for gyro and accelerometer calibration to reach 3.")

    last_status = None

    while True:
        system, gyro, accel, mag = imu.calibration_status()
        status = (system, gyro, accel, mag)

        if status != last_status:
            print(
                "IMU calibration - system: {}, gyro: {}, accel: {}, mag: {}".format(
                    system,
                    gyro,
                    accel,
                    mag
                )
            )

            last_status = status

        if gyro == 3 and accel == 3:
            calibration = {
                "mode": mode,
                "units": units,
                "axis_map_config": axis_map_config,
                "axis_map_sign": axis_map_sign,
                "profile": imu.read_calibration_profile(),
            }

            save_calibration(
                filename,
                calibration,
                mode,
                units,
                axis_map_config,
                axis_map_sign
            )

            print("IMU calibration saved.")
            return

        delay(250)
