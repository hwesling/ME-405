import json


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