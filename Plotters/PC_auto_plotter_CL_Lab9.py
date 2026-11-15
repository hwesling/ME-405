"""Run one simultaneous Romi motor test and save left/right CSVs and plots.

This script uses TaskUser's G command. TaskUser must be the paired version
that prints a normal left data block and a normal right data block once both
motors have completed their simultaneous test.
"""

from datetime import datetime
from pathlib import Path
from time import monotonic

from matplotlib import pyplot
from serial import Serial, SerialException


# Change this to the USB virtual COM port assigned to the Romi.
SERIAL_PORT = "COM4"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.25

# Current default values from main.py.
SETPOINT_RAD_S = 13.0
KP = 0.8
KI = 0.8

# Current TaskUser commands.
VELOCITY_COMMAND = "v"
GAIN_MENU_COMMAND = "k"
KP_COMMAND = "p"
KI_COMMAND = "i"
BOTH_MOTORS_COMMAND = "g"
NUMERIC_LINE_ENDING = "\r\n"

START_MARKER = "STARTING CLOSED-LOOP RESPONSE"
DATA_COMPLETE_MARKER = "TEST_COMPLETE"
BOTH_COMPLETE_MARKER = "BOTH_MOTOR_TEST_COMPLETE"
ABORT_MARKERS = ("TEST_ABORTED", "# EXIT")
PROMPT_TIMEOUT = 5.0
DATA_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 5.0
OUTPUT_DIR = Path(__file__).resolve().parent / "romi_closed_loop_results"


def send_command(ser, command):
    ser.write(command.encode("utf-8"))
    ser.flush()


def read_until(ser, expected_text, timeout, timeout_message):
    """Print serial messages until the expected firmware text arrives."""
    last_message_time = monotonic()
    while monotonic() - last_message_time < timeout:
        raw_line = ser.readline()
        if not raw_line:
            continue
        last_message_time = monotonic()
        line = raw_line.decode("utf-8", errors="replace").strip()
        print(f"Serial message: {line}")
        if expected_text in line:
            return
        if any(marker in line for marker in ABORT_MARKERS):
            raise RuntimeError(f"Romi stopped the test: {line}")
        if "Invalid input" in line or "outside of allowable range" in line:
            raise ValueError(f"Romi rejected a setting: {line}")
    raise TimeoutError(timeout_message)


def send_numeric_setting(ser, command, value, prompt, acknowledgement):
    send_command(ser, command)
    read_until(ser, prompt, PROMPT_TIMEOUT,
               f"No prompt received for '{command}'.")
    ser.write(f"{value}{NUMERIC_LINE_ENDING}".encode("utf-8"))
    ser.flush()
    read_until(ser, acknowledgement, PROMPT_TIMEOUT,
               f"No acknowledgement received for '{command}'.")


def configure_test(ser):
    """Set omega, Kp, and Ki using TaskUser's current menu flow."""
    send_numeric_setting(ser, VELOCITY_COMMAND, SETPOINT_RAD_S,
                         "Enter Velocity as Omega", "Omega set to")

    # TaskUser returns to the main menu after each gain value.
    send_command(ser, GAIN_MENU_COMMAND)
    read_until(ser, "ROMI GAIN MENU", PROMPT_TIMEOUT,
               "Gain menu did not appear before setting Kp.")
    send_numeric_setting(ser, KP_COMMAND, KP,
                         "Enter Proportional Gain Kp", "Proportional Gain Kp set to")

    send_command(ser, GAIN_MENU_COMMAND)
    read_until(ser, "ROMI GAIN MENU", PROMPT_TIMEOUT,
               "Gain menu did not appear before setting Ki.")
    send_numeric_setting(ser, KI_COMMAND, KI,
                         "Enter Integral Gain Ki", "Integral Gain Ki set to")


def collect_dataset(ser):
    """Read one four-column TaskUser response block."""
    metadata = []
    headers = None
    columns = [[], [], [], []]
    in_dataset = False
    last_line_time = monotonic()

    while True:
        raw_line = ser.readline()
        if not raw_line:
            timeout = BETWEEN_LINES_TIMEOUT if in_dataset else DATA_TIMEOUT
            if monotonic() - last_line_time >= timeout:
                raise TimeoutError("Timed out while receiving a motor response.")
            continue

        last_line_time = monotonic()
        line = raw_line.decode("utf-8", errors="replace").strip()

        if START_MARKER in line:
            in_dataset = True
            metadata.append(line)
            continue
        if any(marker in line for marker in ABORT_MARKERS):
            raise RuntimeError(f"Romi stopped the test: {line}")
        if in_dataset and DATA_COMPLETE_MARKER in line:
            break
        if not in_dataset:
            print(f"Serial message: {line}")
            continue

        fields = [field.strip() for field in line.split("#", 1)[0].split(",")]
        if headers is None:
            if len(fields) == 4 and fields[0].lower().startswith("time"):
                headers = fields
            else:
                metadata.append(line)
            continue
        if len(fields) != 4:
            continue
        try:
            values = [float(field) for field in fields]
        except ValueError:
            print(f"Ignoring nonnumeric data line: {line}")
            continue
        for column, value in zip(columns, values):
            column.append(value)

    # TaskUser may print unused preallocated array entries as 0,0,0,0.
    # Remove only those all-zero rows from the END of the dataset.
    while columns[0]:
        last_row = [column[-1] for column in columns]

        if all(abs(value) < 1e-12 for value in last_row):
            for column in columns:
                column.pop()
        else:
            break

    if headers is None or not columns[0]:
        raise ValueError("Response contains no valid data rows.")

    return metadata, headers, columns


def safe_part(value):
    return str(value).replace("-", "neg_").replace(".", "p")


def unique_path(path):
    if not path.exists():
        return path
    for suffix in range(1, 1000):
        candidate = path.with_name(f"{path.stem}_{suffix}{path.suffix}")
        if not candidate.exists():
            return candidate
    raise FileExistsError(f"Could not find a unique name for {path}")


def save_csv(path, metadata, headers, columns, motor, stamp):
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        csv_file.write("# Romi simultaneous G-command response\n")
        csv_file.write(f"# timestamp,{stamp}\n# motor,{motor}\n")
        csv_file.write(f"# setpoint_rad_s,{SETPOINT_RAD_S}\n# kp,{KP}\n# ki,{KI}\n")
        for line in metadata:
            csv_file.write(f"# {line}\n")
        csv_file.write(",".join(headers) + "\n")
        for row in zip(*columns):
            csv_file.write(",".join(f"{value:.12g}" for value in row) + "\n")


def save_plot(path, columns, motor):
    time_s = columns[0]
    figure, axes = pyplot.subplots(2, 1, figsize=(7.2, 6.0), sharex=True)
    axes[0].plot(time_s, columns[1], label="Setpoint", color="tab:green")
    axes[0].plot(time_s, columns[2], label="Measured velocity", color="tab:blue")
    axes[1].plot(time_s, columns[3], label="Voltage command", color="tab:orange")
    figure.suptitle(f"{motor.title()} Motor, Simultaneous G Test\n"
                     f"PI: setpoint = {SETPOINT_RAD_S} rad/s, Kp = {KP}, Ki = {KI}")
    axes[0].set_ylabel("Velocity [rad/s]")
    axes[1].set_ylabel("Voltage [V]")
    axes[1].set_xlabel("Time [s]")
    for axis in axes:
        axis.grid(True)
        axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=200)
    pyplot.close(figure)


def save_response(metadata, headers, columns, motor, stamp):
    stem = (f"{stamp}_both_{motor}_sp{safe_part(SETPOINT_RAD_S)}_"
            f"Kp{safe_part(KP)}_Ki{safe_part(KI)}")
    csv_path = unique_path(OUTPUT_DIR / f"{stem}.csv")
    plot_path = unique_path(OUTPUT_DIR / f"{stem}.png")
    save_csv(csv_path, metadata, headers, columns, motor, stamp)
    save_plot(plot_path, columns, motor)
    print(f"Saved {len(columns[0])} rows to {csv_path}")
    print(f"Saved plot to {plot_path}")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    try:
        with Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
            ser.reset_input_buffer()
            print(f"Setting omega={SETPOINT_RAD_S}, Kp={KP}, Ki={KI}")
            configure_test(ser)

            print("Starting both motors with the G command")
            send_command(ser, BOTH_MOTORS_COMMAND)
            stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            # TaskUser prints these two response blocks only after BOTH motors
            # have run at the same time and reached their sample limits.
            left_response = collect_dataset(ser)
            right_response = collect_dataset(ser)
            read_until(ser, BOTH_COMPLETE_MARKER, BETWEEN_LINES_TIMEOUT,
                       "Both-motor completion marker was not received.")

            save_response(*left_response, "left", stamp)
            save_response(*right_response, "right", stamp)
            print("Both-motor test and both plots completed.")
    except KeyboardInterrupt:
        print("Interrupted; serial port closed.")
    except SerialException as error:
        raise SystemExit(f"Serial-port error: {error}") from error


if __name__ == "__main__":
    main()
