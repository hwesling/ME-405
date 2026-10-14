""" MECHA 15 - Harry """
""" Auto-plotting script for Romi robot. Sends closed-loop tuning commands before
   each motor command then collects one left response and one right response. Saves
   responses as CSV, then creates a plot. """

from datetime   import datetime
from time       import monotonic

from matplotlib import pyplot
from serial     import Serial, SerialException
from pathlib    import Path

# Serial-port settings. Change SERIAL_PORT to the VCP assigned by the computer.
SERIAL_PORT = "COM4"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.25

# Sequence runs without additional user input after the script starts.
# Each tuple is: motor name, velocity setpoint [ticks/s], Kp [V/(ticks/s)], Ki [V/tick]
PRETEST = (
    ("left", 5000.0, 0.001, 0.0),
    ("right", 5000.0, 0.001, 0.0),
)
CONTROL_STRUCTURE = "PI"
NUMERIC_LINE_ENDING = "\r\n"

# Single-character commands used by the firmware menu.
SETPOINT_COMMAND = "o"
KP_COMMAND = "p"
KI_COMMAND = "i"
LEFT_COMMAND = "l"
RIGHT_COMMAND = "r"

# The firmware should print these markers on their own lines.
START_MARKER = "STARTING CLOSED-LOOP RESPONSE"
END_MARKER = "CLOSED-LOOP RESPONSE COMPLETE"
ABORT_MARKER = "CLOSED-LOOP RESPONSE ABORTED"
FAIL_MARKER = "CLOSED-LOOP RESPONSE FAILED"

# Data rows:
HEADERS = ["Time [us]",
           "Setpoint [ticks/s]",
           "Measured Velocity [ticks/s]",
           "Voltage Command [V]"]

# Plot labels used for the saved deliverable figures.
TIME_LABEL = "Time [s]"
SETPOINT_LABEL = "Setpoint [ticks/s]"
VELOCITY_LABEL = "Measured Velocity [ticks/s]"
VOLTAGE_LABEL = "Voltage Command [V]"

# Stop waiting and report an error instead of hanging forever if the firmware
# does not respond or stops transmitting partway through a dataset.
#
# monotonic() returns a steadily increasing time in seconds. Its starting value
# is arbitrary, so it is not used as a date or time of day. Subtracting an old
# reading from a new reading gives the elapsed time without being affected if
# the computer's clock is adjusted while the program is running.
FIRST_RESPONSE_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 5.0
ACK_TIMEOUT = 5.0

OUTPUT_DIR = Path(r"C:\Users\hwesl\OneDrive - Cal Poly\ME 405 LAB\Lab 5\romi_closed_loop_results")


def cleaned_fields(line):
    """Return stripped comma-separated fields after removing a comment."""
    content = line.split("#", 1)[0].strip()
    if not content:
        return []

    fields = []
    for field in content.split(","):
        fields.append(field.strip())

    return fields


def safe_filename_part(value):
    """Return a filesystem-safe string for the motor name or numeric setting."""
    text = str(value).strip().replace("-", "neg_").replace(".", "p")
    safe_text = []

    for char in text:
        if char.isalnum() or char == "_":
            safe_text.append(char)
        else:
            safe_text.append("_")

    return "".join(safe_text)


def unique_filename(filename):
    """Return a filename that does not silently overwrite an older result."""
    path = Path(filename)
    if not path.exists():
        return path

    for index in range(1, 1000):
        candidate = path.with_name(f"{path.stem}_{index}{path.suffix}")
        if not candidate.exists():
            return candidate

    raise FileExistsError(f"Could not create a unique filename for {filename}")


def send_command(ser, command):
    """Send a single-character command without a line ending."""
    ser.write(command.encode("utf-8"))
    ser.flush()


def wait_for_text(ser, expected_text, timeout, error_message):
    """Wait for a line containing expected_text and print other serial text."""
    waiting_since = monotonic()

    while True:
        raw_line = ser.readline()

        if not raw_line:
            if monotonic() - waiting_since >= timeout:
                raise TimeoutError(error_message)
            continue

        waiting_since = monotonic()
        line = raw_line.decode("utf-8", errors="replace").strip()
        print(f"Serial message: {line}")

        if expected_text in line:
            return line

        if "Invalid input" in line:
            raise ValueError("The firmware rejected the numeric input.")


def send_numeric_setting(ser, command, value, prompt_text, ack_text):
    """Send one numeric menu command and wait for the prompt and acknowledgement."""
    send_command(ser, command)

    wait_for_text(ser, prompt_text, ACK_TIMEOUT,
                  f"No prompt was received for command {command}.")

    ser.write(f"{value}{NUMERIC_LINE_ENDING}".encode("utf-8"))
    ser.flush()

    wait_for_text(ser, ack_text, ACK_TIMEOUT,
                  f"No acknowledgement was received for command {command}.")


def configure_test(ser, setpoint, Kp, Ki):
    """Send the closed-loop setpoint and gain values to the firmware."""
    print(f"Setting velocity setpoint to {setpoint} ticks/s")
    send_numeric_setting(ser, SETPOINT_COMMAND, setpoint,
                         "Enter Velocity Setpoint",
                         "Velocity setpoint set to")

    print(f"Setting Kp to {Kp} V/(ticks/s)")
    send_numeric_setting(ser, KP_COMMAND, Kp,
                         "Enter Proportional Gain",
                         "Proportional Gain Kp set to")

    print(f"Setting Ki to {Ki} V/tick")
    send_numeric_setting(ser, KI_COMMAND, Ki,
                         "Enter Integral Gain",
                         "Integral Gain Ki set to")


def collect_dataset(ser):
    """Read one numeric dataset between START_MARKER and a completion marker.

    Header text after START_MARKER is preserved separately from the CSV column
    header. Numeric rows with the expected four fields are collected. Other
    status text before START_MARKER is displayed and ignored.
    """
    headers = None
    header_lines = []
    columns = []
    for header in HEADERS:
        columns.append([])

    in_dataset = False
    serial_line_number = 0
    completion_status = None
    waiting_since = monotonic()

    while True:
        raw_line = ser.readline()

        # readline() returns b"" when its short serial timeout expires. Keep
        # polling until the longer application timeout has also expired.
        if not raw_line:
            timeout = (FIRST_RESPONSE_TIMEOUT if not in_dataset
                       else BETWEEN_LINES_TIMEOUT)
            if monotonic() - waiting_since >= timeout:
                if not in_dataset:
                    raise TimeoutError("No dataset start marker was received.")
                raise TimeoutError("Serial data stopped before a completion marker.")
            continue

        waiting_since = monotonic()
        serial_line_number += 1
        line = raw_line.decode("utf-8", errors="replace").strip()

        if line == START_MARKER:
            in_dataset = True
            header_lines.append(line)
            print("Dataset start marker received")
            continue

        if line in {END_MARKER, ABORT_MARKER, FAIL_MARKER}:
            completion_status = line
            break

        if not in_dataset:
            print(f"Serial message: {line}")
            continue

        fields = cleaned_fields(line)

        if headers is None:
            if len(fields) == len(HEADERS) and fields[0].lower().startswith("time"):
                headers = fields
            else:
                header_lines.append(line)
            continue

        if len(fields) != len(headers):
            continue

        try:
            values = []
            for field in fields:
                values.append(float(field))
        except ValueError:
            print(f"Rejected serial line {serial_line_number}: "
                  "one or more fields are not numeric")
            continue

        for column, value in zip(columns, values):
            column.append(value)

    if headers is None:
        raise ValueError("The dataset did not contain a CSV column header.")

    if not columns[0]:
        raise ValueError("The dataset did not contain any valid numeric rows.")

    if completion_status != END_MARKER:
        raise RuntimeError(f"Firmware reported incomplete collection: "
                           f"{completion_status}")

    return header_lines, headers, columns, completion_status


def save_csv(filename, header_lines, headers, columns, motor_name, setpoint,
             Kp, Ki, run_timestamp, completion_status):
    """Save the collected metadata, firmware header, and numeric data."""
    with open(filename, "w", encoding="utf-8", newline="") as csv_file:
        csv_file.write("# Romi closed-loop response\n")
        csv_file.write(f"# timestamp,{run_timestamp}\n")
        csv_file.write(f"# motor,{motor_name}\n")
        csv_file.write(f"# control_structure,{CONTROL_STRUCTURE}\n")
        csv_file.write(f"# setpoint_ticks_per_second,{setpoint}\n")
        csv_file.write(f"# Kp_V_per_ticks_per_second,{Kp}\n")
        csv_file.write(f"# Ki_V_per_tick,{Ki}\n")
        csv_file.write(f"# completion_status,{completion_status}\n")
        csv_file.write("# units,microseconds,ticks_per_second,ticks_per_second,volts\n")
        csv_file.write("# firmware_header_begin\n")
        for line in header_lines:
            csv_file.write(f"# {line}\n")
        csv_file.write("# firmware_header_end\n")
        csv_file.write("# begin_data\n")
        csv_file.write(",".join(headers) + "\n")
        for row in zip(*columns):
            formatted_values = []
            for value in row:
                formatted_values.append(f"{value:.12g}")
            csv_file.write(",".join(formatted_values) + "\n")
        csv_file.write("# end_data\n")


def save_plot(filename, headers, columns, motor_name, setpoint, Kp, Ki):
    """Plot setpoint, measured velocity, and limited voltage command."""
    time_seconds = []
    for time_us in columns[0]:
        time_seconds.append(time_us / 1_000_000)

    figure, axes = pyplot.subplots(2, 1, figsize=(7.2, 6.0), sharex=True)
    velocity_axes = axes[0]
    voltage_axes = axes[1]

    velocity_axes.plot(time_seconds, columns[1], label=headers[1],
                       color="tab:green", linewidth=1.5)
    velocity_axes.plot(time_seconds, columns[2], label=headers[2],
                       color="tab:blue", linewidth=1.5)
    voltage_axes.plot(time_seconds, columns[3], label=headers[3],
                      color="tab:orange", linewidth=1.5)

    figure.suptitle(f"{motor_name.title()} Motor Closed-Loop Response\n"
                    f"{CONTROL_STRUCTURE}, setpoint = {setpoint} ticks/s, "
                    f"Kp = {Kp}, Ki = {Ki}, motor period = 10 ms")

    velocity_axes.set_ylabel(VELOCITY_LABEL)
    voltage_axes.set_ylabel(VOLTAGE_LABEL)
    voltage_axes.set_xlabel(TIME_LABEL)

    velocity_axes.grid(True)
    voltage_axes.grid(True)
    velocity_axes.legend(loc="best")
    voltage_axes.legend(loc="best")

    figure.tight_layout()
    figure.savefig(filename, dpi=200)
    pyplot.close(figure)


def main():
    """Configure, collect, save, and plot the configured closed-loop responses."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Opening {SERIAL_PORT} at {BAUDRATE} baud")
    try:
        # The context manager closes the port even if collection raises an
        # exception or the user interrupts the program.
        with Serial(SERIAL_PORT,
                    baudrate=BAUDRATE,
                    timeout=SERIAL_TIMEOUT) as ser:
            print("Discarding any unread serial data")
            ser.reset_input_buffer()

            for motor_name, setpoint, Kp, Ki in PRETEST:
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                motor_text = safe_filename_part(motor_name)
                setpoint_text = safe_filename_part(setpoint)
                Kp_text = safe_filename_part(Kp)
                Ki_text = safe_filename_part(Ki)
                base_name = (f"{timestamp}_{motor_text}_sp{setpoint_text}_"
                             f"Kp{Kp_text}_Ki{Ki_text}")
                data_filename = unique_filename(OUTPUT_DIR / f"{base_name}.csv")
                plot_filename = unique_filename(OUTPUT_DIR / f"{base_name}.png")

                configure_test(ser, setpoint, Kp, Ki)

                print(f"Sending command for {motor_name} motor")
                if motor_name.lower() == "left":
                    send_command(ser, LEFT_COMMAND)
                elif motor_name.lower() == "right":
                    send_command(ser, RIGHT_COMMAND)
                else:
                    raise ValueError(f"Unknown motor name: {motor_name}")

                print("Waiting for dataset")
                header_lines, headers, columns, completion_status = collect_dataset(ser)

                save_csv(data_filename, header_lines, headers, columns,
                         motor_name, setpoint, Kp, Ki, timestamp,
                         completion_status)
                save_plot(plot_filename, headers, columns, motor_name,
                          setpoint, Kp, Ki)

                print(f"Saved {len(columns[0])} valid data rows to {data_filename}")
                print(f"Saved plot to {plot_filename}")

    except KeyboardInterrupt:
        print("\nInterrupted by user. Serial port closed.")
    except SerialException as error:
        raise SystemExit(f"Serial-port error: {error}") from error


if __name__ == "__main__":
    main()
