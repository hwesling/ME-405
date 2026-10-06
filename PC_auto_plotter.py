""" MECHA 15 - Harry """
""" Auto-plotting script for Romi robot. Sends the duty-cycle command before each motor
   command then collects one left response and one right response. Saves responses as CSV, then creates a plot. """

from datetime   import datetime
from time       import monotonic

from matplotlib import pyplot
from serial     import Serial, SerialException
from pathlib    import Path

# Serial-port settings. Change SERIAL_PORT to the VCP assigned by the computer.
SERIAL_PORT = "COM4"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.25

# This example requests one left-motor dataset using the required Lab 0x04 menu.
# Single-character commands are sent as soon as they are chosen; they do not
# need a line ending. Students will extend this into the duty-cycle and
# two-motor command sequence. A multicharacter numeric value does need the line
# ending expected by the firmware so that it knows when entry is complete.

PRETEST = (
    ("left", 33.0),
    ("right", 33.0),
)

NUMERIC_LINE_ENDING = "\r\n"


# The firmware should print this marker on its own line after the last data row.
# Using an explicit marker is more reliable than assuming that a quiet serial
# port means the dataset is complete.
START_MARKER = "STARTING OPEN-LOOP RESPONSE"
END_MARKER = "OPEN-LOOP RESPONSE COMPLETE"

# Data rows:
HEADERS = ["Duty Cycle [%]", 
           "time [us]", 
           "Position [ticks]", 
           "Velocity [ticks/s]"]

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

# The run name becomes part of each output filename. Students may replace this
# with a motor name, duty cycle, or another useful test identifier.

OUTPUT_DIR = Path(r"C:\Users\hwesl\OneDrive - Cal Poly\ME 405 LAB\Lab 4\romi_open_loop_results")


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
    """Return a filesystem-safe string for the motor name or duty cycle."""
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


def send_duty_cycle(ser, duty_cycle):
    """Send the duty-cycle command and wait for the firmware acknowledgement."""
    send_command(ser, "d")
    ser.write(f"{duty_cycle}{NUMERIC_LINE_ENDING}".encode("utf-8"))
    ser.flush()

    waiting_since = monotonic()
    while True:
        raw_line = ser.readline()

        if not raw_line:
            if monotonic() - waiting_since >= ACK_TIMEOUT:
                raise TimeoutError("No duty-cycle acknowledgement was received.")
            continue

        waiting_since = monotonic()
        line = raw_line.decode("utf-8", errors="replace").strip()
        print(f"Serial message: {line}")

        if "Value set to" in line:
            return

        if "Invalid input" in line or "Value outside of allowable range" in line:
            raise ValueError(f"The firmware rejected duty cycle {duty_cycle}.")


def collect_dataset(ser):
    """Read one headed numeric dataset, starting at START_MARKER and ending at END_MARKER.

    Status and prompt lines before the CSV header are displayed and ignored.
    After the header is found, malformed rows are reported and skipped rather
    than terminating the whole collection.
    """
    headers = HEADERS
    columns = []
    for header in HEADERS:
        columns.append([])

    in_dataset = False
    serial_line_number = 0

    # Save one reading from the monotonic clock. Later readings are compared
    # with this one to determine how many seconds have elapsed.
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
                    raise TimeoutError("No CSV header was received.")
                raise TimeoutError("Serial data stopped before the end marker.")
            continue

        waiting_since = monotonic()
        serial_line_number += 1
        line = raw_line.decode("utf-8", errors="replace").strip()

        if line == START_MARKER:
            in_dataset = True
            print("Dataset start marker received")
            continue

        if line == END_MARKER:
            break

        if not in_dataset:
            print(f"Serial message: {line}")
            continue

        fields = cleaned_fields(line)
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

    if not columns[0]:
        raise ValueError("The dataset did not contain any valid numeric rows.")

    return headers, columns


def save_csv(filename, headers, columns):
    """Save the collected columns using the received header labels."""
    with open(filename, "w", encoding="utf-8", newline="") as csv_file:
        csv_file.write(",".join(headers) + "\n")
        for row in zip(*columns):
            formatted_values = []
            for value in row:
                formatted_values.append(f"{value:.12g}")
            csv_file.write(",".join(formatted_values) + "\n")


def save_plot(filename, headers, columns):
    """Plot position and velocity against time."""
    time_seconds = []
    for time_us in columns[1]:
        time_seconds.append(time_us / 1_000_000)

    figure, position_axes = pyplot.subplots()
    velocity_axes = position_axes.twinx()

    position_axes.plot(time_seconds, columns[2], label=headers[2],
                       color="tab:blue")
    velocity_axes.plot(time_seconds, columns[3], label=headers[3],
                       color="tab:orange")

    position_axes.set_xlabel("time [s]")
    position_axes.set_ylabel(headers[2], color="tab:blue")
    velocity_axes.set_ylabel(headers[3], color="tab:orange")
    position_axes.tick_params(axis="y", labelcolor="tab:blue")
    velocity_axes.tick_params(axis="y", labelcolor="tab:orange")

    position_axes.grid(True)
    figure.tight_layout()
    figure.savefig(filename, dpi=200)
    pyplot.close(figure)


def main():
    """Collect, save, and plot the configured left and right responses."""
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

            for motor_name, duty_cycle in PRETEST:
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                motor_text = safe_filename_part(motor_name)
                duty_text = safe_filename_part(duty_cycle)
                base_name = f"{timestamp}_{motor_text}_{duty_text}pct"
                data_filename = unique_filename(OUTPUT_DIR / f"{base_name}.csv")
                plot_filename = unique_filename(OUTPUT_DIR / f"{base_name}.png")

                print(f"Setting duty cycle to {duty_cycle}%")
                send_duty_cycle(ser, duty_cycle)

                print(f"Sending command for {motor_name} motor")
                if motor_name.lower() == "left":
                    send_command(ser, "l")
                elif motor_name.lower() == "right":
                    send_command(ser, "r")
                else:
                    raise ValueError(f"Unknown motor name: {motor_name}")

                print("Waiting for dataset")
                headers, columns = collect_dataset(ser)

                save_csv(data_filename, headers, columns)
                save_plot(plot_filename, headers, columns)

                print(f"Saved {len(columns[0])} valid data rows to {data_filename}")
                print(f"Saved plot to {plot_filename}")

    except KeyboardInterrupt:
        print("\nInterrupted by user. Serial port closed.")
    except SerialException as error:
        raise SystemExit(f"Serial-port error: {error}") from error


if __name__ == "__main__":
    main()
