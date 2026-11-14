"""Save a Romi line-following centroid dataset and plot it.

The Romi must print one numeric centroid value (in mm) each time the line task
runs. All other serial messages are ignored. This script configures the speed
and line gain, starts line following, records for LOG_DURATION_S, sends X to
stop safely, and saves a CSV plus a centroid plot.
"""

from datetime import datetime
from pathlib import Path
from time import monotonic

from matplotlib import pyplot
from serial import Serial, SerialException


# Change these four settings for the final demonstration.
SERIAL_PORT = "COM3"
BAUDRATE = 115_200
BASE_SPEED_RAD_S = 5.0
LINE_KP_RAD_S_PER_MM = 0.3
LOG_DURATION_S = 25.0

OUTPUT_DIR = Path(r"C:\Users\rford\Documents\Lab6Me4305\New Wednesday")
SERIAL_TIMEOUT = 0.10
PROMPT_TIMEOUT_S = 5.0
NUMERIC_LINE_ENDING = "\r\n"


def send_command(ser, command):
    """Send one menu character."""
    ser.write(command.encode("utf-8"))
    ser.flush()


def wait_for_text(ser, expected_text):
    """Wait for a menu prompt/acknowledgement while showing Romi messages."""
    started = monotonic()
    while monotonic() - started < PROMPT_TIMEOUT_S:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        print(f"Romi: {line}")
        if expected_text in line:
            return
    raise TimeoutError(f"Did not receive: {expected_text}")


def set_number(ser, command, value, prompt, acknowledgement):
    """Use the existing V and C menu entries to set one number."""
    send_command(ser, command)
    wait_for_text(ser, prompt)
    ser.write(f"{value}{NUMERIC_LINE_ENDING}".encode("utf-8"))
    ser.flush()
    wait_for_text(ser, acknowledgement)


def unique_path(path):
    """Avoid overwriting a previous run."""
    if not path.exists():
        return path
    for number in range(1, 1000):
        candidate = path.with_name(f"{path.stem}_{number}{path.suffix}")
        if not candidate.exists():
            return candidate
    raise FileExistsError("Could not find an unused output filename.")


def collect_centroids(ser):
    """Start line following and collect centroid values for the chosen time."""
    print("Starting line following and collecting centroid values...")
    send_command(ser, "f")
    start = monotonic()
    data = []

    while monotonic() - start < LOG_DURATION_S:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        try:
            # TaskLine prints only centroid_mm, so the PC supplies elapsed time.
            centroid_mm = float(line)
        except ValueError:
            print(f"Romi: {line}")
            continue
        data.append((monotonic() - start, centroid_mm))

    # X is the firmware's safe line-following stop command; it returns to menu.
    send_command(ser, "x")
    return data


def save_csv(filename, data):
    with open(filename, "w", encoding="utf-8", newline="") as file:
        file.write("time_s,centroid_mm\n")
        for time_s, centroid_mm in data:
            file.write(f"{time_s:.6f},{centroid_mm:.6f}\n")


def save_plot(filename, data):
    time_s, centroid_mm = zip(*data)
    figure, axis = pyplot.subplots(figsize=(7.2, 4.2))
    axis.plot(time_s, centroid_mm, color="tab:blue", linewidth=1.2,
              label="Measured centroid")
    axis.axhline(0, color="black", linewidth=0.8, linestyle="--",
                 label="Zero-centroid reference")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Line Centroid [mm]")
    axis.set_title(
        "Romi Line-Following Centroid\n"
        f"Base speed = {BASE_SPEED_RAD_S} rad/s, "
        f"line gain = {LINE_KP_RAD_S_PER_MM} (rad/s)/mm, "
        f"log duration = {LOG_DURATION_S} s"
    )
    axis.grid(True)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(filename, dpi=200)
    pyplot.close(figure)


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    try:
        with Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
            ser.reset_input_buffer()
            set_number(ser, "v", BASE_SPEED_RAD_S,
                       "Enter Velocity as Omega", "Omega set to")
            set_number(ser, "c", LINE_KP_RAD_S_PER_MM,
                       "enter line following gain", "Line gain set to")
            data = collect_centroids(ser)
    except SerialException as error:
        raise SystemExit(f"Serial-port error: {error}") from error

    if not data:
        raise SystemExit("No centroid values received. Add print(centroid_mm) "
                         "to TaskLine before its yield.")

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_file = unique_path(OUTPUT_DIR / f"{timestamp}_centroid.csv")
    plot_file = unique_path(OUTPUT_DIR / f"{timestamp}_centroid.png")
    save_csv(csv_file, data)
    save_plot(plot_file, data)
    print(f"Saved {len(data)} centroid rows to {csv_file}")
    print(f"Saved centroid plot to {plot_file}")


if __name__ == "__main__":
    main()
