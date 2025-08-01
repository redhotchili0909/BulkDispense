import csv
import matplotlib.pyplot as plt

SENSOR_NAME = "SM9235"
PRESSURE_UNIT = "mmH₂O"
TEMP_UNIT = "°C"

def load_log(filepath):
    timestamps = []
    values = []
    mode = None

    with open(filepath, 'r') as file:
        # Skip comment lines starting with '#'
        while True:
            pos = file.tell()
            line = file.readline()
            if not line.startswith("#"):
                file.seek(pos)
                break

        reader = csv.reader(file)
        headers = next(reader)
        if headers != ["timestamp_ms", "mode", "value"]:
            raise ValueError("Invalid file format. Expected 'timestamp_ms,mode,value'.")

        for row in reader:
            if len(row) != 3:
                continue
            ts, m, val = row
            if mode is None:
                mode = m
            if m != mode:
                continue
            timestamps.append(int(ts) / 1000.0)
            values.append(float(val))

    return timestamps, values, mode

def plot_log(filepath):
    try:
        x, y, mode = load_log(filepath)
    except Exception as e:
        print(f"Error loading file: {e}")
        return

    y_label = ""
    if mode == "pressure":
        y_label = f"Pressure ({PRESSURE_UNIT})"
    elif mode == "temperature":
        y_label = f"Temperature ({TEMP_UNIT})"
    else:
        y_label = "Sensor Value"

    plt.figure(figsize=(10, 5))
    plt.plot(x, y, label=mode.capitalize(), linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel(y_label)
    plt.title(f"{SENSOR_NAME} - {mode.capitalize()} vs Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python plot_log.py path/to/sensor_log_*.csv")
    else:
        plot_log(sys.argv[1])
