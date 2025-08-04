import serial
import csv
import time
import os
from datetime import datetime
import matplotlib.pyplot as plt

# === CONFIGURATION ===
PORT = 'COM6'  # Adjust this to your actual port
BAUD = 115200
RECORD_DURATION = 150  # seconds
VAlVE_ON_DURATION = 15  # seconds
OUTPUT_FOLDER = './log'
PRESSURE_UNIT = "mmH₂O"
TEMP_UNIT = "°C"

# SENSOR_NAME = "SM9235"
# SENSOR_UNITS = {
#     "pressure": "mmH₂O",
#     "temperature": "°C"
# }

# SENSOR_NAME = "ABP2DDAN001PD2A3XX"
# SENSOR_UNITS = {
#     "pressure": "mmH₂O",
#     "temperature": "°C"
# }

# SENSOR_NAME = "DLVR-L01D-E1NJ-C-NI3F"
# SENSOR_UNITS = {
#     "pressure": "mmH₂O",
#     "temperature": "°C"
# }

# SENSOR_NAME = "ABP2MRRN002ND2A3XX"
# SENSOR_UNITS = {
#     "pressure": "mmH₂O",
#     "temperature": "°C"
# }

SENSOR_NAME = "SSCDRRN002ND2A3"
SENSOR_UNITS = {
    "pressure": "mmH₂O",
    "temperature": "°C"
}

def generate_filename():
    sensor_folder = os.path.join(OUTPUT_FOLDER, SENSOR_NAME)
    os.makedirs(sensor_folder, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(sensor_folder, f"sensor_log_{timestamp}.csv")


def wait_for_serial_connection(port, baud):
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(2)
    ser.reset_input_buffer()
    return ser

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

def main():
    filename = generate_filename()
    ser = wait_for_serial_connection(PORT, BAUD)

    print("Starting test...")
    ser.write(b'start\n')
    time.sleep(2)

    print("Opening valve...")
    ser.write(b'valve_on\n')
    valve_on_time = time.time()

    with open(filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        print(f"Logging to {filename}")

        # Write metadata as comments
        mode = None
        while not mode:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("timestamp_ms"):
                _, mode, _ = line.split(",")
                csvfile.write(f"# Sensor: {SENSOR_NAME}\n")
                csvfile.write(f"# Mode: {mode}\n")
                csvfile.write(f"# Units: {SENSOR_UNITS.get(mode, 'unknown')}\n")
                writer.writerow(line.split(','))
                break

        start_time = time.time()
        header_written = False

        while (time.time() - start_time) < RECORD_DURATION + 5:
            try:
                line = ser.readline().decode('utf-8').strip()

                if not line:
                    continue

                if not header_written and line.startswith("timestamp_ms"):
                    writer.writerow(line.split(','))
                    header_written = True
                    continue

                if ',' in line:
                    writer.writerow(line.split(','))
                    print(line)

                if time.time() - valve_on_time > VAlVE_ON_DURATION:
                    print("Closing valve...")
                    ser.write(b'valve_off\n')
                    valve_on_time = float('inf')

            except Exception as e:
                print(f"Error reading serial: {e}")
                break

    ser.close()
    print("Logging complete.")
    print("Plotting result...")
    plot_log(filename)

if __name__ == "__main__":
    main()
