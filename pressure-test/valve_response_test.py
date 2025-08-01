import serial
import csv
import time
import os
from datetime import datetime
import matplotlib.pyplot as plt

# === CONFIGURATION ===
PORT = 'COM6'         # Update to your Arduino COM port
BAUD = 115200
OUTPUT_FOLDER = './log/response_test'
VALVE_DURATIONS = [1.0, 2.0, 5.0, 10.0]  # Seconds
LOG_DURATION = 15     # Seconds per test

# === SETUP ===
def generate_filename(duration):
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return f"{OUTPUT_FOLDER}/valve_{duration}s_{timestamp}.csv"

def wait_for_serial(port, baud):
    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(2)
    ser.reset_input_buffer()
    return ser

def load_and_plot(filepath, label, valve_duration):
    timestamps, values = [], []
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith("#") or line.startswith("timestamp"):
                continue
            try:
                ts, mode, val = line.strip().split(',')
                if mode == 'pressure':
                    timestamps.append(int(ts) / 1000.0)
                    values.append(float(val))
            except:
                continue
    plt.plot(timestamps, values, label=f"{label}s")

    # Draw valve ON period (assumed to be 1s → 1s + duration)
    plt.axvspan(1, 1 + valve_duration, color='gray', alpha=0.2)


# === MAIN TEST LOGIC ===
def run_test():
    ser = wait_for_serial(PORT, BAUD)

    for duration in VALVE_DURATIONS:
        print(f"\n=== Running test: valve ON for {duration} seconds ===")
        filename = generate_filename(duration)

        ser.write(b'start\n')
        time.sleep(1)

        ser.write(b'valve_on\n')
        t_on = time.time()
        time.sleep(duration)
        ser.write(b'valve_off\n')
        t_off = time.time()

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            print(f"Logging to {filename}")
            start_time = time.time()
            header_written = False

            while time.time() - start_time < LOG_DURATION:
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
                except Exception as e:
                    print(f"Serial read error: {e}")
                    break

        load_and_plot(filename, label=f"{duration}s", valve_duration=duration)

    ser.close()
    print("\nAll tests complete. Plotting results...")
    plt.title("Valve Response Test")
    plt.xlabel("Time (s)")
    plt.ylabel("Pressure (mmH₂O)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_test()
