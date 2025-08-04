import os
import csv
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from matplotlib.ticker import MultipleLocator

# === CONFIGURATION ===
SENSOR_NAME = "SSCDRRN002ND2A3"
LOG_DIR = f"./log/{SENSOR_NAME}"
PRESSURE_UNIT = "mmH₂O"
TEMP_UNIT = "°C"

# --- Load log data ---
def load_log(filepath):
    timestamps = []
    values = []
    mode = None

    with open(filepath, 'r') as file:
        while True:
            pos = file.tell()
            line = file.readline()
            if not line.startswith("#"):
                file.seek(pos)
                break

        reader = csv.reader(file)
        headers = next(reader)
        if headers != ["timestamp_ms", "mode", "value"]:
            raise ValueError("Invalid format")

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

# --- Steady state detection ---
def detect_steady_state(timestamps, values, window_sec=3, std_threshold=0.5, mean_delta=2.0):
    if len(timestamps) < 10:
        return None

    x = np.array(timestamps)
    y = np.array(values)
    dt = np.median(np.diff(x))
    window_size = int(window_sec / dt)

    global_mean = np.mean(y)

    for i in range(len(y) - window_size):
        window = y[i:i + window_size]
        win_mean = np.mean(window)
        win_std = np.std(window)

        if win_std < std_threshold and abs(win_mean - global_mean) < mean_delta:
            return x[i]
    return None

# --- Plot annotation ---
def annotate_steady_state(ax, x, y, time_value, color='red', label='Steady'):
    ax.axvline(time_value, color=color, linestyle='--', label=f'{label} state')
    ax.text(time_value + 1.0, min(y) + 0.1*(max(y) - min(y)),
            f"{label} ≈ {time_value:.1f}s", color=color, fontsize=10,
            bbox=dict(facecolor='white', edgecolor=color, boxstyle='round,pad=0.3'))

# --- PNG Export ---
def ask_to_save_plot(fig, title):
    choice = input("Do you want to export this plot as PNG? (y/n): ").strip().lower()
    if choice == 'y':
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"{title.replace(' ', '_')}_{timestamp}.png"
        filepath = os.path.join(LOG_DIR, filename)
        fig.savefig(filepath)
        print(f"Saved plot to {filepath}")

# --- Individual plot ---
def plot_individual(filepath):
    x, y, mode = load_log(filepath)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(x, y, label=os.path.basename(filepath), linewidth=2)
    ax.set_xlabel("Time (s)")
    ylabel = f"{mode.capitalize()} ({PRESSURE_UNIT if mode == 'pressure' else TEMP_UNIT})"
    ax.set_ylabel(ylabel)
    title = f"{SENSOR_NAME} - {mode.capitalize()} (Individual)"
    ax.set_title(title)
    ax.grid(True)
    ax.legend()
    ax.xaxis.set_major_locator(MultipleLocator(10))  # <--- Add this
    plt.tight_layout()

    manual = input("Would you like to manually annotate steady state time? (y/n): ").strip().lower()
    if manual == 'y':
        try:
            t_input = float(input("Enter steady-state time (in seconds): "))
            annotate_steady_state(ax, x, y, t_input, color='blue', label='Manual Steady')
        except ValueError:
            print("Invalid input. Skipping manual annotation.")
    else:
        steady_time = detect_steady_state(x, y)
        if steady_time:
            annotate_steady_state(ax, x, y, steady_time, color='red', label='Auto Steady')

    plt.show()
    ask_to_save_plot(fig, title)

# --- Overlayed plot ---
def plot_overlayed(filepaths):
    mode_set = set()
    fig, ax = plt.subplots(figsize=(12, 6))

    lines = []
    for filepath in filepaths:
        x, y, mode = load_log(filepath)
        mode_set.add(mode)
        label = os.path.basename(filepath)
        lines.append((x, y, label))
        ax.plot(x, y, label=label, linewidth=2)

    if len(mode_set) > 1:
        raise ValueError(f"All logs must be of the same mode. Found: {mode_set}")

    ylabel = f"{mode.capitalize()} ({PRESSURE_UNIT if mode == 'pressure' else TEMP_UNIT})"
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    title = f"{SENSOR_NAME} - {mode.capitalize()} (Overlayed)"
    ax.set_title(title)
    ax.grid(True)
    ax.legend()
    ax.xaxis.set_major_locator(MultipleLocator(10))  # <--- Add this
    plt.tight_layout()

    x, y, _ = lines[-1]  # Use last run for annotation

    manual = input("Would you like to manually annotate steady state time? (y/n): ").strip().lower()
    if manual == 'y':
        try:
            t_input = float(input("Enter steady-state time (in seconds): "))
            annotate_steady_state(ax, x, y, t_input, color='blue', label='Manual Steady')
        except ValueError:
            print("Invalid input. Skipping manual annotation.")
    else:
        steady_time = detect_steady_state(x, y)
        if steady_time:
            annotate_steady_state(ax, x, y, steady_time, color='red', label='Auto Steady')

    plt.show()
    ask_to_save_plot(fig, title)

# --- Main selection logic ---
def select_and_plot():
    if not os.path.isdir(LOG_DIR):
        print(f"No log directory found at {LOG_DIR}")
        return

    files = [f for f in os.listdir(LOG_DIR) if f.endswith(".csv")]
    files.sort()

    print("\nAvailable logs:")
    for idx, f in enumerate(files):
        print(f"{idx+1}: {f}")

    print("\nOptions:")
    print("  [i] Plot individually")
    print("  [a] Plot all overlayed")
    print("  [m] Choose multiple to overlay (comma-separated numbers)")
    choice = input("\nSelect mode (i/a/m): ").strip().lower()

    if choice == 'i':
        sel = input("Enter file number to plot: ").strip()
        if sel.isdigit() and 0 < int(sel) <= len(files):
            plot_individual(os.path.join(LOG_DIR, files[int(sel)-1]))
        else:
            print("Invalid selection.")

    elif choice == 'a':
        full_paths = [os.path.join(LOG_DIR, f) for f in files]
        plot_overlayed(full_paths)

    elif choice == 'm':
        sel = input("Enter comma-separated file numbers (e.g., 1,3,5): ")
        try:
            idxs = [int(s.strip())-1 for s in sel.split(",")]
            selected = [os.path.join(LOG_DIR, files[i]) for i in idxs if 0 <= i < len(files)]
            if selected:
                plot_overlayed(selected)
            else:
                print("No valid selections.")
        except Exception as e:
            print(f"Error: {e}")

    else:
        print("Invalid mode.")

if __name__ == "__main__":
    select_and_plot()
