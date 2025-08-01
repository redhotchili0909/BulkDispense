# Generic Sensor Data Logger

This project provides a flexible framework for logging data from various sensors using Arduino and Python.

## Files Overview

### Arduino Code
- **`generic_sensor_logger.ino`** - Main Arduino sketch with sensor interface framework
- **`sensor_examples.ino`** - Examples of how to implement different sensor types
- **`sm9235.ino`** - Original SM9235-specific code (legacy)

### Python Code
- **`generic_data_logger.py`** - Enhanced Python logger that auto-detects sensor types
- **`data_logger.py`** - Your current working logger
- **`test.py`** - Original test script (legacy)

## How It Works

### Arduino Side
The Arduino code uses an object-oriented approach with a `SensorInterface` base class. This allows you to easily swap between different sensors by changing just one line of code.

```cpp
// To change sensors, just modify this line in generic_sensor_logger.ino:
SM9235Sensor sensor;           // Current: SM9235 pressure sensor
// DummyTempSensor sensor;     // Alternative: Dummy temperature sensor
// BMP280Sensor sensor;        // Future: BMP280 pressure sensor
```

### Python Side
The Python logger automatically:
- Detects the connected sensor type
- Adapts to different CSV formats
- Creates appropriately named log files
- Handles various data types

## Adding a New Sensor

### Step 1: Implement the Sensor Class (Arduino)
Create a new class that inherits from `SensorInterface`:

```cpp
class YourSensor : public SensorInterface {
public:
  bool initialize() override {
    // Initialize your sensor hardware
    return true; // or false if initialization fails
  }

  bool readData() override {
    // Read data from your sensor
    return true; // or false if read fails
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,your_data_columns");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.println(your_sensor_value);
  }

  void calibrate() override {
    // Implement calibration if needed
  }

  String getSensorName() override {
    return "Your_Sensor_Name";
  }
};
```

### Step 2: Select Your Sensor
In `generic_sensor_logger.ino`, change the sensor declaration:

```cpp
YourSensor sensor;  // Use your new sensor
```

### Step 3: Test
1. Upload the Arduino code
2. Run the Python logger
3. The system should auto-detect your sensor and log data appropriately

## Current Sensor Support

### Implemented
- **SM9235** - Pressure and temperature sensor (I2C)
- **DummyTempSensor** - Simulated temperature sensor for testing

### Examples Provided (in sensor_examples.ino)
- **BMP280** - Pressure, temperature, altitude
- **DHT22** - Temperature and humidity
- **DS18B20** - High-precision temperature
- **MPU6050** - Accelerometer and gyroscope
- **Analog Sensor** - Generic analog input

## Configuration

### Arduino Settings
```cpp
#define RECORD_DURATION_MS 45000UL    // Recording duration (45 seconds)
#define SAMPLING_INTERVAL_MS 100      // Sample every 100ms (10 Hz)
```

### Python Settings
```python
PORT = 'COM5'                    # Serial port
BAUD = 115200                   # Baud rate
RECORD_DURATION = 45            # Recording duration (seconds)
OUTPUT_FOLDER = './log'         # Output directory
AUTO_DETECT_SENSOR = True       # Auto-detect sensor type
```

## Serial Commands

- **`start`** - Begin data logging
- **`info`** - Get current sensor information

## Output Files

Files are automatically named with:
- Sensor name
- Timestamp
- CSV format

Example: `SM9235_Pressure_Sensor_log_20250729_143022.csv`

## Data Format

Each sensor can define its own CSV format. The Python logger automatically adapts to:
- Different numbers of columns
- Different data types
- Different sensor-specific headers

## Advantages of This Approach

1. **Modularity** - Easy to add new sensors without changing core code
2. **Consistency** - All sensors use the same logging framework
3. **Flexibility** - Each sensor can have its own data format and calibration
4. **Auto-detection** - Python logger automatically adapts to different sensors
5. **Maintainability** - Clear separation between sensor-specific and generic code

## Troubleshooting

### Arduino Issues
- Make sure the correct sensor is uncommented in the main file
- Check I2C connections for I2C sensors
- Verify sensor-specific library installations

### Python Issues
- Install required packages: `pip install pyserial`
- Check COM port in configuration
- Ensure output directory exists and is writable

### Serial Communication
- Both Arduino and Python must use the same baud rate
- Check that the correct port is specified
- Make sure no other programs are using the serial port

## Future Enhancements

- Web-based configuration interface
- Real-time plotting
- Data export to different formats (JSON, Excel)
- Multi-sensor logging from single Arduino
- Wireless data transmission
