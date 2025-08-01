#include <Wire.h>

// === SENSOR INTERFACE ===
// Abstract base class for sensors
class SensorInterface {
public:
  virtual bool initialize() = 0;
  virtual bool readData() = 0;
  virtual void printCSVHeader() = 0;
  virtual void printCSVData(unsigned long timestamp) = 0;
  virtual void calibrate() = 0;
  virtual String getSensorName() = 0;
};

// === SM9235 PRESSURE SENSOR ===
class SM9235Sensor : public SensorInterface {
private:
  static const uint8_t SENSOR_ADDR = 0x6C;
  static const float ALPHA = 0.2;  // EMA smoothing factor
  
  float filteredPressure = 0.0;
  float pressureOffset = 0.0;
  float filteredTemp = 0.0;
  float rawPressure = 0.0;
  float rawTemp = 0.0;

  uint16_t readWord(uint8_t regAddr) {
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(regAddr);
    Wire.endTransmission(false);
    Wire.requestFrom(SENSOR_ADDR, 2);
    if (Wire.available() < 2) return 0xFFFF;
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    return (hi << 8) | lo;
  }

  float pressureCountsTo_mmH2O(int16_t counts) {
    float outMin = -26215.0;
    float outMax =  26214.0;
    float pMin = 0.0;
    float pMax = 300.0;
    float pressurePa = pMin + ((counts - outMin) / (outMax - outMin)) * (pMax - pMin);
    return pressurePa * 0.10197;
  }

  float tempCountsToCelsius(int16_t counts) {
    float b1 = 397.2;
    float b0 = -16881.0;
    return (counts - b0) / b1;
  }

public:
  bool initialize() override {
    Wire.begin();
    delay(100);
    // Test if sensor responds
    uint16_t testRead = readWord(0x30);
    return testRead != 0xFFFF;
  }

  bool readData() override {
    // Read pressure
    uint16_t rawP = readWord(0x30);
    if (rawP == 0xFFFF) return false;
    int16_t countsP = (int16_t)rawP;
    rawPressure = pressureCountsTo_mmH2O(countsP) - pressureOffset;
    filteredPressure = ALPHA * rawPressure + (1 - ALPHA) * filteredPressure;

    // Read temperature
    uint16_t rawT = readWord(0x2E);
    if (rawT == 0xFFFF) return false;
    int16_t countsT = (int16_t)rawT;
    rawTemp = tempCountsToCelsius(countsT);
    filteredTemp = ALPHA * rawTemp + (1 - ALPHA) * filteredTemp;

    return true;
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,pressure_mmH2O,temperature_C,raw_pressure_mmH2O,raw_temperature_C");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(filteredPressure, 3);
    Serial.print(",");
    Serial.print(filteredTemp, 2);
    Serial.print(",");
    Serial.print(rawPressure, 3);
    Serial.print(",");
    Serial.println(rawTemp, 2);
  }

  void calibrate() override {
    uint16_t raw = readWord(0x30);
    int16_t counts = (int16_t)raw;
    pressureOffset = pressureCountsTo_mmH2O(counts);
    filteredPressure = 0.0;
    filteredTemp = 0.0;
  }

  String getSensorName() override {
    return "SM9235_Pressure_Sensor";
  }
};

// === EXAMPLE: DUMMY TEMPERATURE SENSOR ===
class DummyTempSensor : public SensorInterface {
private:
  float temperature = 25.0;
  float offset = 0.0;

public:
  bool initialize() override {
    // Dummy sensor always initializes successfully
    return true;
  }

  bool readData() override {
    // Simulate temperature reading with some noise
    temperature = 25.0 + sin(millis() * 0.001) * 5.0 + random(-100, 100) * 0.01;
    return true;
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,temperature_C");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.println(temperature - offset, 2);
  }

  void calibrate() override {
    offset = temperature - 25.0; // Calibrate to 25°C
  }

  String getSensorName() override {
    return "Dummy_Temperature_Sensor";
  }
};

// === MAIN LOGGER CODE ===
#define RECORD_DURATION_MS 45000UL
#define SAMPLING_INTERVAL_MS 100

// Select your sensor here - just change this line!
SM9235Sensor sensor;
// DummyTempSensor sensor;  // Uncomment this line to use dummy sensor instead

bool recording = false;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.print("Initializing ");
  Serial.print(sensor.getSensorName());
  Serial.println("...");
  
  if (sensor.initialize()) {
    Serial.println("Sensor initialized successfully!");
  } else {
    Serial.println("Failed to initialize sensor!");
    while(1); // Stop execution
  }
  
  Serial.println("Type 'start' to begin test recording (45 seconds)...");
}

void handleSerialInput() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("start")) {
      Serial.print("Calibrating ");
      Serial.print(sensor.getSensorName());
      Serial.println("...");
      sensor.calibrate();
      
      startTime = millis();
      recording = true;
      sensor.printCSVHeader();
    } else if (cmd.equalsIgnoreCase("info")) {
      Serial.print("Current sensor: ");
      Serial.println(sensor.getSensorName());
    }
  }
}

void loop() {
  handleSerialInput();

  if (recording) {
    unsigned long now = millis();
    if (now - startTime <= RECORD_DURATION_MS) {
      static unsigned long lastSample = 0;
      if (now - lastSample >= SAMPLING_INTERVAL_MS) {
        lastSample = now;

        if (sensor.readData()) {
          sensor.printCSVData(now - startTime);
        } else {
          Serial.println("Error reading sensor data");
        }
      }
    } else {
      recording = false;
      Serial.println("Test complete. Copy above and save as .csv");
      Serial.print("Type 'start' to begin another run with ");
      Serial.print(sensor.getSensorName());
      Serial.println(".");
    }
  }
}
