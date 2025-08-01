#include <Wire.h>
#include <Arduino_PortentaMachineControl.h>

// === CONFIG ===
#define SENSOR_ADDR 0x28  // I2C address for DLVR
#define VALVE_CHANNEL 7   // Digital output channel

enum Mode {MODE_PRESSURE, MODE_TEMPERATURE, MODE_SLEEP};
Mode currentMode = MODE_PRESSURE;

float alpha = 0.2;
float filteredPressure = 0.0;
float filteredTemp = 0.0;
bool sensorSleeping = false;
bool loggingStarted = false;

unsigned long startTimestamp = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  MachineControl_DigitalOutputs.begin(true);
  MachineControl_DigitalOutputs.writeAll(LOW);
  delay(100);
}

// === SENSOR READ (DLVR 14-bit protocol) ===
bool readDLVR(float &pressure_mmH2O, float &temp_C) {
  Wire.requestFrom(SENSOR_ADDR, 4);
  if (Wire.available() < 4) return false;

  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  uint8_t b3 = Wire.read();
  uint8_t b4 = Wire.read();

  uint16_t raw_press = ((uint16_t)b1 << 8 | b2) & 0x3FFF;  // 14-bit
  uint16_t raw_temp  = ((uint16_t)b3 << 8 | b4) >> 5;      // 11-bit

  float outMin = 1638.0, outMax = 14745.0;
  float fullScale = 2.0;  // ±1 inH2O span
  float offset = (outMax + outMin) / 2.0;

  float pressure_inH2O = ((float)(raw_press - offset) / (outMax - outMin)) * fullScale;
  pressure_mmH2O = pressure_inH2O * 25.4;

  temp_C = ((float)raw_temp * 200.0 / 2047.0) - 50.0;

  return true;
}

void sendSleepCommand() {
  sensorSleeping = true;
  currentMode = MODE_SLEEP;
  Serial.println("Sensor set to SLEEP mode");
}

void wakeSensor() {
  sensorSleeping = false;
  Serial.println("Sensor WOKE from sleep");
}

void processSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("pres")) {
      currentMode = MODE_PRESSURE;
      Serial.println("Switched to PRESSURE mode");
    } else if (input.equalsIgnoreCase("temp")) {
      currentMode = MODE_TEMPERATURE;
      Serial.println("Switched to TEMPERATURE mode");
    } else if (input.equalsIgnoreCase("sleep")) {
      sendSleepCommand();
    } else if (input.equalsIgnoreCase("wake")) {
      wakeSensor();
    } else if (input.equalsIgnoreCase("valve_on")) {
      MachineControl_DigitalOutputs.write(VALVE_CHANNEL, HIGH);
      Serial.println("VALVE OPENED");
    } else if (input.equalsIgnoreCase("valve_off")) {
      MachineControl_DigitalOutputs.write(VALVE_CHANNEL, LOW);
      Serial.println("VALVE CLOSED");
    } else if (input.equalsIgnoreCase("start")) {
      loggingStarted = true;
      startTimestamp = millis();
      Serial.println(">>> START RECEIVED <<<");
      Serial.println("# Sensor: DLVR-L01D-E1NJ-C-NI3F");
      Serial.println("# Units: mmH2O");
      Serial.println("timestamp_ms,mode,value");
    }
  }
}

void loop() {
  processSerial();

  if (sensorSleeping || currentMode == MODE_SLEEP || !loggingStarted) {
    delay(100);
    return;
  }

  float pressure_mmH2O, temp_C;

  Serial.println("Reading sensor...");

  if (!readDLVR(pressure_mmH2O, temp_C)) {
    Serial.println("Error: I2C read failed");
    delay(100);
    return;
  }

  unsigned long timestamp = millis() - startTimestamp;

  if (currentMode == MODE_PRESSURE) {
    filteredPressure = alpha * pressure_mmH2O + (1 - alpha) * filteredPressure;
    Serial.print(timestamp); Serial.print(",pressure,"); Serial.println(filteredPressure, 2);
  } else if (currentMode == MODE_TEMPERATURE) {
    filteredTemp = alpha * temp_C + (1 - alpha) * filteredTemp;
    Serial.print(timestamp); Serial.print(",temperature,"); Serial.println(filteredTemp, 2);
  }

  delay(100); // 10 Hz
}
