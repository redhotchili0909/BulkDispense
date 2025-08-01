#include <Wire.h>
#include <Arduino_PortentaMachineControl.h>

// === CONFIGURATION ===
#define SENSOR_ADDR 0x28
#define VALVE_CHANNEL 7   // J6 pin 9

enum Mode { MODE_PRESSURE, MODE_TEMPERATURE, MODE_SLEEP };
Mode currentMode = MODE_PRESSURE;

bool sensorSleeping = false;
bool loggingStarted = false;
float alpha = 0.2;
float filteredPressure = 0.0;
float filteredTemp = 0.0;

unsigned long startTimestamp = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  MachineControl_DigitalOutputs.begin(true);
  MachineControl_DigitalOutputs.writeAll(LOW);  // valve off
  delay(100);
}

// === SENSOR READ FUNCTION ===
bool readSSC(float &pressure_mmH2O, float &temp_C) {
  Wire.requestFrom(SENSOR_ADDR, 4);
  if (Wire.available() < 4) return false;

  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  uint8_t b3 = Wire.read();
  uint8_t b4 = Wire.read();

  uint16_t raw_press = ((uint16_t)b1 << 8 | b2) & 0x3FFF;
  uint16_t raw_temp  = ((uint16_t)b3 << 8 | b4) >> 5;

  // Pressure conversion (±2 inH₂O range)
  float outMin = 1638.0, outMax = 14745.0;
  float inH2O_min = -2.0, inH2O_max = 2.0;
  float pressure_inH2O = ((raw_press - outMin) * (inH2O_max - inH2O_min)) / (outMax - outMin) + inH2O_min;

  pressure_mmH2O = pressure_inH2O * 25.4;  // Convert to mmH₂O
  temp_C = ((float)raw_temp * 200.0 / 2047.0) - 50.0;

  return true;
}

// === SERIAL COMMANDS ===
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
      sensorSleeping = true;
      currentMode = MODE_SLEEP;
      Serial.println("Sensor set to SLEEP mode");

    } else if (input.equalsIgnoreCase("wake")) {
      sensorSleeping = false;
      Serial.println("Sensor WOKE from sleep");

    } else if (input.equalsIgnoreCase("valve_on")) {
      MachineControl_DigitalOutputs.write(VALVE_CHANNEL, HIGH);
      Serial.println("VALVE OPENED");

    } else if (input.equalsIgnoreCase("valve_off")) {
      MachineControl_DigitalOutputs.write(VALVE_CHANNEL, LOW);
      Serial.println("VALVE CLOSED");

    } else if (input.equalsIgnoreCase("start")) {
      loggingStarted = true;
      startTimestamp = millis();
      Serial.println("# Sensor: SSCDRRN002ND2A3");
      Serial.println("# Units: mmH2O");
      Serial.println("timestamp_ms,mode,value");
    }
  }
}

// === MAIN LOOP ===
void loop() {
  processSerial();

  if (sensorSleeping || currentMode == MODE_SLEEP || !loggingStarted) {
    delay(100);
    return;
  }

  float pressure_mmH2O, temperature_C;
  if (!readSSC(pressure_mmH2O, temperature_C)) {
    Serial.println("Error: I2C read failed");
    delay(100);
    return;
  }

  unsigned long timestamp = millis() - startTimestamp;

  if (currentMode == MODE_PRESSURE) {
    filteredPressure = alpha * pressure_mmH2O + (1 - alpha) * filteredPressure;
    Serial.print(timestamp); Serial.print(",pressure,"); Serial.println(filteredPressure, 2);
  } else if (currentMode == MODE_TEMPERATURE) {
    filteredTemp = alpha * temperature_C + (1 - alpha) * filteredTemp;
    Serial.print(timestamp); Serial.print(",temperature,"); Serial.println(filteredTemp, 2);
  }

  delay(100);  // ~10 Hz
}
