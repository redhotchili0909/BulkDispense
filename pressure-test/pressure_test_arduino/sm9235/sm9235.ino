#include <Wire.h>
#include <Arduino_PortentaMachineControl.h>

// === CONFIG ===
#define SENSOR_ADDR 0x6C
#define VALVE_CHANNEL 7

// === MODES ===
enum Mode {MODE_PRESSURE, MODE_TEMPERATURE, MODE_SLEEP};
Mode currentMode = MODE_PRESSURE;

float alpha = 0.2;
float filteredPressure = 0.0;
float filteredTemp = 0.0;
bool sensorSleeping = false;
bool loggingStarted = false;

unsigned long startTimestamp = 0;

// === SETUP ===
void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Initialize 24V Digital Outputs in latch mode
  MachineControl_DigitalOutputs.begin(true);
  MachineControl_DigitalOutputs.writeAll(LOW); // Safety: start off

  delay(100);
}

// === READ WORD OVER I2C ===
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

// === CONVERSION FUNCTIONS ===
float pressureCountsTo_mmH2O(int16_t counts) {
  float outMin = -26215.0, outMax = 26214.0, pMin = 0.0, pMax = 300.0;
  float pressurePa = pMin + ((counts - outMin) / (outMax - outMin)) * (pMax - pMin);
  return pressurePa * 0.10197;
}

float tempCountsToCelsius(int16_t counts) {
  float b1 = 397.2, b0 = -16881.0;
  return (counts - b0) / b1;
}

// === SENSOR POWER CONTROL ===
void sendSleepCommand() {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x22);
  Wire.write(0x32);
  Wire.write(0x6C);
  Wire.endTransmission();
  sensorSleeping = true;
  Serial.println("Sensor set to SLEEP mode");
}

void wakeSensor() {
  pinMode(PIN_WIRE_SCL, OUTPUT);
  digitalWrite(PIN_WIRE_SCL, LOW);
  delay(1);
  digitalWrite(PIN_WIRE_SCL, HIGH);
  delay(5);
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
  sensorSleeping = false;
  Serial.println("Sensor WOKE from sleep");
}

// === SERIAL COMMAND HANDLER ===
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
      currentMode = MODE_SLEEP;

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

  unsigned long timestamp = millis() - startTimestamp;

  if (currentMode == MODE_PRESSURE) {
    uint16_t raw = readWord(0x30);
    int16_t counts = (int16_t)raw;
    float pressure = pressureCountsTo_mmH2O(counts);
    filteredPressure = alpha * pressure + (1 - alpha) * filteredPressure;
    Serial.print(timestamp); Serial.print(",pressure,"); Serial.println(filteredPressure, 2);

  } else if (currentMode == MODE_TEMPERATURE) {
    uint16_t raw = readWord(0x2E);
    int16_t counts = (int16_t)raw;
    float temp = tempCountsToCelsius(counts);
    filteredTemp = alpha * temp + (1 - alpha) * filteredTemp;
    Serial.print(timestamp); Serial.print(",temperature,"); Serial.println(filteredTemp, 2);
  }

  delay(100); // 10Hz
}
