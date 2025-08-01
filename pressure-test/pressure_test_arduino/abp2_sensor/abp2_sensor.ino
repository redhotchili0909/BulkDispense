#include <Wire.h>

#define SENSOR_ADDR 0x28  // I2C address of ABP2 sensor

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  Serial.println("ABP2MRRN002ND2A3XX (14-bit I2C)");
  Serial.println("Time_ms,Pressure_mmH2O,Temperature_C,Raw_Pressure,Raw_Temp");
}

void loop() {
  Wire.requestFrom(SENSOR_ADDR, 4);
  if (Wire.available() < 4) {
    Serial.println("I2C read failed");
    delay(500);
    return;
  }

  // === Read 4 bytes ===
  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  uint8_t b3 = Wire.read();
  uint8_t b4 = Wire.read();

  // === Decode pressure (14-bit) and temp (11-bit) ===
  uint16_t raw_press = ((uint16_t)b1 << 8 | b2) & 0x3FFF;
  uint16_t raw_temp  = ((uint16_t)b3 << 8 | b4) >> 5;

  // === Convert pressure: ±2 inH2O full scale ===
  const float output_min = 1638.0;
  const float output_max = 14745.0;
  const float inH2O_min = -2.0;
  const float inH2O_max =  2.0;
  float pressure_inH2O = ((raw_press - output_min) * (inH2O_max - inH2O_min)) / (output_max - output_min) + inH2O_min;
  float pressure_mmH2O = pressure_inH2O * 25.4;

  // === Convert temperature ===
  float temp_C = ((float)raw_temp * 200.0 / 2047.0) - 50.0;

  // === Output CSV ===
  Serial.print(millis());
  Serial.print(",");
  Serial.print(pressure_mmH2O, 2);
  Serial.print(",");
  Serial.print(temp_C, 2);
  Serial.print(",");
  Serial.print(raw_press);
  Serial.print(",");
  Serial.println(raw_temp);

  delay(200);  // 5 Hz sampling
}
