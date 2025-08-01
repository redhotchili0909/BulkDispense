// === SENSOR LIBRARY ===
// Add new sensors by implementing the SensorInterface class

// === BMP280 PRESSURE/TEMPERATURE SENSOR EXAMPLE ===
/*
class BMP280Sensor : public SensorInterface {
private:
  // BMP280 I2C address (0x76 or 0x77)
  static const uint8_t SENSOR_ADDR = 0x76;
  
  float temperature = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  
  // Add your BMP280-specific register definitions here
  // static const uint8_t BMP280_TEMP_REG = 0xFA;
  // static const uint8_t BMP280_PRESS_REG = 0xF7;

public:
  bool initialize() override {
    Wire.begin();
    delay(100);
    // Add BMP280 initialization code here
    // Check device ID, configure settings, etc.
    return true; // Return false if initialization fails
  }

  bool readData() override {
    // Add code to read temperature and pressure from BMP280
    // temperature = readTemperature();
    // pressure = readPressure();
    // altitude = calculateAltitude(pressure);
    return true; // Return false if read fails
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,temperature_C,pressure_hPa,altitude_m");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.println(altitude, 2);
  }

  void calibrate() override {
    // Add calibration code if needed
    // For pressure sensors, might zero the baseline
  }

  String getSensorName() override {
    return "BMP280_Pressure_Temperature_Sensor";
  }
};
*/

// === DHT22 TEMPERATURE/HUMIDITY SENSOR EXAMPLE ===
/*
#include "DHT.h"
#define DHT_PIN 2
#define DHT_TYPE DHT22

class DHT22Sensor : public SensorInterface {
private:
  DHT dht;
  float temperature = 0.0;
  float humidity = 0.0;

public:
  DHT22Sensor() : dht(DHT_PIN, DHT_TYPE) {}

  bool initialize() override {
    dht.begin();
    delay(2000); // DHT22 needs time to stabilize
    return true;
  }

  bool readData() override {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    
    // Check if reads failed
    if (isnan(humidity) || isnan(temperature)) {
      return false;
    }
    return true;
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,temperature_C,humidity_percent");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.println(humidity, 2);
  }

  void calibrate() override {
    // DHT22 typically doesn't need calibration
  }

  String getSensorName() override {
    return "DHT22_Temperature_Humidity_Sensor";
  }
};
*/

// === DS18B20 TEMPERATURE SENSOR EXAMPLE ===
/*
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2

class DS18B20Sensor : public SensorInterface {
private:
  OneWire oneWire;
  DallasTemperature sensors;
  float temperature = 0.0;

public:
  DS18B20Sensor() : oneWire(ONE_WIRE_BUS), sensors(&oneWire) {}

  bool initialize() override {
    sensors.begin();
    int deviceCount = sensors.getDeviceCount();
    if (deviceCount == 0) {
      return false; // No sensors found
    }
    return true;
  }

  bool readData() override {
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    return temperature != DEVICE_DISCONNECTED_C;
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,temperature_C");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.println(temperature, 3);
  }

  void calibrate() override {
    // DS18B20 is factory calibrated
  }

  String getSensorName() override {
    return "DS18B20_Temperature_Sensor";
  }
};
*/

// === MPU6050 ACCELEROMETER/GYROSCOPE EXAMPLE ===
/*
#include "MPU6050.h"

class MPU6050Sensor : public SensorInterface {
private:
  MPU6050 mpu;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temperature;

public:
  bool initialize() override {
    Wire.begin();
    mpu.initialize();
    return mpu.testConnection();
  }

  bool readData() override {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    temperature = mpu.getTemperature();
    return true;
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temperature_raw");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    Serial.println(temperature);
  }

  void calibrate() override {
    // Could implement offset calibration here
  }

  String getSensorName() override {
    return "MPU6050_IMU_Sensor";
  }
};
*/

// === ANALOG SENSOR EXAMPLE ===
/*
class AnalogSensor : public SensorInterface {
private:
  int analogPin;
  float voltage = 0.0;
  int rawValue = 0;
  float offset = 0.0;

public:
  AnalogSensor(int pin) : analogPin(pin) {}

  bool initialize() override {
    pinMode(analogPin, INPUT);
    return true;
  }

  bool readData() override {
    rawValue = analogRead(analogPin);
    voltage = (rawValue * 5.0) / 1023.0; // Convert to voltage (5V reference)
    return true;
  }

  void printCSVHeader() override {
    Serial.println("timestamp_ms,raw_value,voltage_V,adjusted_voltage_V");
  }

  void printCSVData(unsigned long timestamp) override {
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(rawValue);
    Serial.print(",");
    Serial.print(voltage, 3);
    Serial.print(",");
    Serial.println(voltage - offset, 3);
  }

  void calibrate() override {
    rawValue = analogRead(analogPin);
    voltage = (rawValue * 5.0) / 1023.0;
    offset = voltage; // Set current reading as zero point
  }

  String getSensorName() override {
    return "Analog_Sensor_Pin_" + String(analogPin);
  }
};
*/
