#include <Arduino.h>
#include "Wire.h"
#include "Adafruit_INA219.h"
#include "hs_sensors.h"
#include <ArduinoJson.h>

Adafruit_INA219 ina219;
JsonDocument current_vals;

void sensor_init() {
  Wire.begin(21, 22); // Initialize I2C with SDA on GPIO 21 and SCL on GPIO 22

  while (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    delay(100);
  }

  ina219.setCalibration_32V_2A(); // Set calibration for 32V, 2A range

  Serial.println("Sensors initialized!");
}

JsonDocument get_current() {
  float shunt_mV = ina219.getShuntVoltage_mV();
  float bus_V = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float load_V = bus_V + (shunt_mV / 1000.0f);

  current_vals["shunt_mV"] = shunt_mV;
  current_vals["bus_V"] = bus_V;
  current_vals["current_mA"] = current_mA;
  current_vals["power_mW"] = power_mW;
  current_vals["load_V"] = load_V;

  return current_vals;
}

float get_flow() {
  float flow = analogRead(34); // Read flow sensor (example pin)
  return flow;
}