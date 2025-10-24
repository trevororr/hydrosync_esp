#ifndef HS_SENSORS_H
#define HS_SENSORS_H

#include <Arduino.h>  // needed for pinMode, digitalWrite, etc.
#include <ArduinoJson.h>

// Function declarations
void sensor_init();
JsonDocument get_current();
double get_flow();
uint8_t get_water_level();

#endif