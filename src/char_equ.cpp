#include <Arduino.h>
#include "char_equ.h"
#include "hs_sensors.h"
#include <ArduinoJson.h>
#include <array>

const int array_size = 10;
double current_val;
double power_val;
double voltage_val;
double flow_val;
JsonDocument current_senses;
JsonDocument curve_points;

void curve_gen() {
    for (int i = 0; i < array_size; i++) {
        current_senses = get_current();
        current_val = current_senses["current_mA"];
        power_val = current_senses["power_mW"];
        voltage_val = current_senses["load_V"];
        flow_val = get_flow();
        curve_points[i][0] = flow_val;
        curve_points[i][1] = current_val;
        curve_points[i][2] = power_val;
        curve_points[i][3] = voltage_val;

        Serial.print("[");
        Serial.print(flow_val, 3);      // flow
        Serial.print(",");
        Serial.print(current_val, 3);   // current
        Serial.print(",");
        Serial.print(voltage_val, 3);   // voltage
        Serial.print(",");
        Serial.print(power_val, 3);     // power (mW or W)
        Serial.println("]");

        delay(100); // small delay to simulate sensor reading time
    }
}