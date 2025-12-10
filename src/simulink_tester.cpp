/*
Used to test Simulink integration with C++ code.
*/

#include "simulink_tester.h"
#include <Arduino.h>

void setDACVoltage(uint8_t pin, float volts);

// Choose explicit pins
constexpr uint8_t MOTOR_CMD_DAC_PIN   = 25; // DAC output
constexpr uint8_t MOTOR_CMD_SENSE_PIN = 35; // ADC input

void init_simulink_tester(){
    analogReadResolution(12);  // 0–4095

    setDACVoltage(MOTOR_CMD_DAC_PIN, 0.0f);   // full scale output as a test
    pinMode(MOTOR_CMD_SENSE_PIN, INPUT);     // not strictly required for analogRead(), but ok
}

// sets motor command voltage by writing to a DAC pin
void set_motor_cmd_v(float level){
    Serial.printf("SIMULINK_ANALOG command: raw value=%f\n", level);
    setDACVoltage(MOTOR_CMD_DAC_PIN, level);
}

// gets actual motor command voltage by reading from an analog pin
float get_motor_cmd_v(){
    int level = analogRead(MOTOR_CMD_SENSE_PIN);
    float voltage = (level / 4095.0f) * 3.3f;
    return voltage;
}

// sets the voltage on a DAC pin (0-3.3V)
void setDACVoltage(uint8_t pin, float volts) {
  if (volts < 0) volts = 0;
  if (volts > 3.3) volts = 3.3;

  uint8_t dacValue = (uint8_t)((volts / 3.3f) * 255);
  dacWrite(pin, dacValue);
}
