#ifndef SIMULINK_TESTER_H
#define SIMULINK_TESTER_H

#include <Arduino.h>

// Function declarations
void init_simulink_tester();
void set_motor_cmd_v(float level);
float get_motor_cmd_v();

#endif