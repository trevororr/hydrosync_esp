#ifndef HYDROSYNCCMD_H
#define HYDROSYNCCMD_H

#include <Arduino.h>  // needed for pinMode, digitalWrite, etc.

// Function declarations
void check_ser();
void send_ser( uint32_t seq );
void update_specs();
void initSpecs();
void poll_serial();

#endif