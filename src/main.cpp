#include <Arduino.h>
#include "hydrosync_cmd.h"
#include "hs_sensors.h"

void initSpecs();
void sensor_init();

uint32_t lastTx = 0;
uint32_t seq = 0;
uint32_t now = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  initSpecs();
  sensor_init();
  Serial.println("ESP32 ready.");
}

void loop() {
  poll_serial();               // always service RX

  // periodic TX independent of RX
  uint32_t now = millis();
  if (now - lastTx >= 100) {   // 100 ms
    update_specs();
    send_ser(seq++);
    lastTx = now;
  }
}
