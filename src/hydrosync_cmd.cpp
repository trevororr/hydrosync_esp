#include <Arduino.h>
#include <ArduinoJson.h>
#include "hydrosync_cmd.h"
#include "hs_sensors.h"

JsonDocument specs;

bool running = false;

// Sensor library prototypes
JsonDocument get_current();
// Prototypes
void pump_set(int level);
void flow_set(int level);
void e_stop();
void handle_line(const String& line);

// RX buffer (non-blocking, line-delimited)
static String rxLine;

// --- Call this in setup() ---
void initSpecs() {
  specs["flow"] = 0.0f;
  specs["current"] = 0.0f;
  specs["voltage"] = 0.0f;
  specs["UR"] = 0.0f;   // align with Python
  specs["LR"] = 0.0f;   // align with Python
  specs["power"] = 0.0f;
  specs["charge"] = 0.0f;

  Serial.setTimeout(10); // keep readStringUntil reasonably snappy if you still use it
}

// Non-blocking serial poll; call every loop()
void poll_serial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rxLine.length() > 0) {
        handle_line(rxLine);
        rxLine = "";
      }
    } else if (c != '\r') {
      rxLine += c;
      if (rxLine.length() > 512) rxLine = ""; // safety cap
    }
  }
}

// Handle one complete JSON line
void handle_line(const String& line) {
  // v6: StaticJsonDocument<256> doc; v7: JsonDocument doc;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    // Optional: structured error (JSON), or ignore to keep wire clean
    // Serial.printf("{\"error\":\"json\",\"detail\":\"%s\"}\n", err.c_str());
    return;
  }

  const char* action = doc["action"] | "";
  const char* state  = doc["state"]  | "";  // optional
  int value          = doc["value"]  | INT_MIN;

  if (strcmp(action, "LED_ON") == 0) {
    digitalWrite(2, HIGH);
  } else if (strcmp(action, "LED_OFF") == 0) {
    digitalWrite(2, LOW);
  } else if (strcmp(action, "START") == 0) {
    running = true;
    digitalWrite(2, HIGH);
  } else if (strcmp(action, "STOP") == 0) {
    running = false;
    digitalWrite(2, LOW);
    e_stop();
  } else if (strcmp(action, "PUMP_SET") == 0) {
    int level = (value != INT_MIN) ? value : atoi(state);
    pump_set(level);
  } else if (strcmp(action, "FLOW_SET") == 0) {
    int level = (value != INT_MIN) ? value : atoi(state);
    flow_set(level); // <-- fixed
  } else if (strcmp(action, "status") == 0) {
    // send immediate snapshot
    static uint32_t seq = 0;
    update_specs();
    send_ser(seq++);
  } else {
    // Unknown action – optional structured response
    Serial.println("{\"error\":\"unknown_action\"}");
  }
}

// Send one JSON telemetry line
void send_ser (uint32_t seq) {
  specs["seq"] = seq;
  // specs["action"] = "update"; // optional metadata
  serializeJson(specs, Serial);
  Serial.print('\n'); // line delimiter
}

// IMPLEMENTATIONS — remove Serial.printlns to keep the wire clean
void pump_set(int level) {
  // TODO: apply pump control
  (void)level;
}
void flow_set(int level) {
  // TODO: apply flow control
  (void)level;
}

float voltage_get() { return 12.6f; }
float ur_get()      { return 75.0f; }
float lr_get()      { return 33.0f; }
float power_get()   { return fabsf(ur_get() - lr_get()); }
float charge_get()  {
  float charge = analogRead(4);
  charge = (charge / 4095.0f) * 100.0f; // assuming 12-bit ADC, scale to 0-100%
  return charge;
}

void e_stop() {
  flow_set(0);
  pump_set(0);
}

// Update the JSON doc from sensors/state
void update_specs() {
  specs["flow"]    = get_flow();
  specs["current"] = get_current();
  specs["voltage"] = voltage_get();
  specs["UR"]      = ur_get();     // aligned with Python
  specs["LR"]      = lr_get();     // aligned with Python
  specs["power"]   = power_get();
  specs["charge"]  = charge_get();
}
