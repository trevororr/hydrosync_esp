#include <Arduino.h>
#include <ArduinoJson.h>
#include "hydrosync_cmd.h"
#include "hs_sensors.h"
#include "hs_setters.h"

JsonDocument specs;

bool running = false;

// Sensor library prototypes
JsonDocument get_current();
// Prototypes
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

// Update the JSON doc from sensors/state
void update_specs() {
  JsonDocument ina_vals = get_current();

  specs["flow"]    = get_flow();
  specs["gen_current"] = ina_vals["current_mA"];
  specs["gen_voltage"] = ina_vals["bus_V"];
  specs["gen_power"] = ina_vals["power_mW"];
  specs["UR"]      = get_water_level();
  //specs["LR"]      = lr_get();
  //specs["charge"]  = charge_get();
}

void e_stop() {
  set_flow(0);
  set_pump(0);
}

/*
Process incoming serial data
*/
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
    set_pump(level);
  } else if (strcmp(action, "FLOW_SET") == 0) {
    int level = (value != INT_MIN) ? value : atoi(state);
    set_flow(level);
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
/*
End of serial command processing
*/
