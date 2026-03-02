#include "transmit.h"
#include "receive.h"


// ==========================
// DEFINE YOUR SENSOR PINS
// ==========================

#define LS_LEFT_PIN   4   // <-- change me
#define LS_RIGHT_PIN  5   // <-- change me

// ==========================
// CREATE RECEIVER
// ==========================

Receiver receiver;

// Optional: track state changes for debug
int lastState = -1;

void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println("Receiver Test Starting...");

  // init(syncMs, shortMs, longMs, tolerance, minReading)
  receiver.init(
    100,   // sync pulse ms
    30,    // short pulse ms
    60,    // long pulse ms
    2,     // pulse tolerance ms
    200,    // minimum delta threshold
    true   // debug flag
  );

  Serial.println("Receiver initialised.");
}

void loop() {

  

  // ==========================
  // DEBUG: Print state changes
  // ==========================
  int currentState = receiver.check();

  if (currentState != lastState) {
    Serial.print("State changed to: ");
    Serial.println(currentState);
    lastState = currentState;
  }

  // ==========================
  // If byte received
  // ==========================
  if (receiver.available()) {

    byte b = receiver.read();

    Serial.print("Received byte: 0x");
    Serial.print(b, HEX);
    Serial.print(" (");
    Serial.print(b, BIN);
    Serial.println(")");
  }
}