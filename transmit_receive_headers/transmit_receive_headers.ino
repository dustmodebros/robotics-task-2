#include "transceiver.h"

// ==========================
// CREATE TRANSCEIVER
// ==========================

Transceiver transceiver;

void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println("Transceiver Test Starting...");

  // init(syncMs, postSyncMs, shortMs, longMs, interBitMs, tolerance, minReading,
  //      autoAck, dbgProtocol, dbgRejection, dbgStrength, dbgOutputs, dbgInputs, dbgStates)
  transceiver.init(
    100,    // sync pulse ms
    50,     // post-sync ms
    30,     // short pulse ms
    60,     // long pulse ms
    10,     // inter-bit ms
    2,      // pulse tolerance ms
    200,    // minimum delta threshold
    true,   // autoAck - automatically send ACK after receiving
    true,   // debugProtocol - prints transceiver state changes
    false,  // debugRejectionReason - prints why packets are rejected
    false,  // debugStrength - prints max signal strength
    false,  // debugOutputs - prints pulse durations
    false,  // debugInputs - prints raw sensor readings to serial plotter
    false   // debugStates - prints receiver state changes
  );

  Serial.println("Transceiver initialised.");
}

void loop() {

  // Run the transceiver state machine
  transceiver.check();

  // Check if a byte was received
  if (transceiver.available()) {
    byte b = transceiver.read();

    Serial.print("Got byte: 0x");
    Serial.print(b, HEX);
    Serial.print(" (");
    Serial.print(b, BIN);
    Serial.println(")");
  }
}
