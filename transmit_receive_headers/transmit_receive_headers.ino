#include "transceiver.h"

// ==========================
// CREATE TRANSCEIVER
// ==========================

Transceiver transceiver;
bool transmit = true; // change to false to receive
byte test_byte = 0x00; // will increment every time a byte is sent and successfully received
void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println("SEQ-based Transceiver Protocol");
  Serial.println("===============================");
  Serial.println("Protocol:");
  Serial.println("  1. Sender: DATA (8b) + checksum (2b) + SEQ (1b) = 11 bits");
  Serial.println("  2. Receiver: ACK (2b) if checksum valid, else no ACK");
  Serial.println("  3. Duplicate detection via SEQ bit : if SEQ == lastSEQ, it's a duplicate, ACK to increment sender's SEQ but don't deliver to application");
  Serial.println();

  // init(syncMs, postSyncMs, shortMs, longMs, interBitMs, tolerance, minReading,
  //      ackTimeout, preAckDelay, dbgProtocol, dbgRejection, dbgStrength, dbgOutputs, dbgInputs, dbgStates)
  transceiver.init(
    150,    // sync pulse ms
    50,     // post-sync ms
    30,     // short pulse ms
    60,     // long pulse ms
    10,     // inter-bit ms
    2,      // pulse tolerance ms
    200,    // minimum delta threshold
    500,    // ACK timeout ms
    75,     // pre-ACK delay ms (wait before sending ACK)
    false,  // debugProtocol - prints handshake state changes
    false,  // debugRejectionReason
    false,  // debugStrength
    false,  // debugOutputs
    false,  // debugInputs
    false   // debugStates
  );

  Serial.println("Transceiver initialised, listening...");
  Serial.println();
}

void loop() {

  // Run the transceiver state machine
  transceiver.check();

  // Check if a byte was successfully received (full handshake complete)
  if (transceiver.available()) {
    byte b = transceiver.read();


    Serial.print("Received byte: 0x");
    Serial.print(b, HEX);
    Serial.print(" (");
    Serial.print(b, BIN);
    Serial.println(")");
  }

   // Check send result BEFORE starting a new send (only relevant if we're transmitting)
   if (transmit && transceiver.isTransactionComplete()) {
     if (transceiver.wasSuccessful()) {
       Serial.println("Send succeeded!");
       test_byte++;
     } else {
       Serial.println("Send failed!");
     }
     transceiver.clearTransaction();
   }

   // Start new transmission if idle
   if (transmit && transceiver.isReceiving()) {
     transceiver.sendByte(test_byte);
   }
}
