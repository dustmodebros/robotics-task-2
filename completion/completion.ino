#include "transceiver.h"

/*

 * Each robot continuously broadcasts its `collected_ids` bitmask and listens
 * for transmissions from its neighbour(s). When it receives a byte it ORs it
 * into its own `collected_ids`, so information propagates to master.

 * Once master sees collected_ids == 0b1111 it broadcasts HALT_SIGNAL (0xAA).
 * Every robot that receives 0xAA stops transmitting and enters CALIB_DONE.


 *   Change MY_ID_BIT on each robot before flashing:
 *     Robot 1 (master) : 0b0001
 *     Robot 2          : 0b0010
 *     Robot 3          : 0b0100
 *     Robot 4          : 0b1000

 */


#define MY_ID_BIT     0b0001   // change fr each robot
#define MASTER_ID_BIT 0b0001   // Master is always robot whose bit is 0001
#define ALL_READY     0b1111   
#define HALT_SIGNAL   0xAA     // needs to change when we decide transmission language


Transceiver transceiver;


enum CalibState {
  CALIB_BROADCASTING,   
  CALIB_HALTING,        
  CALIB_DONE            
};

CalibState calibState   = CALIB_BROADCASTING;
byte       collected_ids = MY_ID_BIT;          // Starts with only own bit set
bool       isMaster      = (MY_ID_BIT == MASTER_ID_BIT);

// How many times the master re-broadcasts the halt signal
#define HALT_REPEAT_COUNT 5
int haltRepeatsSent = 0;



void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== Calibration Confirmation Routine ===");
  Serial.print("Role : ");
  Serial.println(isMaster ? "MASTER" : "ROBOT");
  Serial.print("My ID bit : 0b");
  Serial.println(MY_ID_BIT, BIN);
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
    75,     // pre-ACK delay ms (wait before sending ACK to allow transmitter to get into the correct mode)
    false,  // debugProtocol        - prints protocol state changes
    false,  // debugRejectionReason - prints reason for not accepting packet
    false,  // debugStrength        - prints max delta peak
    false,  // debugOutputs         - prints pulse lengths and corresponding interpretation
    false,  // debugInputs          - prints raw delta input to serial plotter as well as behaviour visualisation
    false   // debugStates          - prints receiver's current state every time the state changes
  );

  Serial.println("Transceiver initialised. Starting calibration broadcast...");
  Serial.println();
}



void loop() {

  transceiver.check();


  if (transceiver.available()) {
    byte received = transceiver.read();

    if (calibState == CALIB_DONE) {
      
    }
    else if (received == HALT_SIGNAL) {
      // Any robot stops here
      calibState = CALIB_DONE;
      Serial.println("HALT signal Calibration complete");
    }
    else {
      // Merge the received collected_ids into our own
      byte prev = collected_ids;
      collected_ids |= received;

      if (collected_ids != prev) {
        Serial.print("Merged received 0b");
        Serial.print(received, BIN);
        Serial.print(" → collected_ids now 0b");
        Serial.println(collected_ids, BIN);
      }

      // Master check: if we've just seen all four robots, move to halting
      if (isMaster && collected_ids == ALL_READY && calibState == CALIB_BROADCASTING) {
        calibState    = CALIB_HALTING;
        haltRepeatsSent = 0;
        Serial.println(">>> All robots confirmed! Sending HALT signal... <<<");
      }
    }
  }

  
  if (transceiver.isTransactionComplete()) {
    if (!transceiver.wasSuccessful()) {
      // ACK timeout — retransmit next idle cycle
      Serial.println("Send failed (ACK timeout) — will retry.");
    }
    transceiver.clearTransaction();

    // Count halt repeats
    if (calibState == CALIB_HALTING) {
      haltRepeatsSent++;
      if (haltRepeatsSent >= HALT_REPEAT_COUNT) {
        calibState = CALIB_DONE;
        Serial.println("Calibration done from master side(finished sending HALT signal)");
      }
    }
  }

  
  if (transceiver.isReceiving()) {
    switch (calibState) {

      case CALIB_BROADCASTING:
        // Everyone broadcasts their current collected_ids bitmask
        transceiver.sendByte(collected_ids);
        break;

      case CALIB_HALTING:
        // Master floods the halt signal
        transceiver.sendByte(HALT_SIGNAL);
        break;

      case CALIB_DONE:
        
        break;
    }
  }
}