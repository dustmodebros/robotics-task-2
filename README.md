# robotics-task-2
A repository to consolidate code for the "The Mark 4" group for the SEMTM0042-43 (Robotics) unit for Bristol University

## Things to focus on
Branch protection is on! All changes have to be done on a PR now, and PRs to main require at least one reviewer. We'll see whether that's annoying.

We need to better define what the task _is_ specifically. We need a flowchart or something describing an individual robot's behaviour, will they need to synchronise? See issue #1 for list of things we need to figure out.

## Receive and Transmit
To use the contents of this repo, first of all clone it, and open each of the transmit and receive .ino files in separate arduino windows.
Then remove the chassis-s from two Pololu 3pi+ robots, and upload the code respectively to one and the other. You'll need batteries in the transmitter, and your receiver communicating over Serial.

## Debug Settings
You should see the test bit of 0xAA transmitted and received, and you can see debug info by enabling the following:
|          Name         | Serial Monitor or Plotter? |                                                 Description                                                  |
| --------------------- | -------------------------- | ------------------------------------------------------------------------------------------------------------ |
| DEBUG_STATES          | Monitor                    | Prints the state that the receiver state machine is in whenever the state changes.                           |
| DEBUG_OUTPUTS         | Monitor                    | Prints the duration of received pulse lengths whenever a pulse is detected, as well as how it's interpreted. |
| DEBUG_STRENGTH        | Monitor                    | Prints the maximum detected delta from baseline per pulse.                                                   |
| DEBUG_REJCTION_REASON | Monitor                    | Prints the reason why a pulse is detected to be malformed and thus ignored.                                  |
| DEBUG_INPUTS          | Plotter                    | Shows the received pulses, along with whether the pulses are interpreted as sync, data or checksum.          |

Happy RX/TXing!

## How it works
The transmission and reception works based on the principle of pulse-duration encoding/decoding. A pulse of a long duration encodes a 1, and a short duration encodes a 0. There's also a longer sync pulse so that the recipient can tell when a byte begins and ends. A two-bit checksum is transmitted after the byte which checks the interleaved XOR as follows:
```ascii
               Byte                Checksum
  b7  b6  b5  b4  b3  b2  b1  b0    c1  c0
  │   ╚═══╪XOR╝   │   ╚═══╪XOR╝     ↑   ⇑
  └──XOR──┘ ║     └──XOR──┘ ║       │   ║
      └─────╫XOR──────┘     ║       │   ║
            ╚═╪═════XOR═════╝       │   ║
              └──────╫──────────────┘   ║
                     ╚══════════════════╝
```
This checksum approach is implemented to check for burst errors, and has the benefit of detecting all single bit flips, as well as detecting all 2 and 3 consecutive bit flips.

Right now the timings are as follows:
| Timing Variable Name |            Duration in Milliseconds           |
| -------------------- | --------------------------------------------- |
| SYNC_PULSE_MS        | 100 (signals a byte is about to be sent)      |
| SHORT_PULSE_MS       | 30 (0)                                        |
| LONG_PULSE_MS        | 60 (1)                                        |
| PULSE_TOLERANCE      | 2 (the extent that the pulse can be too long) |

## SEQ-based Protocol (Transceiver)

The `transmit_receive_headers/` folder contains a robust communication system with sequence numbers for duplicate detection.

### Protocol Flow

```
Sender                                    Receiver
  │                                          │
  │── DATA (8b) + checksum (2b) + SEQ (1b) ─>│  11 bits after sync
  │                                          │
  │        (checksum valid?)                 │
  │                                          │
  │<── Block parity ACK (2 bits) ───────────│                     Only if checksum valid
  │                                          │
  ✓ Increment SEQ                            ✓ Accept if SEQ != lastSEQ
```

### Transmission Format

```
SENDER OUTPUT (11 bits after sync):
    ┌───────────┐ ┌──┐ ┌────┐ ┌──┐ ┌────┐ ┌──┐ ┌──┐ ┌────┐ ┌──┐ ┌────┐ ┌──┐ ┌────┐ ┌──┐
    │   SYNC    │ │b7│ │ b6 │ │b5│ │ b4 │ │b3│ │b2│ │ b1 │ │b0│ │ c1 │ │c0│ │SEQ │ │  │
────┘   100ms   └─┘  └─┘    └─┘  └─┘    └─┘  └─┘  └─┘    └─┘  └─┘    └─┘  └─┘    └─┘  └──
                  │         DATA (8 bits)         │ CHECKSUM │ SEQ │
                  │   (30ms=0, 60ms=1, +padding)  │ (2 bits) │(1b) │
```

### Sequence Number Protocol

The SEQ bit provides duplicate detection:

1. **Sender** starts with SEQ=0
2. **Sender** transmits byte with current SEQ
3. **Receiver** checks:
   - If checksum invalid: don't send ACK (sender will timeout and retry)
   - If checksum valid: send ACK (block parity)
   - If SEQ == lastSEQ: it's a duplicate, ACK but don't deliver to application
   - If SEQ != lastSEQ: new data, ACK and deliver to application
4. **Sender** receives ACK: increment SEQ (toggle 0↔1)
5. **Sender** timeout: retry with same SEQ

This handles:
- **Lost data**: No ACK sent, sender retries
- **Lost ACK**: Sender retries with same SEQ, receiver detects duplicate
- **Corruption**: Checksum fails, no ACK, sender retries

### Checksums

| Checksum | Bit 0 covers | Bit 1 covers | When used |
|----------|--------------|--------------|-----------|
| Interleaved parity | bits 0,2,4,6 | bits 1,3,5,7 | Sent with data |
| Block parity | bits 0,1,4,5 | bits 2,3,6,7 | ACK response |

### Pulse Types

| Pulse | Duration | Purpose |
|-------|----------|---------|
| Short | 30ms | Bit value 0 |
| Long | 60ms | Bit value 1 |
| Sync | 100ms | Start of data transmission |

### Files

| File | Description |
|------|-------------|
| `transmit.h` | Transmitter class with `sendByte()`, `sendBits()`, SEQ tracking |
| `receive.h` | Receiver class with 11-bit reception, duplicate detection |
| `transceiver.h` | Coordinated transceiver handling the full protocol |
| `transmit_receive_headers.ino` | Example usage |

### Usage

```cpp
#include "transceiver.h"

/* This is a test program/usage for the transceiver.h class.
 * It demonstrates transmission and reception of 8 bit packets, with tolerance against:
 *    packet loss, 
 *    corruption via bit flips, 
 *    retransmissions being double counted 
 * The behaviour is as follows:
 * 1. The receiver listens for the sync pulse
 * 2. The receiver sends a sync pulse
 * 3. The receiver listens for a byte
 * 4. The sender sends a byte
 * 5. The sender listens for an ACK
 * 6. The receiver sends an ACK (which increments the sender's SEQ, and the transmitted byte.)
 * The effect of this is that the receiver will only display receiving bytes counting up from 0.
 */

// ==========================
// CREATE TRANSCEIVER
// ==========================

Transceiver transceiver;
bool transmit = true; // true to transmit test program, false for receiving test program transmitted by another bot
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
    75,     // pre-ACK delay ms (wait before sending ACK to allow transmitter to get into the correct mode)
    false,  // debugProtocol        - prints protocol state changes
    false,  // debugRejectionReason - prints reason for not accepting packet
    false,  // debugStrength        - prints max delta peak
    false,  // debugOutputs         - prints pulse lengths and corresponding interpretation
    false,  // debugInputs          - prints raw delta input to serial plotter as well as behaviour visualisation
    false   // debugStates          - prints receiver's current state every time the state changes
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

```

## Notes
Going for a smaller timescale to up the bitrate seems to run into problems where we struggle to read the pulses due to the latency on reading the IR sensor, as well as delays caused by execution on the microcontroller.
Unfortunately this limits us to a somewhat piddly ~1.4ish Bytes/sec. Additionally, going slower seems to make the connection less fiddly and precise.
Experimentation is encouraged! ~It's probably possible to make the sync pulse like 20ms shorter without compromising accuracy... give it a try!~ Turns out no, that creates issues with reading the first bit.
