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
  │<── Block parity ACK (2 bits) ────────────│  Only if checksum valid
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

Transceiver transceiver;

void setup() {
  transceiver.init(100, 50, 30, 60, 10, 2, 200, 500, 75, true);
}

void loop() {
  transceiver.check();
  
  // Receiving
  if (transceiver.available()) {
    byte b = transceiver.read();
    // New data received (duplicates filtered out)
  }
  
  // Sending
  if (someCondition && transceiver.isReceiving()) {
    transceiver.sendByte(0x42);
  }
  
  // Check send result
  if (transceiver.isTransactionComplete()) {
    if (transceiver.wasSuccessful()) {
      // ACK received, SEQ incremented
    } else {
      // Timeout, will retry with same SEQ
    }
    transceiver.clearTransaction();
  }
}
```

## Notes
Going for a smaller timescale to up the bitrate seems to run into problems where we struggle to read the pulses due to the latency on reading the IR sensor, as well as delays caused by execution on the microcontroller.
Unfortunately this limits us to a somewhat piddly ~1.4ish Bytes/sec. Additionally, going slower seems to make the connection less fiddly and precise.
Experimentation is encouraged! ~It's probably possible to make the sync pulse like 20ms shorter without compromising accuracy... give it a try!~ Turns out no, that creates issues with reading the first bit.
