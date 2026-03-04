#define LS_LEFT_PIN   4  
#define LS_RIGHT_PIN  5 

class Receiver {

  private:
    enum State {
      STATE_LISTENING,
      STATE_SYNC,
      STATE_MONITORING,
      STATE_RECEIVING,
      STATE_BITS_WAITING,
      STATE_BITS_RECEIVING
    };

    enum ReceiveMode {
      RX_MODE_BYTE,
      RX_MODE_BITS
    };
  
    State currentState;
    ReceiveMode rxMode;
  
    // Data
    byte data;
    byte receivedChecksum;
    byte receivedSeq;
    int receivedBits;
    int expectedBits;
  
    bool byteAvailable;
    byte receivedByte;
    bool checksumValid;

    bool bitsAvailable;
    byte receivedRawBits;

    // Sequence tracking for duplicate detection
    byte lastSeq;
    bool lastSeqValid;  // false until first packet received
  
    // Sensor
    long baselineLeft;
    long baselineRight;
    int activePin;
  
    // Timing
    unsigned long syncPulseStart;
    unsigned long currentRead;
  
    // Config
    int syncPulseMs;
    int shortPulseMs;
    int longPulseMs;
    int pulseTolerance;
    int minReading;

    // Debug flags
    bool debugRejectionReason;
    bool debugStrength;
    bool debugOutputs;
    bool debugInputs;
    bool debugStates;

    // Debug variables
    int debugCurrentlyReading;
    int debugCurrentlySyncing;
    int debugCurrentlyChecksum;
    int debugCurrentlyAckBits;
    int debugMaxRead;

    bool isTransmitting;
  
    unsigned long measureSensor(int pin) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      delayMicroseconds(10);
      pinMode(pin, INPUT);
  
      unsigned long start = micros();
      while (digitalRead(pin) == HIGH) {
        if (micros() - start > 16000) break;
      }
      return micros() - start;
    }
  
    byte interleavedParity(byte d) {
      byte even = 0;
      byte odd = 0;
      for (int i = 0; i < 8; i += 2) {
        even ^= (d >> i) & 1;
        odd  ^= (d >> (i + 1)) & 1;
      }
      return (odd << 1) | even;
    }
  
    void resetState() {
      data = 0;
      receivedChecksum = 0;
      receivedBits = 0;
      receivedRawBits = 0;
      // Don't reset receivedSeq or checksumValid - they need to persist until read by transceiver
      
      switch (rxMode) {
        case RX_MODE_BYTE:
          currentState = STATE_LISTENING;
          break;
        case RX_MODE_BITS:
          currentState = STATE_BITS_WAITING;
          break;
      }
    }
  
    void selectBestPin() {
      unsigned long left = measureSensor(LS_LEFT_PIN);
      unsigned long right = measureSensor(LS_RIGHT_PIN);
  
      long deltaLeft = left - baselineLeft;
      long deltaRight = right - baselineRight;
  
      activePin = (deltaRight > deltaLeft) ? LS_RIGHT_PIN : LS_LEFT_PIN;
    }
  
  public:
  
    void init(int syncMs, int shortMs, int longMs,
              int tolerance, int minRead,
              bool dbgRejection = false, bool dbgStrength = false,
              bool dbgOutputs = false, bool dbgInputs = false,
              bool dbgStates = false) {
  
      syncPulseMs = syncMs;
      shortPulseMs = shortMs;
      longPulseMs = longMs;
      pulseTolerance = tolerance;
      minReading = minRead;
  
      baselineLeft = measureSensor(LS_LEFT_PIN);
      baselineRight = measureSensor(LS_RIGHT_PIN);
  
      activePin = LS_LEFT_PIN;
  
      debugRejectionReason = dbgRejection;
      debugStrength = dbgStrength;
      debugOutputs = dbgOutputs;
      debugInputs = dbgInputs;
      debugStates = dbgStates;

      debugCurrentlyReading = 0;
      debugCurrentlySyncing = 0;
      debugCurrentlyChecksum = 0;
      debugCurrentlyAckBits = 0;
      debugMaxRead = 0;
      isTransmitting = false;

      byteAvailable = false;
      bitsAvailable = false;
      checksumValid = false;
      receivedRawBits = 0;
      receivedSeq = 0;
      expectedBits = 2;

      lastSeq = 0;
      lastSeqValid = false;

      rxMode = RX_MODE_BYTE;
      resetState();

      if (debugInputs) {
        Serial.println("delta, data, sync, checksum, ack_bits (transmitting = -1000)");
      }
    }

    void listenForBits(int numBits) {
      rxMode = RX_MODE_BITS;
      expectedBits = numBits;
      bitsAvailable = false;
      receivedRawBits = 0;
      receivedBits = 0;
      currentState = STATE_BITS_WAITING;
    }

    void listenForByte() {
      rxMode = RX_MODE_BYTE;
      byteAvailable = false;
      resetState();
    }
  
    State check() {
  
      unsigned long now = millis();
  
      unsigned long left = measureSensor(LS_LEFT_PIN);
      unsigned long right = measureSensor(LS_RIGHT_PIN);
  
      baselineLeft  = (baselineLeft * 15 + left) / 16;
      baselineRight = (baselineRight * 15 + right) / 16;
  
      unsigned long measurement =
        (activePin == LS_LEFT_PIN) ? left : right;
  
      long baseline =
        (activePin == LS_LEFT_PIN) ? baselineLeft : baselineRight;
  
      long delta = measurement - baseline;

      State prevState = currentState;
  
      switch (currentState) {
  
        case STATE_LISTENING:
          debugCurrentlyReading = 0;
          receivedBits = 0;
          debugCurrentlySyncing = 0;
          debugCurrentlyChecksum = 0;
          debugCurrentlyAckBits = 0;

          if (delta > minReading) {
            currentState = STATE_SYNC;
            debugCurrentlySyncing = 1000;
            syncPulseStart = now;
          }
          break;
  
        case STATE_SYNC:
          receivedBits = 0;
          debugCurrentlyReading = 0;
          debugCurrentlyChecksum = 0;
          debugCurrentlyAckBits = 0;

          if (delta < 0) {
            debugCurrentlySyncing = 0;
            unsigned long duration = now - syncPulseStart;

            if (debugOutputs) {
              Serial.print("pulse duration for sync: ");
              Serial.println(duration);
            }
  
            if (duration > longPulseMs + pulseTolerance &&
                duration <= syncPulseMs + pulseTolerance) {
              if (debugOutputs) Serial.println("SYNC LOCKED");
              selectBestPin();
              currentState = STATE_MONITORING;
              currentRead = now;
            } else {
              if (debugRejectionReason) Serial.println("REJECTED: Sync pulse incorrect timing");
              resetState();
            }
          }
          break;
  
        case STATE_MONITORING:
          debugCurrentlySyncing = 0;
          debugCurrentlyReading = 0;
          debugCurrentlyChecksum = 0;
          debugCurrentlyAckBits = 0;

          if (now > currentRead + 1000) {
            currentState = STATE_SYNC;
          }
          if (delta > minReading) {
            currentState = STATE_RECEIVING;
            currentRead = now;
            if (receivedBits >= 8) {
              debugCurrentlyChecksum = 1000;
            } else {
              debugCurrentlyReading = 1000;
            }
            if (debugStrength && debugMaxRead < delta) {
              debugMaxRead = delta;
            }
          }
          break;
  
        case STATE_RECEIVING:
          debugCurrentlyAckBits = 0;

          if (delta < 0) {
            debugCurrentlyReading = 0;
            debugCurrentlyChecksum = 0;
  
            unsigned long pulseLen = now - currentRead;

            if (debugOutputs) {
              Serial.print("pulse duration for read: ");
              Serial.println(pulseLen);
            }
  
            if (pulseLen > longPulseMs + pulseTolerance) {
              if (debugRejectionReason) Serial.println("REJECTED: data pulse was too long");
              resetState();
              break;
            }
  
            bool bitValue = (pulseLen > shortPulseMs + pulseTolerance);
  
            receivedBits++;
  
            if (receivedBits <= 8) {
              // Data bits (bits 1-8)
              data = (data << 1) | bitValue;
            } else if (receivedBits <= 10) {
              // Checksum bits (bits 9-10)
              receivedChecksum = (receivedChecksum << 1) | bitValue;
            } else {
              // SEQ bit (bit 11)
              receivedSeq = bitValue;
            }
  
            if (receivedBits == 11) {
              byte expected = interleavedParity(data);
              checksumValid = (receivedChecksum == expected);
  
              if (checksumValid) {
                receivedByte = data;
                byteAvailable = true;

                if (!debugInputs) {
                  Serial.print("received byte: 0x");
                  Serial.print(data, HEX);
                  Serial.print(" (SEQ=");
                  Serial.print(receivedSeq);
                  Serial.print(") from ");
                  Serial.println(activePin == LS_LEFT_PIN ? "Left" : "Right");
                  if (debugStrength) {
                    Serial.print("Max signal strength: ");
                    Serial.println(debugMaxRead);
                  }
                  debugMaxRead = 0;
                }
              } else {
                if (debugRejectionReason) {
                  Serial.print("REJECTED: checksum mismatch - data: 0x");
                  Serial.print(data, HEX);
                  Serial.print(", checksum: 0b");
                  Serial.print(receivedChecksum, BIN);
                  Serial.print(" (expected 0b");
                  Serial.print(expected, BIN);
                  Serial.println(")");
                }
              }
  
              resetState();
            } else {
              currentState = STATE_MONITORING;
            }
          }
          break;

        case STATE_BITS_WAITING:
          debugCurrentlyAckBits = 0;

          if (delta > minReading) {
            currentState = STATE_BITS_RECEIVING;
            currentRead = now;
            debugCurrentlyAckBits = 1000;
            if (debugOutputs) Serial.println("Raw bit pulse detected");
          }
          break;

        case STATE_BITS_RECEIVING:
          if (delta < 0) {
            debugCurrentlyAckBits = 0;
            unsigned long pulseLen = now - currentRead;

            if (debugOutputs) {
              Serial.print("Raw bit pulse duration: ");
              Serial.println(pulseLen);
            }

            if (pulseLen > longPulseMs + pulseTolerance) {
              if (debugRejectionReason) Serial.println("REJECTED: raw bit pulse too long");
              resetState();
              break;
            }

            bool bitValue = (pulseLen > shortPulseMs + pulseTolerance);
            receivedBits++;
            receivedRawBits = (receivedRawBits << 1) | bitValue;

            if (receivedBits >= expectedBits) {
              bitsAvailable = true;
              if (debugOutputs) {
                Serial.print("Raw bits received: 0b");
                Serial.println(receivedRawBits, BIN);
              }
              currentState = STATE_BITS_WAITING;
              receivedBits = 0;
            } else {
              currentState = STATE_BITS_WAITING;
            }
          }
          break;
      }

      if (debugStates) {
        if (prevState != currentState) {
          switch (currentState) {
            case STATE_LISTENING:
              Serial.println("in STATE_LISTENING");
              break;
            case STATE_SYNC:
              Serial.println("in STATE_SYNC");
              break;
            case STATE_MONITORING:
              Serial.println("in STATE_MONITORING");
              break;
            case STATE_RECEIVING:
              Serial.println("in STATE_RECEIVING");
              break;
            case STATE_BITS_WAITING:
              Serial.println("in STATE_BITS_WAITING");
              break;
            case STATE_BITS_RECEIVING:
              Serial.println("in STATE_BITS_RECEIVING");
              break;
          }
        }
      }

      if (debugInputs) {
        Serial.print(delta);
        Serial.print(",\t");
        if (isTransmitting) {
          Serial.print("-1000,\t-1000,\t-1000,\t-1000");
        } else {
          Serial.print(debugCurrentlyReading);
          Serial.print(",\t");
          Serial.print(debugCurrentlySyncing);
          Serial.print(",\t");
          Serial.print(debugCurrentlyChecksum);
          Serial.print(",\t");
          Serial.print(debugCurrentlyAckBits);
        }
        Serial.println();
      }

      return currentState;
    }
  
    bool available() {
      return byteAvailable;
    }
  
    byte read() {
      byteAvailable = false;
      return receivedByte;
    }

    bool bitsReady() {
      return bitsAvailable;
    }

    byte readBits() {
      bitsAvailable = false;
      return receivedRawBits;
    }

    // Check if checksum was valid for last received packet
    bool wasChecksumValid() {
      return checksumValid;
    }

    // Get the SEQ bit from last received packet
    byte getReceivedSeq() {
      return receivedSeq;
    }

    // Check if this is a duplicate (same SEQ as last)
    bool isDuplicate() {
      if (!lastSeqValid) return false;
      return receivedSeq == lastSeq;
    }

    // Update last SEQ after accepting a packet
    void updateLastSeq() {
      lastSeq = receivedSeq;
      lastSeqValid = true;
    }

    ReceiveMode getMode() {
      return rxMode;
    }

    void setTransmitting(bool transmitting) {
      isTransmitting = transmitting;
    }
  };
