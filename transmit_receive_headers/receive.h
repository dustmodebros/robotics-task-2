

#define LS_LEFT_PIN   4  
#define LS_RIGHT_PIN  5 

class Receiver {

  private:
    enum State {
      STATE_LISTENING,
      STATE_SYNC,
      STATE_MONITORING,
      STATE_RECEIVING
    };
  
    State currentState;
  
    // Data
    byte data;
    byte receivedChecksum;
    int receivedBits;
  
    bool byteAvailable;
    byte receivedByte;
  
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
    int debugMaxRead;
  
    
  
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
      currentState = STATE_LISTENING;
    }
  
    // TODO Still include?
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
      debugMaxRead = 0;

      byteAvailable = false;
      resetState();

      if (debugInputs) {
        Serial.println("delta, read_status, sync_status, checksum_status");
      }
    }
  
    State check() {
  
      unsigned long now = millis();
  
      // Measure both sensors
      unsigned long left = measureSensor(LS_LEFT_PIN);
      unsigned long right = measureSensor(LS_RIGHT_PIN);
  
      // Slowly update baselines
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

          if (delta < 0) {
            debugCurrentlySyncing = 0;
            unsigned long duration = now - syncPulseStart;

            if (debugOutputs) {
              Serial.print("pulse duration for sync: ");
              Serial.println(duration);
            }
  
            if (duration > longPulseMs &&
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
  
            bool bitValue =
              (pulseLen > shortPulseMs + pulseTolerance);
  
            receivedBits++;
  
            if (receivedBits <= 8) {
              data = (data << 1) | bitValue;
            } else {
              receivedChecksum =
                (receivedChecksum << 1) | bitValue;
            }
  
            if (receivedBits == 10) {
  
              byte expected = interleavedParity(data);
  
              if (receivedChecksum == expected) {
                receivedByte = data;
                byteAvailable = true;

                if (!debugInputs) {
                  Serial.print("received byte: 0x");
                  Serial.print(data, BIN);
                  Serial.print(" from ");
                  Serial.println(activePin == LS_LEFT_PIN ? "Left" : "Right");
                  if (debugStrength) {
                    Serial.print("Max signal strength: ");
                    Serial.println(debugMaxRead);
                  }
                  debugMaxRead = 0;
                }
              } else {
                if (debugRejectionReason) {
                  Serial.print("REJECTED: checksum mismatch - received bits: ");
                  Serial.print(data, BIN);
                  Serial.print(", received checksum: ");
                  Serial.print(receivedChecksum, BIN);
                  Serial.print(" (expected ");
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
          }
        }
      }

      if (debugInputs) {
        Serial.print(delta);
        Serial.print(",\t");
        Serial.print(debugCurrentlyReading);
        Serial.print(",\t");
        Serial.print(debugCurrentlySyncing);
        Serial.print(",\t");
        Serial.println(debugCurrentlyChecksum);
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


  };
