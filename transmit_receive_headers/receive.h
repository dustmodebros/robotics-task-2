
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
              int tolerance, int minRead) {
  
      syncPulseMs = syncMs;
      shortPulseMs = shortMs;
      longPulseMs = longMs;
      pulseTolerance = tolerance;
      minReading = minRead;
  
      baselineLeft = measureSensor(LS_LEFT_PIN);
      baselineRight = measureSensor(LS_RIGHT_PIN);
  
      activePin = LS_LEFT_PIN;
  
      byteAvailable = false;
      resetState();
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
  
      switch (currentState) {
  
        case STATE_LISTENING:
          if (delta > minReading) {
            currentState = STATE_SYNC;
            syncPulseStart = now;
          }
          break;
  
        case STATE_SYNC:
          if (delta < 0) {
            unsigned long duration = now - syncPulseStart;
  
            if (duration > longPulseMs &&
                duration <= syncPulseMs + pulseTolerance) {
              // TODO do we still want to do this if we're doing both?
              selectBestPin();
              currentState = STATE_MONITORING;
              currentRead = now;
            } else {
              resetState();
            }
          }
          break;
  
        case STATE_MONITORING:
          if (delta > minReading) {
            currentState = STATE_RECEIVING;
            currentRead = now;
          }
          break;
  
        case STATE_RECEIVING:
          if (delta < 0) {
  
            unsigned long pulseLen = now - currentRead;
  
            if (pulseLen > longPulseMs + pulseTolerance) {
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
              }
  
              resetState();
            } else {
              currentState = STATE_MONITORING;
            }
          }
          break;
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