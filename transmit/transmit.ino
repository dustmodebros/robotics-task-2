#define EMIT_PIN 11

// #define SYNC_PULSE_MS  100
// #define SHORT_PULSE_MS 30     // pulse length for 0
// #define LONG_PULSE_MS  60     // pulse length for 1
// #define INTER_BIT_MS   10    // space between bits

class Transmitter {
  private:
    int SYNC_PULSE_MS;
    int SHORT_PULSE_MS;
    int LONG_PULSE_MS;
    int INTER_BIT_MS;

    byte myData;
    int bitNum;

    bool sending;
    unsigned long send_byte_ts;

    bool pulsing;
    unsigned long pulse_ts;
    unsigned long pulse_len_ms;

    bool padding;
    unsigned long padding_ts;
    unsigned long padding_len_ms;

    bool syncing;
    unsigned long sync_ts;
    
    bool inter_bit;
    unsigned long inter_bit_ts;

    void startInterBit() {
      inter_bit = true;
      digitalWrite(EMIT_PIN, LOW);
      inter_bit_ts = millis();
    }

    void startPadding() {
      digitalWrite(EMIT_PIN, LOW);
      padding = true;
      padding_len_ms = (myData >> bitNum) & 0b1 ? 0 : LONG_PULSE_MS - SHORT_PULSE_MS;
      padding_ts = millis();
    }

    void checkPadding() {
      if (!padding || millis() < padding_ts + padding_len_ms) {return;}
      padding = false;
      --bitNum; // move on to the next bit.
      startInterBit();
    }

    void startPulse() {
      pulse_len_ms = (myData >> bitNum) & 0b1 ? LONG_PULSE_MS : SHORT_PULSE_MS;
      pulsing = true;
      digitalWrite(EMIT_PIN, HIGH);
    }

    void checkPulse() {
      if (!pulsing || millis() < pulse_ts + pulse_len_ms) {return;}
      pulsing = false;
      startPadding();
    }

    void checkInterBit() {
      if (!inter_bit || millis() < inter_bit_ts + INTER_BIT_MS) {return;}
      inter_bit = false;

      if (bitNum < 0) {return;}
      startPulse(); // Only start pulsing if we still have any bits left to transmit.
    }

    void startSync() {
      syncing = true;
      sync_ts = millis();
    }

    void checkSync() {
      if (!syncing || millis() < sync_ts + SYNC_PULSE_MS) {return;}
      syncing = false;
      startInterBit();
    }
  
  public:
    Transmitter() {} // constructor. need this

    void init (int sync_pulse_ms, int short_pulse_ms, int long_pulse_ms, int inter_bit_ms) {
      SYNC_PULSE_MS = sync_pulse_ms;
      SHORT_PULSE_MS = short_pulse_ms;
      LONG_PULSE_MS = long_pulse_ms;
      INTER_BIT_MS = inter_bit_ms;
      send_byte_ts = 0;
      sending = false;
    }
    
    // 1 second between transmissions
    void sendByte(byte data) {
      if (sending || send_byte_ts < millis() + 30) {return;}
      sending = true;
      myData = data;
      bitNum = 7;
      startSync();
    }

    void check() {
      checkSync();
      checkInterBit();
      checkPulse();
      checkPadding();
    }
};

Transmitter transmitter;
byte testByte;

void setup() {
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);
  transmitter.init(100, 30, 60, 10);
  testByte = 0xAA;

  Serial.begin(9600);
  delay(1000);
}

void loop() {
  transmitter.sendByte(testByte);
  transmitter.check();
}
