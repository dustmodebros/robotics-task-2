#define EMIT_PIN 11

#define SYNC_PULSE_MS  90
#define SHORT_PULSE_MS 30     // pulse length for 0
#define LONG_PULSE_MS  60     // pulse length for 1
#define INTER_BIT_MS   10    // space between bits

class Transmitter {
  private:
    int SYNC_PULSE_MS;
    int SHORT_PULSE_MS;
    int LONG_PULSE_MS;
    int INTER_BIT_MS;

    uint16_t myPayload;  // 10 bits: 8 data + 2 checksum
    int bitNum;          // counts from 9 down to 0

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

    byte interleavedParity(byte data) {
      byte even_parity = 0;
      byte odd_parity = 0;
      for (int i = 0; i < 8; i += 2) {
        even_parity ^= (data >> i) & 1;
        odd_parity ^= (data >> (i + 1)) & 1;
      }
      return (odd_parity << 1) | even_parity;
    }

    void startInterBit() {
      inter_bit = true;
      digitalWrite(EMIT_PIN, LOW);
      inter_bit_ts = millis();
    }

    void startPadding() {
      digitalWrite(EMIT_PIN, LOW);
      padding = true;
      padding_len_ms = (myPayload >> bitNum) & 0b1 ? 0 : LONG_PULSE_MS - SHORT_PULSE_MS;
      padding_ts = millis();
    }

    void checkPadding() {
      if (!padding || millis() < padding_ts + padding_len_ms) {return;}
      padding = false;
      --bitNum; // move on to the next bit.
      startInterBit();
    }

    void startPulse() {
      pulse_len_ms = (myPayload >> bitNum) & 0b1 ? LONG_PULSE_MS : SHORT_PULSE_MS;
      pulsing = true;
      pulse_ts = millis();
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

      if (bitNum < 0) {
        send_byte_ts = millis();
        sending = false;
        return;
      }
      startPulse(); // Only start pulsing if we still have any bits left to transmit.
    }

    void startSync() {
      syncing = true;
      sync_ts = millis();
      digitalWrite(EMIT_PIN, HIGH);
    }

    void checkSync() {
      if (!syncing || millis() < sync_ts + SYNC_PULSE_MS) {return;}
      syncing = false;
      startInterBit();
    }
  
  public:
    Transmitter() {} // constructor. need this

    void init(int sync_pulse_ms, int short_pulse_ms, int long_pulse_ms, int inter_bit_ms) {
      SYNC_PULSE_MS = sync_pulse_ms;
      SHORT_PULSE_MS = short_pulse_ms;
      LONG_PULSE_MS = long_pulse_ms;
      INTER_BIT_MS = inter_bit_ms;
      sending = false;
      send_byte_ts = 0;

      pulsing = false;
      pulse_ts = 0;
      pulse_len_ms = 0;

      padding = false;
      padding_ts = 0;
      padding_len_ms = 0;

      syncing = false;
      sync_ts = 0;
    
      inter_bit = false;
      inter_bit_ts = 0;
    }
    
    void sendByte(byte data) {
      if (sending || millis() < send_byte_ts + 30) {return;}
      sending = true;
      
      byte checksum = interleavedParity(data);
      myPayload = ((uint16_t)data << 2) | checksum;
      bitNum = 9;
      
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
  transmitter.init(90, 30, 60, 10);
  testByte = 0xAA;

  Serial.begin(9600);
  delay(2000);
}

void loop() {
  transmitter.sendByte(testByte);
  transmitter.check();
}
