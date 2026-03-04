#define EMIT_PIN 11

class Transmitter {
  private:
    int syncPulseMs;
    int postSyncMs;
    int shortPulseMs;
    int longPulseMs;
    int interBitMs;

    uint16_t myPayload;
    int bitNum;
    int totalBits;

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

    bool post_sync;
    unsigned long post_sync_ts;
    
    bool inter_bit;
    unsigned long inter_bit_ts;

    bool skipSync;

    // Sequence number (alternating 0/1)
    byte currentSeq;

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
      padding_len_ms = (myPayload >> bitNum) & 0b1 ? 0 : longPulseMs - shortPulseMs;
      padding_ts = millis();
    }

    void checkPadding() {
      if (!padding || millis() < padding_ts + padding_len_ms) {return;}
      padding = false;
      --bitNum;
      startInterBit();
    }

    void startPulse() {
      pulse_len_ms = (myPayload >> bitNum) & 0b1 ? longPulseMs : shortPulseMs;
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
      if (!inter_bit || millis() < inter_bit_ts + interBitMs) {return;}
      inter_bit = false;

      if (bitNum < 0) {
        send_byte_ts = millis();
        sending = false;
        return;
      }
      startPulse();
    }

    void startPostSync() {
      post_sync = true;
      post_sync_ts = millis();
      digitalWrite(EMIT_PIN, LOW);
    }

    void checkPostSync() {
      if (!post_sync || millis() < post_sync_ts + postSyncMs) {return;}
      post_sync = false;
      startPulse();
    }

    void startSync() {
      syncing = true;
      sync_ts = millis();
      digitalWrite(EMIT_PIN, HIGH);
    }

    void checkSync() {
      if (!syncing || millis() < sync_ts + syncPulseMs) {return;}
      syncing = false;
      startPostSync();
    }
  
  public:
    Transmitter() {}

    void init(int sync_pulse_ms, int post_sync_ms, int short_pulse_ms, int long_pulse_ms, int inter_bit_ms) {
      syncPulseMs = sync_pulse_ms;
      postSyncMs = post_sync_ms;
      shortPulseMs = short_pulse_ms;
      longPulseMs = long_pulse_ms;
      interBitMs = inter_bit_ms;
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

      post_sync = false;
      post_sync_ts = 0;
    
      inter_bit = false;
      inter_bit_ts = 0;

      skipSync = false;
      totalBits = 11;
      currentSeq = 0;
    }
    
    // Send a full byte with sync pulse, interleaved parity, and SEQ bit
    // Format: [8 data bits] [2 checksum bits] [1 SEQ bit] = 11 bits
    void sendByte(byte data) {
      if (sending || millis() < send_byte_ts + 30) {return;}
      sending = true;
      skipSync = false;
      
      byte checksum = interleavedParity(data);
      // Payload: data(8) | checksum(2) | seq(1) = 11 bits
      myPayload = ((uint16_t)data << 3) | (checksum << 1) | currentSeq;
      bitNum = 10;  // 11 bits, 0-indexed from 10
      totalBits = 11;
      
      startSync();
    }

    // Send raw bits without sync (for ACK)
    void sendBits(byte bits, int numBits) {
      if (sending || millis() < send_byte_ts + 30) {return;}
      sending = true;
      skipSync = true;
      
      myPayload = bits;
      bitNum = numBits - 1;
      totalBits = numBits;
      
      startPulse();
    }

    void check() {
      checkSync();
      checkPostSync();
      checkInterBit();
      checkPulse();
      checkPadding();
    }

    bool isSending() {
      return sending;
    }

    // Called when ACK is received - increment SEQ for next transmission
    void ackReceived() {
      currentSeq = currentSeq ^ 1;  // Toggle between 0 and 1
    }

    byte getSeq() {
      return currentSeq;
    }
};
