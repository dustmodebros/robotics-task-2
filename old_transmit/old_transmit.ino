#define EMIT_PIN 11

#define SYNC_PULSE_MS  100
#define SHORT_PULSE_MS 30     // pulse length for 0
#define LONG_PULSE_MS  60     // pulse length for 1
#define INTER_BIT_MS   10     // space between bits

byte interleavedParity(byte data) {
  byte even_parity = 0;
  byte odd_parity = 0;
  for (int i = 0; i < 8; i += 2) {
    even_parity ^= (data >> i) & 1;
    odd_parity ^= (data >> (i + 1)) & 1;
  }
  return (odd_parity << 1) | even_parity;
}

void sendSync() {
  digitalWrite(EMIT_PIN, HIGH);
  delay(SYNC_PULSE_MS);
  digitalWrite(EMIT_PIN, LOW);
  delay(INTER_BIT_MS);
}

void sendBit(bool bitVal) {
  unsigned long pulseLength = bitVal ? LONG_PULSE_MS : SHORT_PULSE_MS;
  unsigned long padding = bitVal ? 0 : LONG_PULSE_MS - SHORT_PULSE_MS;
  digitalWrite(EMIT_PIN, HIGH);
  delay(pulseLength);
  digitalWrite(EMIT_PIN, LOW);
  delay(padding);
  delay(INTER_BIT_MS);
}

void sendByte(byte data) {
  sendSync();
  
  // Send 8 data bits
  for (int i = 7; i >= 0; i--) {
    sendBit((data >> i) & 1);
  }
  
  // Send 2-bit checksum (interleaved parity)
  byte checksum = interleavedParity(data);
  sendBit((checksum >> 1) & 1);  // odd position parity
  sendBit(checksum & 1);          // even position parity
}

void setup() {
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);
  Serial.begin(9600);
  delay(2000);
}

void loop() {
  byte testByte = 0xAA;
  sendByte(testByte);
  delay(30);
}
