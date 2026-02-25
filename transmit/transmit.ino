#define EMIT_PIN 11

#define SYNC_PULSE_MS  100
#define SHORT_PULSE_MS 30     // pulse length for 0
#define LONG_PULSE_MS  60     // pulse length for 1
#define INTER_BIT_MS   10    // space between bits

void sendSync(int check){
  digitalWrite(EMIT_PIN, HIGH);
  delay(SYNC_PULSE_MS);
  digitalWrite(EMIT_PIN, LOW);
  delay(10);
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
  
  sendSync(SYNC_PULSE_MS);
  for (int i = 7; i >= 0; i--) {
    sendBit((data >> i) & 1);
  }
}

void setup() {
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  byte testByte = 0xAA;  // example, 0x10101010
  sendByte(testByte);
  delay(30);            // 1 second between transmissions
}
