// RECEIVE PINS
#define LS_LEFT_PIN 4
#define LS_RIGHT_PIN 5
int active_pin = LS_LEFT_PIN;  // Which pin to use for current byte

// RECEIVER STATE MACHINE
#define STATE_LISTENING 0     // look for pulse
#define STATE_SYNC 1          // if pulse length > LONG_PULSE_MS and < SYNC_PULSE_MS, start monitoring
#define STATE_MONITORING 2    // look for data pulse, and go to Receiving
#define STATE_RECEIVING 3     // decode data by pulse length, and pack into byte. After, go back to Listening
int current_state = 0;

// DEBUG FLAGS AND VARIABLES
#define DEBUG_REJECTION_REASON false
#define DEBUG_STRENGTH false
#define DEBUG_OUTPUTS false    // prints pulse durations (read and sync) to serial monitor
#define DEBUG_INPUTS true    // prints raw sensor readings and indications of sync/read to plotter
#define DEBUG_STATES false     // prints what state the receiver is in
int debug_currently_reading = 0;
int debug_currently_syncing = 0;
int debug_currently_checksum = 0;
int debug_max_read = 0;

// TRANSMIT PARAMETERS
#define SYNC_PULSE_MS 100
#define SHORT_PULSE_MS 30     // pulse length for 0. Don't go lower than 30, as then we run into read speed problems
#define LONG_PULSE_MS  60     // pulse length for 1. 
#define PULSE_TOLERANCE 2

// READ SENSITIVITY
#define MIN_READING 200
long baseline_left = 0;
long baseline_right = 0;

// SYNC VARIABLES
unsigned long sync_pulse_start = 0;

// TIMING VARIABLES
unsigned long current_read;
unsigned long now;

// OUTPUT VARIABLES
byte data = 0x00;
int received_bits = 0;
byte received_checksum = 0;

// BASELINE SENSITIVITY
#define BASELINE_UPDATE_RATIO 16

void setup() {
  Serial.begin(115200);
  delay(2000); //wait for Serial

  if (!DEBUG_INPUTS) {
    Serial.println("Receiver start");
  } else {
    Serial.println("delta, read_status, sync_status, checksum_status"); // If we're using the Serial plotter, print column names
  }

  baseline_left = measureSensor(LS_LEFT_PIN);
  baseline_right = measureSensor(LS_RIGHT_PIN);
}

void reset_state() {
  data = 0;
  received_bits = 0;
  received_checksum = 0;
  current_state = STATE_LISTENING;
}

// Interleaved parity: bit0 = parity of even positions, bit1 = parity of odd positions
byte interleavedParity(byte data) {
  byte even_parity = 0;
  byte odd_parity = 0;
  for (int i = 0; i < 8; i += 2) {
    even_parity ^= (data >> i) & 1;
    odd_parity ^= (data >> (i + 1)) & 1;
  }
  return (odd_parity << 1) | even_parity;
}

unsigned long measureSensor(int pin) {
  // Do digital light sensor read via capacitor method
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10); // should be a small enough block to avoid screwing with anything timing based...
  pinMode(pin, INPUT);

  unsigned long start_time = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start_time > 16000) break; //timeout if read is taking too long (low light)
  }
  return micros() - start_time;
}

void selectBestPin() {
  // Measure both sensors and pick the one with the higher delta
  unsigned long left_measurement = measureSensor(LS_LEFT_PIN);
  unsigned long right_measurement = measureSensor(LS_RIGHT_PIN);

  long left_delta = left_measurement - baseline_left;
  long right_delta = right_measurement - baseline_right;

  if (right_delta > left_delta) {
    active_pin = LS_RIGHT_PIN;
  } else {
    active_pin = LS_LEFT_PIN;
  }
}

void loop() {

  // Measure both sensors and update baselines
  unsigned long measurement_left = measureSensor(LS_LEFT_PIN);
  unsigned long measurement_right = measureSensor(LS_RIGHT_PIN);
  unsigned long now = millis();

  // Slowly track ambient baseline for both sensors
  baseline_left = (baseline_left * (BASELINE_UPDATE_RATIO - 1) + measurement_left) / BASELINE_UPDATE_RATIO;
  baseline_right = (baseline_right * (BASELINE_UPDATE_RATIO - 1) + measurement_right) / BASELINE_UPDATE_RATIO;

  // Use the active pin's measurement and baseline for delta calculation
  unsigned long measurement = (active_pin == LS_LEFT_PIN) ? measurement_left : measurement_right;
  long baseline = (active_pin == LS_LEFT_PIN) ? baseline_left : baseline_right;

  // Work based on a delta to avoid ambient conditions influencing signal quality
  long delta = measurement - baseline;

  // Track prev state in case we want to DEBUG_STATES
  int prev_state = current_state;

  // Depending on what state we're in, different pulses mean different things (sync, data, etc.) so it's state machine time
  switch (current_state) {
    case STATE_LISTENING:
      debug_currently_reading = 0;
      received_bits = 0;
      debug_currently_syncing = 0;
      debug_currently_checksum = 0;

      if (delta > MIN_READING) {
        current_state = STATE_SYNC;
        debug_currently_syncing = 1000;
        sync_pulse_start = now;
      }
      break;
    case STATE_SYNC: {
        received_bits = 0;
        debug_currently_reading = 0;
        debug_currently_checksum = 0;

        // Detect falling edge
        if (delta < 0) {
          debug_currently_syncing = 0;
          unsigned long pulse_duration = now - sync_pulse_start;
          if (DEBUG_OUTPUTS) Serial.print("pulse duration for sync: ");
          if (DEBUG_OUTPUTS) Serial.println(pulse_duration);

          // Require between max sync pulse and min of max data length to avoid locking on incorrect signal timing
          if (pulse_duration > LONG_PULSE_MS and pulse_duration <= SYNC_PULSE_MS + PULSE_TOLERANCE) { // add tolerance on top end because sometimes the pulse is read to be like 1ms longer
            if (DEBUG_OUTPUTS) Serial.println("SYNC LOCKED");
            selectBestPin();  // Select the best pin for this byte
            current_state = STATE_MONITORING;
            current_read = now;
          } else {
            if (DEBUG_REJECTION_REASON) Serial.println("REJECTED: Sync pulse incorrect timing");
            // If pulse isn't a sync, go back to listening state for another potential sync.
            current_state = STATE_LISTENING;
          }
        }
        break;
      }
    case STATE_MONITORING:
      debug_currently_syncing = 0;
      debug_currently_reading = 0;
      debug_currently_checksum = 0;
      if (now > current_read + 1000) {
        current_state = STATE_SYNC;
      }
      if (delta > MIN_READING) {
        current_state = STATE_RECEIVING;
        current_read = now;  // mark pulse start
        if (received_bits >= 8) {
          debug_currently_checksum = 1000;
        } else {
          debug_currently_reading = 1000;
        }
        if (DEBUG_STRENGTH and debug_max_read < delta) {
          debug_max_read = delta;
        }
      }
      break;
    case STATE_RECEIVING: {

        if (delta < 0) {
          debug_currently_reading = 0;
          debug_currently_checksum = 0;
          unsigned long pulseLength = now - current_read;
          if (DEBUG_OUTPUTS) Serial.print("pulse duration for read: ");
          if (DEBUG_OUTPUTS) Serial.println(pulseLength);

          bool bit_r = (pulseLength > SHORT_PULSE_MS + PULSE_TOLERANCE);
          if (pulseLength > LONG_PULSE_MS + PULSE_TOLERANCE) {
            // Reject packet, wait for next sync
            if (DEBUG_REJECTION_REASON) Serial.println("REJECTED: data pulse was too long");
            reset_state();
            break;
          }

          received_bits++;

          if (received_bits <= 8) {
            // Pack received bit into 'data'
            data = (data << 1) | bit_r;
          } else {
            // Pack received bit into checksum (bits 9 and 10)
            received_checksum = (received_checksum << 1) | bit_r;
          }

          if (received_bits == 10) {
            // Validate checksum (interleaved parity)
            byte expected_checksum = interleavedParity(data);
            if (received_checksum != expected_checksum) {
              if (DEBUG_REJECTION_REASON) {
                Serial.print("REJECTED: checksum mismatch - received bits: ");
                Serial.print(data, BIN);
                Serial.print(", received checksum: ");
                Serial.print(received_checksum, BIN);
                Serial.print(" (expected ");
                Serial.print(expected_checksum, BIN);
                Serial.println(")");
              }
              reset_state();
              break;
            }

            if (!DEBUG_INPUTS) {
              Serial.print("received byte: 0x");
              Serial.print(data, BIN);
              Serial.print(" from ");
              Serial.println(active_pin == LS_LEFT_PIN ? "Left" : "Right");
              if (DEBUG_STRENGTH) {
                Serial.print("Max signal strength: ");
                Serial.println(debug_max_read);
              }
              debug_max_read = 0;
            }
            reset_state();
          } else {
            current_state = STATE_MONITORING;
          }
        }
        break;
      }
  }

  //Debug print zone

  if (DEBUG_STATES) {
    if (prev_state != current_state) {
      switch (current_state) {
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

  if (DEBUG_INPUTS) {
    Serial.print(delta);
    Serial.print(",\t");
    Serial.print(debug_currently_reading);
    Serial.print(",\t");
    Serial.print(debug_currently_syncing);
    Serial.print(",\t");
    Serial.println(debug_currently_checksum);
  }

}
