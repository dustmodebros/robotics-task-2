#include "transmit.h"
#include "receive.h"

#define ACK_BYTE 0x06  // ASCII ACK character

class Transceiver {

  private:
    enum Mode {
      MODE_RECEIVING,
      MODE_SENDING,
      MODE_IDLE
    };

    Mode currentMode;

    Transmitter transmitter;
    Receiver receiver;

    // Last received byte
    byte lastReceivedByte;
    bool hasNewByte;

    // Track if we started sending (to detect completion)
    bool sendStarted;

    // Config
    bool autoAck;  // Automatically send ACK after receiving

    // Debug
    bool debugProtocol;

  public:

    void init(int syncMs, int postSyncMs, int shortMs, int longMs,
              int interBitMs, int tolerance, int minRead,
              bool autoAckEnabled = true, bool dbgProtocol = false,
              bool dbgRejection = false, bool dbgStrength = false,
              bool dbgOutputs = false, bool dbgInputs = false,
              bool dbgStates = false) {

      autoAck = autoAckEnabled;
      debugProtocol = dbgProtocol;

      // Initialize transmitter
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, LOW);
      transmitter.init(syncMs, postSyncMs, shortMs, longMs, interBitMs);

      // Initialize receiver
      receiver.init(syncMs, shortMs, longMs, tolerance, minRead,
                    dbgRejection, dbgStrength, dbgOutputs, dbgInputs, dbgStates);

      currentMode = MODE_RECEIVING;
      lastReceivedByte = 0;
      hasNewByte = false;
      sendStarted = false;
    }

    void check() {
      switch (currentMode) {

        case MODE_RECEIVING:
          receiver.check();

          if (receiver.available()) {
            lastReceivedByte = receiver.read();
            hasNewByte = true;

            if (debugProtocol) {
              Serial.print("Transceiver: Received 0x");
              Serial.print(lastReceivedByte, HEX);
              Serial.println(", switching to send mode");
            }

            if (autoAck) {
              currentMode = MODE_SENDING;
              transmitter.sendByte(ACK_BYTE);
              sendStarted = true;
            }
          }
          break;

        case MODE_SENDING:
          transmitter.check();

          // Detect when transmission completes
          if (sendStarted && !transmitter.isSending()) {
            sendStarted = false;
            currentMode = MODE_RECEIVING;

            if (debugProtocol) {
              Serial.println("Transceiver: Send complete, back to receiving");
            }
          }
          break;

        case MODE_IDLE:
          break;
      }
    }

    bool available() {
      return hasNewByte;
    }

    byte read() {
      hasNewByte = false;
      return lastReceivedByte;
    }

    // Manually send a byte (switches to send mode)
    void sendByte(byte b) {
      if (currentMode != MODE_SENDING) {
        currentMode = MODE_SENDING;
        transmitter.sendByte(b);
        sendStarted = true;

        if (debugProtocol) {
          Serial.print("Transceiver: Sending 0x");
          Serial.println(b, HEX);
        }
      }
    }

    // Force back to receiving mode
    void startReceiving() {
      currentMode = MODE_RECEIVING;
      sendStarted = false;
    }

    // Enter idle mode (neither sending nor receiving)
    void setIdle() {
      currentMode = MODE_IDLE;
      sendStarted = false;
    }

    bool isReceiving() {
      return currentMode == MODE_RECEIVING;
    }

    bool isSending() {
      return currentMode == MODE_SENDING;
    }

    bool isIdle() {
      return currentMode == MODE_IDLE;
    }
};
