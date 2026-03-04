#include "transmit.h"
#include "receive.h"

class Transceiver {

  private:
    enum Mode {
      MODE_IDLE,
      // Responder states (receiving data first)
      MODE_RESP_RECEIVING,
      MODE_RESP_PRE_ACK_DELAY,
      MODE_RESP_SENDING_ACK,
      // Initiator states (sending data first)
      MODE_INIT_SENDING_DATA,
      MODE_INIT_WAITING_ACK,
      MODE_INIT_INTER_PACKET_DELAY
    };

    Mode currentMode;

    Transmitter transmitter;
    Receiver receiver;

    // Data tracking
    byte dataToSend;
    byte lastReceivedByte;
    byte pendingAck;
    bool hasNewByte;
    bool sendStarted;

    // Transaction result
    bool transactionSuccess;
    bool transactionComplete;

    // Timeout tracking
    unsigned long waitStartTime;
    unsigned long ackTimeoutMs;
    unsigned long preAckDelayMs;
    unsigned long interPacketDelayMs;

    // Debug
    bool debugProtocol;

    // Block parity for ACK
    byte blockParity(byte d) {
      byte groupA = ((d >> 0) & 1) ^ ((d >> 1) & 1) ^ ((d >> 4) & 1) ^ ((d >> 5) & 1);
      byte groupB = ((d >> 2) & 1) ^ ((d >> 3) & 1) ^ ((d >> 6) & 1) ^ ((d >> 7) & 1);
      return (groupB << 1) | groupA;
    }

  public:

    void init(int syncMs, int postSyncMs, int shortMs, int longMs,
              int interBitMs, int tolerance, int minRead,
              unsigned long ackTimeout = 500,
              unsigned long preAckDelay = 75,
              unsigned long interPacketDelay = 50,
              bool dbgProtocol = false,
              bool dbgRejection = false, bool dbgStrength = false,
              bool dbgOutputs = false, bool dbgInputs = false,
              bool dbgStates = false) {

      ackTimeoutMs = ackTimeout;
      preAckDelayMs = preAckDelay;
      interPacketDelayMs = interPacketDelay;
      debugProtocol = dbgProtocol;

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, LOW);
      transmitter.init(syncMs, postSyncMs, shortMs, longMs, interBitMs);

      receiver.init(syncMs, shortMs, longMs, tolerance, minRead,
                    dbgRejection, dbgStrength, dbgOutputs, dbgInputs, dbgStates);

      currentMode = MODE_RESP_RECEIVING;
      lastReceivedByte = 0;
      hasNewByte = false;
      sendStarted = false;
      transactionSuccess = false;
      transactionComplete = false;
      pendingAck = 0;
    }

    void check() {
      unsigned long now = millis();

      switch (currentMode) {

        // ========== RESPONDER STATES ==========

        case MODE_RESP_RECEIVING:
          receiver.check();

          if (receiver.available()) {
            lastReceivedByte = receiver.read();
            bool checksumOk = receiver.wasChecksumValid();
            bool isDup = receiver.isDuplicate();

            if (checksumOk) {
              pendingAck = blockParity(lastReceivedByte);

              if (debugProtocol) {
                Serial.print("Transceiver: Received 0x");
                Serial.print(lastReceivedByte, HEX);
                Serial.print(" SEQ=");
                Serial.print(receiver.getReceivedSeq());
                if (isDup) {
                  Serial.println(" (DUPLICATE - will ACK but ignore)");
                } else {
                  Serial.println(" (NEW - will ACK)");
                }
              }

              currentMode = MODE_RESP_PRE_ACK_DELAY;
              waitStartTime = now;
            } else {
              if (debugProtocol) {
                Serial.println("Transceiver: Checksum invalid, not sending ACK");
              }
            }
          }
          break;

        case MODE_RESP_PRE_ACK_DELAY:
          if (now - waitStartTime >= preAckDelayMs) {
            if (debugProtocol) {
              Serial.print("Transceiver: Sending ACK 0b");
              Serial.println(pendingAck, BIN);
            }

            currentMode = MODE_RESP_SENDING_ACK;
            receiver.setTransmitting(true);
            transmitter.sendBits(pendingAck, 2);
            sendStarted = true;
          }
          break;

        case MODE_RESP_SENDING_ACK:
          transmitter.check();

          if (sendStarted && !transmitter.isSending()) {
            sendStarted = false;
            receiver.setTransmitting(false);
            
            bool isDup = receiver.isDuplicate();
            
            if (!isDup) {
              hasNewByte = true;
              receiver.updateLastSeq();
            }
            
            transactionSuccess = true;
            transactionComplete = true;
            currentMode = MODE_RESP_RECEIVING;

            if (debugProtocol) {
              if (isDup) {
                Serial.println("Transceiver: ACK sent (duplicate ignored)");
              } else {
                Serial.print("Transceiver: ACK sent, new data: 0x");
                Serial.println(lastReceivedByte, HEX);
              }
            }
          }
          break;

        // ========== INITIATOR STATES ==========

        case MODE_INIT_SENDING_DATA:
          transmitter.check();

          if (sendStarted && !transmitter.isSending()) {
            sendStarted = false;
            receiver.setTransmitting(false);
            currentMode = MODE_INIT_WAITING_ACK;
            waitStartTime = now;

            receiver.listenForBits(2);

            if (debugProtocol) {
              Serial.println("Transceiver: Data sent, waiting for ACK");
            }
          }
          break;

        case MODE_INIT_WAITING_ACK:
          receiver.check();

          if (receiver.bitsReady()) {
            byte receivedAck = receiver.readBits();

            transmitter.ackReceived();
            transactionSuccess = true;
            transactionComplete = true;

            if (debugProtocol) {
              Serial.print("Transceiver: ACK received (0b");
              Serial.print(receivedAck, BIN);
              Serial.println("), waiting inter-packet delay");
            }

            // Enter inter-packet delay before returning to receive mode
            currentMode = MODE_INIT_INTER_PACKET_DELAY;
            waitStartTime = now;
          }
          else if (now - waitStartTime > ackTimeoutMs) {
            transactionSuccess = false;
            transactionComplete = true;
            receiver.listenForByte();
            currentMode = MODE_RESP_RECEIVING;

            if (debugProtocol) {
              Serial.println("Transceiver: ACK timeout, transaction failed");
            }
          }
          break;

        case MODE_INIT_INTER_PACKET_DELAY:
          if (now - waitStartTime >= interPacketDelayMs) {
            receiver.listenForByte();
            currentMode = MODE_RESP_RECEIVING;

            if (debugProtocol) {
              Serial.println("Transceiver: Inter-packet delay complete");
            }
          }
          break;

        case MODE_IDLE:
          break;
      }
    }

    void sendByte(byte b) {
      if (currentMode == MODE_RESP_RECEIVING || currentMode == MODE_IDLE) {
        dataToSend = b;
        transactionComplete = false;
        transactionSuccess = false;

        currentMode = MODE_INIT_SENDING_DATA;
        receiver.setTransmitting(true);
        transmitter.sendByte(b);
        sendStarted = true;

        if (debugProtocol) {
          Serial.print("Transceiver: Sending 0x");
          Serial.print(b, HEX);
          Serial.print(" with SEQ=");
          Serial.println(transmitter.getSeq());
        }
      }
    }

    bool available() {
      return hasNewByte;
    }

    byte read() {
      hasNewByte = false;
      return lastReceivedByte;
    }

    bool isTransactionComplete() {
      return transactionComplete;
    }

    bool wasSuccessful() {
      return transactionSuccess;
    }

    void clearTransaction() {
      transactionComplete = false;
      transactionSuccess = false;
    }

    void startReceiving() {
      receiver.listenForByte();
      currentMode = MODE_RESP_RECEIVING;
      sendStarted = false;
    }

    bool isReceiving() {
      return currentMode == MODE_RESP_RECEIVING;
    }

    bool isSending() {
      return currentMode == MODE_INIT_SENDING_DATA ||
             currentMode == MODE_INIT_WAITING_ACK ||
             currentMode == MODE_INIT_INTER_PACKET_DELAY;
    }
};
