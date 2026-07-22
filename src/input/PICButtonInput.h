#pragma once

#include "InputBroker.h"
#include "concurrency/OSThread.h"

class PICButtonInput : public Observable<const InputEvent *>, public concurrency::OSThread
{
  public:
    PICButtonInput();
    void init();

  protected:
    int32_t runOnce() override;

  private:
    // PIC UART protocol (rpPICComm, freewilimain rmpLib/rpPICComm.cpp). Frame:
    // 0xB0 0x1D | len_lo len_hi | event | payload | ck_lo ck_hi. Checksum = sum
    // of every byte from sync1 through the last payload byte, 16-bit. The PIC
    // streams ONE unified status frame autonomously: event 0xB2 (STATUS), payload
    // = 20-byte device_status_t whose first uint16 (bytes 0..1, little-endian) is
    // the button bitmask. (The old split protocol — buttons 0xB5/len2, battery
    // 0xB1 — is obsolete; the PIC was unified to 0xB2 and this parser was never
    // updated, which is why every frame was discarded and buttons never worked.)
    // We only consume the button bitmask; the rest of the status payload is
    // skipped to stay frame-aligned (Meshtastic reads the BQ27441 over I2C).
    enum PicRxState {
        WAIT_SYNC1,
        WAIT_SYNC2,
        WAIT_LEN_LSB,
        WAIT_LEN_MSB,
        WAIT_EVENT,
        WAIT_BTN_LOW,
        WAIT_BTN_HIGH,
        WAIT_PAYLOAD,
        WAIT_CK_LSB,
        WAIT_CK_MSB,
    };
    PicRxState picState = WAIT_SYNC1;

    uint16_t length = 0;
    uint16_t calcChecksum = 0;
    uint16_t rxChecksum = 0;
    uint8_t event = 0;
    uint8_t btnLow = 0;  // first button byte on the wire = buttons.all bits 0..7
    uint8_t btnHigh = 0; // second button byte = buttons.all bits 8..15
    uint16_t payloadCount = 0;
    uint16_t prevButtons = 0;

    static const uint8_t PIC_SYNC1 = 0xB0;
    static const uint8_t PIC_SYNC2 = 0x1D;
    static const uint8_t PIC_EVENT_STATUS = 0xB2; // unified device_status_t frame
    static const uint16_t PIC_STATUS_LEN = 20;    // sizeof(device_status_t), buttons at [0..1]

    // Button bit positions (match rpPICComm buttonsstates_t exactly).
    static const uint16_t BTN_GREY = (1 << 0);
    static const uint16_t BTN_YELLOW = (1 << 1);
    static const uint16_t BTN_GREEN = (1 << 2);
    static const uint16_t BTN_BLUE = (1 << 3);
    static const uint16_t BTN_RED = (1 << 4);
    static const uint16_t BTN_CENTER = (1 << 5);
    static const uint16_t BTN_DOWN = (1 << 6);
    static const uint16_t BTN_RIGHT = (1 << 7);
    static const uint16_t BTN_UP = (1 << 8);
    static const uint16_t BTN_LEFT = (1 << 9);
    static const uint16_t BTN_HOME = (1 << 10);
    static const uint16_t BTN_OK = (1 << 11);
    static const uint16_t BTN_CANCEL = (1 << 12);
    static const uint16_t BTN_AI = (1 << 13);

    void emitEvent(input_broker_event event);
    void processButtonChange(uint16_t buttons);
};

extern PICButtonInput *picButtonInput;
