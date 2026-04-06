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
    // PIC UART protocol state machine
    enum PicRxState { WAIT_HEADER1, WAIT_HEADER2, WAIT_LSB, WAIT_MSB };
    PicRxState picState = WAIT_HEADER1;

    uint16_t prevButtons = 0;
    uint8_t lsb = 0;

    static const uint8_t PIC_SYNC1 = 0xC0;
    static const uint8_t PIC_SYNC2 = 0xC5;

    // Button bit positions
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
