#include "PICButtonInput.h"
#include "configuration.h"

// PIC16 buttons use SerialPIO — pins 38/39 are UART1 pins but both hardware UARTs are occupied
#if defined(PIC_UART_TX_PIN) && defined(PIC_UART_RX_PIN)
#include "SerialPIO.h"
static SerialPIO picSerial(PIC_UART_TX_PIN, PIC_UART_RX_PIN, 64);
#endif

PICButtonInput *picButtonInput = nullptr;

PICButtonInput::PICButtonInput() : concurrency::OSThread("PICButton")
{
}

void PICButtonInput::init()
{
#if defined(PIC_UART_RX_PIN) && defined(PIC_UART_TX_PIN)
    picSerial.begin(PIC_UART_BAUD);
    LOG_INFO("PICButtonInput initialized on RX=%d TX=%d baud=%d (SerialPIO)", PIC_UART_RX_PIN, PIC_UART_TX_PIN, PIC_UART_BAUD);
#endif
}

int32_t PICButtonInput::runOnce()
{
#if defined(PIC_UART_RX_PIN)
    while (picSerial.available()) {
        uint8_t byte = picSerial.read();
        switch (picState) {
        case WAIT_HEADER1:
            if (byte == PIC_SYNC1)
                picState = WAIT_HEADER2;
            break;
        case WAIT_HEADER2:
            // Match the rpPICComm resync behavior: a second 0xC0 keeps us in
            // WAIT_HEADER2 in case bytes got chopped; otherwise need 0xC5 to advance.
            if (byte == PIC_SYNC2)
                picState = WAIT_LSB;
            else if (byte == PIC_SYNC1)
                picState = WAIT_HEADER2;
            else
                picState = WAIT_HEADER1;
            break;
        case WAIT_LSB:
            // Wire format from PIC is BIG-ENDIAN: byte after 0xC0 0xC5 is the
            // MSB. (Field is named `lsb` per the old draft; despite the name
            // we store the MSB here, matching freewilimain rpPICComm.cpp.)
            lsb = byte;
            picState = WAIT_MSB;
            break;
        case WAIT_MSB: {
            uint16_t buttons = ((uint16_t)lsb << 8) | byte;
            processButtonChange(buttons);
            prevButtons = buttons;
            picState = WAIT_HEADER1;
            break;
        }
        }
    }
#endif
    return 20; // Poll every 20ms
}

void PICButtonInput::emitEvent(input_broker_event event)
{
    InputEvent e;
    e.source = "picbutton";
    e.inputEvent = event;
    e.kbchar = 0;
    e.touchX = 0;
    e.touchY = 0;
    notifyObservers(&e);
}

void PICButtonInput::processButtonChange(uint16_t buttons)
{
    uint16_t pressed = buttons & ~prevButtons;

    if (pressed & BTN_UP)
        emitEvent(INPUT_BROKER_UP);
    if (pressed & BTN_DOWN)
        emitEvent(INPUT_BROKER_DOWN);
    if (pressed & BTN_LEFT)
        emitEvent(INPUT_BROKER_LEFT);
    if (pressed & BTN_RIGHT)
        emitEvent(INPUT_BROKER_RIGHT);
    if (pressed & BTN_CENTER)
        emitEvent(INPUT_BROKER_SELECT);
    if (pressed & BTN_OK)
        emitEvent(INPUT_BROKER_SELECT);
    if (pressed & BTN_CANCEL)
        emitEvent(INPUT_BROKER_CANCEL);
    if (pressed & BTN_HOME)
        emitEvent(INPUT_BROKER_HOME);
    if (pressed & BTN_AI)
        emitEvent(INPUT_BROKER_MESSAGES);
}
