#include "PICButtonInput.h"
#include "configuration.h"

// PIC16 buttons use SerialPIO — pins 38/39 are UART1 pins but both hardware UARTs are occupied
#if defined(PIC_UART_TX_PIN) && defined(PIC_UART_RX_PIN)
#include "SerialPIO.h"
static SerialPIO picSerial(PIC_UART_TX_PIN, PIC_UART_RX_PIN, 64);
#endif

PICButtonInput *picButtonInput = nullptr;

// SWD-observable: proves the new rpPICComm parser is syncing + checksum-passing
// the PIC's button frames even with no key pressed (frames stream as a heartbeat).
volatile uint32_t g_freewili_pic_btn_frames = 0;
volatile uint16_t g_freewili_pic_last_buttons = 0;
volatile uint32_t g_freewili_pic_rx_bytes = 0;
// Diagnostic: event byte + payload length of the most recent framed PIC message,
// so a build that still decodes 0 button frames can be told what the PIC actually
// sends (expect event=0xB2, len=20). __attribute__((used)) keeps them past --gc-sections.
volatile uint8_t __attribute__((used)) g_freewili_pic_last_event = 0;
volatile uint16_t __attribute__((used)) g_freewili_pic_last_len = 0;

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
        g_freewili_pic_rx_bytes++;
        switch (picState) {
        case WAIT_SYNC1:
            if (byte == PIC_SYNC1) {
                calcChecksum = byte;
                picState = WAIT_SYNC2;
            }
            break;
        case WAIT_SYNC2:
            // Resync: a second 0xB0 keeps us hunting for the 0x1D that follows.
            if (byte == PIC_SYNC2) {
                calcChecksum += byte;
                picState = WAIT_LEN_LSB;
            } else if (byte == PIC_SYNC1) {
                calcChecksum = byte;
                picState = WAIT_SYNC2;
            } else {
                picState = WAIT_SYNC1;
            }
            break;
        case WAIT_LEN_LSB:
            length = byte;
            calcChecksum += byte;
            picState = WAIT_LEN_MSB;
            break;
        case WAIT_LEN_MSB:
            length |= ((uint16_t)byte << 8);
            calcChecksum += byte;
            picState = WAIT_EVENT;
            break;
        case WAIT_EVENT:
            calcChecksum += byte;
            event = byte;
            g_freewili_pic_last_event = byte;
            g_freewili_pic_last_len = length;
            payloadCount = 0;
            if (byte == PIC_EVENT_STATUS && length >= 2 && length <= 512) {
                // Unified status frame: first two payload bytes are the button
                // bitmask; remaining status bytes are consumed to stay aligned.
                picState = WAIT_BTN_LOW;
            } else if (length <= 512) {
                // Any other framed event: consume the payload to stay aligned,
                // then discard at the checksum stage (event != STATUS).
                picState = length ? WAIT_PAYLOAD : WAIT_CK_LSB;
            } else {
                picState = WAIT_SYNC1; // implausible length — resync
            }
            break;
        case WAIT_BTN_LOW:
            // First payload byte = buttons.all bits 0..7.
            btnLow = byte;
            calcChecksum += byte;
            payloadCount = 1;
            picState = WAIT_BTN_HIGH;
            break;
        case WAIT_BTN_HIGH:
            btnHigh = byte; // second payload byte = buttons.all bits 8..15
            calcChecksum += byte;
            payloadCount = 2;
            // Consume any remaining status bytes (battery/gpio) before checksum.
            picState = (length > 2) ? WAIT_PAYLOAD : WAIT_CK_LSB;
            break;
        case WAIT_PAYLOAD:
            calcChecksum += byte;
            if (++payloadCount >= length)
                picState = WAIT_CK_LSB;
            break;
        case WAIT_CK_LSB:
            rxChecksum = byte;
            picState = WAIT_CK_MSB;
            break;
        case WAIT_CK_MSB:
            rxChecksum |= ((uint16_t)byte << 8);
            if (rxChecksum == calcChecksum && event == PIC_EVENT_STATUS) {
                uint16_t buttons = ((uint16_t)btnHigh << 8) | btnLow;
                g_freewili_pic_btn_frames++;
                g_freewili_pic_last_buttons = buttons;
                processButtonChange(buttons);
                prevButtons = buttons;
            }
            picState = WAIT_SYNC1;
            break;
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
