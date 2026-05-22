#pragma once

#include "RadioInterface.h"
#include "UARTRadioProtocol.h"
#if defined(ARCH_RP2040)
#include "hardware/gpio.h"
#endif

// RadioInterface backed by a UART-framed bridge MCU (WIO-E5).
class UARTRadioInterface : public RadioInterface
{
  public:
    UARTRadioInterface();
    virtual ~UARTRadioInterface();

    virtual bool init() override;
    virtual ErrorCode send(meshtastic_MeshPacket *p) override;
    virtual bool reconfigure() override;
    virtual bool canSleep() override;

    virtual uint32_t getPacketTime(uint32_t totalPacketLen, bool received = false) override;

    void processUART();

  private:
    enum RxState { SYNC1, SYNC2, CMD, LEN_HI, LEN_LO, PAYLOAD, CRC_BYTE };
    RxState rxState = SYNC1;
    uint8_t rxCmd;
    uint16_t rxPayloadLen;
    uint16_t rxPayloadIdx;
    uint8_t rxPayload[UART_RADIO_MAX_PAYLOAD];

    bool txPending = false;
    bool receiving = false;
    bool bridgeAlive = false;
    uint32_t lastReinitMs = 0;

    void sendCommand(uint8_t cmd, const uint8_t *payload = nullptr, uint16_t len = 0);
    void sendRadioConfig();
    void handleResponse(uint8_t cmd, const uint8_t *payload, uint16_t len);
    void parseByte(uint8_t byte);
};

extern "C" void freewili_poll_uart_radio(void);
