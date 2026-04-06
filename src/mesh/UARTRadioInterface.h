#pragma once

#include "RadioInterface.h"
#include "UARTRadioProtocol.h"

/**
 * RadioInterface implementation that communicates with an external WIO-E5
 * LoRa radio bridge over UART using a simple framed protocol.
 *
 * This is used on platforms (like FreeWili/RP2350) where the LoRa radio
 * is not directly connected via SPI but instead via a UART bridge MCU.
 */
class UARTRadioInterface : public RadioInterface
{
  public:
    UARTRadioInterface();
    virtual ~UARTRadioInterface();

    // RadioInterface overrides
    virtual bool init() override;
    virtual ErrorCode send(meshtastic_MeshPacket *p) override;
    virtual bool reconfigure() override;
    virtual bool canSleep() override;

    /**
     * Calculate LoRa airtime for a given packet length.
     * Uses the formula from Semtech AN1200.13.
     */
    virtual uint32_t getPacketTime(uint32_t totalPacketLen, bool received = false) override;

    /**
     * Called from the main loop to process incoming UART data from the bridge.
     * Parses bytes through a state machine and dispatches complete frames.
     */
    void processUART();

  private:
    // UART RX state machine states
    enum RxState { SYNC1, SYNC2, CMD, LEN_HI, LEN_LO, PAYLOAD, CRC_BYTE };
    RxState rxState = SYNC1;
    uint8_t rxCmd;
    uint16_t rxPayloadLen;
    uint16_t rxPayloadIdx;
    uint8_t rxPayload[UART_RADIO_MAX_PAYLOAD];

    bool txPending = false;
    bool receiving = false;

    /**
     * Send a framed command to the bridge over UART.
     */
    void sendCommand(uint8_t cmd, const uint8_t *payload = nullptr, uint16_t len = 0);

    /**
     * Send the current radio configuration (frequency, bw, sf, cr, power, preamble, syncword)
     * to the bridge as a CMD_RADIO_CONFIGURE frame.
     */
    void sendRadioConfig();

    /**
     * Handle a complete response frame from the bridge.
     */
    void handleResponse(uint8_t cmd, const uint8_t *payload, uint16_t len);

    /**
     * Feed one byte into the RX state machine.
     */
    void parseByte(uint8_t byte);
};
