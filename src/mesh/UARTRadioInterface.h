#pragma once

#include "RadioInterface.h"
#include "UARTRadioProtocol.h"
#if defined(ARCH_RP2040)
#include "hardware/gpio.h"
#endif

/**
 * RadioInterface implementation that communicates with an external WIO-E5
 * LoRa radio bridge over UART using a simple framed protocol.
 *
 * This is used on platforms (like FreeWili/RP2350) where the LoRa radio
 * is not directly connected via SPI but instead via a UART bridge MCU.
 *
 * Polling: `freewili_poll_uart_radio()` (free function below) must be called
 * regularly from the main loop. UARTRadioInterface deliberately does NOT
 * inherit OSThread on this platform — multi-inheritance with RadioInterface
 * was preventing the OSThread base from being scheduled (counters showed
 * runOnce never fired). The free function gets called from main.cpp's loop().
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
    bool bridgeAlive = false;       // set true once we've received any byte from the bridge
    uint32_t lastReinitMs = 0;      // millis() of last re-init attempt while bridge is silent

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

// Hook for the main loop on platforms that don't have an OSThread for this
// interface. The UARTRadioInterface constructor stashes a `this` pointer in a
// file-static and this function calls processUART() on it. Safe to call
// before init() (no-op if the interface hasn't been created yet).
extern "C" void freewili_poll_uart_radio(void);
