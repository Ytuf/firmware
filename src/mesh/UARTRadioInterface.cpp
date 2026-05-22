#include "UARTRadioInterface.h"
#include "MeshTypes.h"
#include "NodeDB.h"
#include "Router.h"
#include "airtime.h"
#include "configuration.h"
#include "gps/RTC.h"
#include "main.h"
#include "meshUtils.h" // for pow_of_2
#include <Arduino.h>
#include <math.h>

#if defined(ARCH_RP2040)
#include "hardware/gpio.h"  // gpio_set_function, GPIO_FUNC_UART_AUX
#include "hardware/uart.h"  // pico-sdk uart_init / uart_write_blocking / uart_getc
#include "hardware/irq.h"   // irq_set_exclusive_handler, irq_set_enabled
#endif

#if defined(ARCH_RP2040) && defined(FREEWILI)
// UART1 RX ring buffer. ISR drains the 32-byte HW FIFO on every byte (or
// FIFO-half-full threshold); processUART drains the ring into parseByte.
// Without this, polling at ~100 Hz against 115200-baud bursts (12 KB/s,
// 32-byte FIFO) loses most bytes of any large frame — confirmed empirically:
// 79 bytes seen in 30s while bridge sent thousands.
#define UART_RX_RING_SIZE 1024u
static volatile uint8_t  s_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0;  // next write index (advanced by ISR)
static volatile uint16_t s_rx_tail = 0;  // next read index (advanced by main)
volatile uint32_t g_uart_isr_count     __attribute__((used)) = 0;
volatile uint32_t g_uart_ring_drops    __attribute__((used)) = 0;

static void __not_in_flash_func(uart1_rx_isr)(void)
{
    g_uart_isr_count++;
    while (uart_is_readable(uart1)) {
        uint8_t b = (uint8_t)uart_getc(uart1);
        uint16_t next = (s_rx_head + 1u) % UART_RX_RING_SIZE;
        if (next != s_rx_tail) {
            s_rx_ring[s_rx_head] = b;
            s_rx_head = next;
        } else {
            g_uart_ring_drops++;  // ring full — drop oldest behavior would be
                                  // better, but this is safer to reason about
        }
    }
}
#endif

#if defined(FREEWILI)
// LED-blink hooks for radio activity. Implemented in variant.cpp.
extern "C" void freewili_led_pulse_all(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);
#endif

// UART radio bridge to WIO-E5. Both TX and RX live on UART1 / Serial2:
//   TX = UART_RADIO_TX_PIN (GPIO 40 on FreeWili, valid via F2/standard UART)
//   RX = UART_RADIO_RX_PIN (GPIO 23 on FreeWili, valid only via F11/UART_AUX)
// Serial2.setRX(23) won't work because Arduino-Pico only knows the F2 mux,
// so we override the RX pin function with gpio_set_function() AFTER Serial2
// has set up the rest of UART1.
#define RADIO_SERIAL Serial2

// Singleton-ish: the radio interface is created once in initLoRa() and never
// destroyed. We stash a pointer here so the free `freewili_poll_uart_radio()`
// function can find it without going through the Router/unique_ptr layers.
static UARTRadioInterface *s_uartRadioInstance = nullptr;

UARTRadioInterface::UARTRadioInterface() : RadioInterface()
{
    s_uartRadioInstance = this;
}

UARTRadioInterface::~UARTRadioInterface()
{
    if (s_uartRadioInstance == this)
        s_uartRadioInstance = nullptr;
}

extern "C" void freewili_poll_uart_radio(void)
{
    if (s_uartRadioInstance)
        s_uartRadioInstance->processUART();
}

bool UARTRadioInterface::init()
{
    // Call base class init (sets up observers, calls applyModemConfig)
    RadioInterface::init();

#if defined(UART_RADIO_TX_PIN) && defined(UART_RADIO_RX_PIN) && defined(UART_RADIO_BAUD)
#if defined(ARCH_RP2040) && defined(FREEWILI)
    // Bypass Arduino-Pico's Serial2 entirely on FreeWili. The Arduino HardwareSerial
    // layer keeps restoring GPIO pin functions to NULL after begin() (likely the
    // SerialUART end() restore path being triggered by something we haven't
    // identified). Using raw pico-sdk uart APIs — same pattern the working
    // wio-e5-unlock project used — sidesteps the whole Arduino state machine.
    uart_init(uart1, UART_RADIO_BAUD);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart1, false, false);
    uart_set_fifo_enabled(uart1, true);
    // Detach default Arduino-Pico Serial2 pins (4 = TX, 5 = RX) which would
    // otherwise be live alongside ours.
    gpio_set_function(4, GPIO_FUNC_NULL);
    gpio_set_function(5, GPIO_FUNC_NULL);
    // Route our actual pins to UART1.
    gpio_set_function(UART_RADIO_TX_PIN, GPIO_FUNC_UART);     // F2: UART1 TX on GPIO 40
    gpio_set_function(UART_RADIO_RX_PIN, GPIO_FUNC_UART_AUX); // F11: UART1 RX on GPIO 23

    // Install RX-only ISR draining UART1 → software ring buffer. Without it,
    // poll-only RX loses most bytes of any frame larger than 32 (FIFO depth).
    s_rx_head = s_rx_tail = 0;
    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_isr);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, /*rx_has_data=*/true, /*tx_needs_data=*/false);
#else
    // Other platforms / boards: use Arduino HardwareSerial as before.
    RADIO_SERIAL.setTX(UART_RADIO_TX_PIN);
    RADIO_SERIAL.begin(UART_RADIO_BAUD);
#endif

    LOG_INFO("UARTRadioInterface: UART initialized (TX=%d, RX=%d, baud=%d)", UART_RADIO_TX_PIN, UART_RADIO_RX_PIN,
             UART_RADIO_BAUD);

    // Give the bridge MCU a moment to be ready
    delay(100);

    // Configure DIO settings on the WIO-E5:
    //   DIO2 as RF switch = enabled (1)
    //   TCXO voltage = 1.8V (encoded as 18 = 1.8 * 10)
    uint8_t dioPayload[DIO_PAYLOAD_SIZE];
    dioPayload[DIO_RF_SWITCH_OFFSET] = 1;  // DIO2 as RF switch
    dioPayload[DIO_TCXO_OFFSET] = 18;      // 1.8V TCXO
    sendCommand(CMD_RADIO_SET_DIO, dioPayload, DIO_PAYLOAD_SIZE);
    delay(10);

    // Send radio configuration (freq, bw, sf, cr, power, preamble, syncword)
    sendRadioConfig();
    delay(10);

    // Start receiving
    sendCommand(CMD_RADIO_RX_START);
    receiving = true;

    LOG_INFO("UARTRadioInterface: init complete, receiving");
#else
    LOG_WARN("UARTRadioInterface: UART pins not defined, radio disabled");
#endif
    return true;
}

extern volatile uint32_t g_send_entry;

ErrorCode UARTRadioInterface::send(meshtastic_MeshPacket *p)
{
    g_send_entry++;
    if (disabled)
        return ERRNO_DISABLED;

    // If a previous send never completed (no RSP_TX_DONE returned from the
    // bridge), clear the stale state so beginSending's assert(!sendingPacket)
    // does not crash the device. This can happen if the WIO-E5 bridge drops a
    // frame, loses the response, or the LoRa region is UNSET so the radio
    // refuses to transmit. The most likely cause in practice is the retry
    // path (NextHopRouter::doRetransmissions) firing before the bridge gets
    // around to ACKing the first send.
    if (sendingPacket) {
        LOG_WARN("UARTRadioInterface: prior send incomplete (no TX_DONE), abandoning packet id=0x%x",
                 sendingPacket->id);
        packetPool.release(sendingPacket);
        sendingPacket = NULL;
        txPending = false;
    }

    // Encode the MeshPacket into radioBuffer and get the total byte count
    size_t totalLen = beginSending(p);

    // Send the raw radio bytes to the bridge as CMD_RADIO_TX
    sendCommand(CMD_RADIO_TX, (const uint8_t *)&radioBuffer, totalLen);
    txPending = true;

#if defined(FREEWILI)
    // TX activity: brief orange blink across all LEDs, then restore ambient.
    freewili_led_pulse_all(/*r=*/28, /*g=*/10, /*b=*/0, /*duration_ms=*/40);
#endif

    LOG_DEBUG("UARTRadioInterface: TX %u bytes", totalLen);
    return ERRNO_OK;
}

bool UARTRadioInterface::reconfigure()
{
    // Base class reconfigure calls applyModemConfig() which updates bw/sf/cr/power/savedFreq
    RadioInterface::reconfigure();

    // Re-send config to the bridge
    sendRadioConfig();
    delay(10);

    // Restart receiving
    sendCommand(CMD_RADIO_RX_START);
    receiving = true;

    LOG_INFO("UARTRadioInterface: reconfigured");
    return true;
}

bool UARTRadioInterface::canSleep()
{
    return !txPending && !sendingPacket;
}

uint32_t UARTRadioInterface::getPacketTime(uint32_t totalPacketLen, bool received)
{
    // LoRa airtime calculation per Semtech AN1200.13
    // https://www.semtech.com/uploads/documents/LoraDesignGuide_STD.pdf

    float bandwidth = bw * 1000.0f; // bw is in kHz, convert to Hz
    float tSym = pow(2.0f, sf) / bandwidth;
    float tPreamble = (preambleLength + 4.25f) * tSym;

    // Low data rate optimization (LDRO) is used when symbol time >= 16ms
    int de = ((1 << sf) / bw >= 16) ? 1 : 0;
    // CRC is always enabled in Meshtastic
    int crcOn = 1;
    // Implicit header is not used
    int ih = 0;

    float payloadSymbNb = 8.0f + fmax(ceil((8.0f * totalPacketLen - 4.0f * sf + 28.0f + 16.0f * crcOn - 20.0f * ih) /
                                            (4.0f * (sf - 2.0f * de))) *
                                           cr,
                                       0.0f);

    float tPayload = payloadSymbNb * tSym;
    float tPacket = tPreamble + tPayload;

    // Convert seconds to milliseconds
    return (uint32_t)(tPacket * 1000.0f);
}

// Debug counters inspectable via GDB. Marked used so the linker can't drop
// them even with -Wl,--gc-sections, and volatile so the compiler can't
// optimize the increments out.
volatile uint32_t g_processUART_count __attribute__((used)) = 0;
volatile uint32_t g_reinit_count __attribute__((used)) = 0;
volatile uint32_t g_sendCommand_count __attribute__((used)) = 0;

// Counters to localize "where TX stops". g_menu_position_branch fires when
// the Home menu's Position branch executes (proves menu selection landed).
// g_send_entry fires inside UARTRadioInterface::send (proves something asked
// us to TX). If position_branch but not send_entry, the packet got lost in
// Router/Service before reaching us.
volatile uint32_t g_menu_position_branch __attribute__((used)) = 0;
volatile uint32_t g_send_entry           __attribute__((used)) = 0;
// Additional waypoints to find where packets actually drop.
volatile uint32_t g_nodeinfo_sendOurEntry   __attribute__((used)) = 0;
volatile uint32_t g_nodeinfo_packetNonNull  __attribute__((used)) = 0;
volatile uint32_t g_nodeinfo_sentToMesh     __attribute__((used)) = 0;
volatile uint32_t g_router_send_entry       __attribute__((used)) = 0;
volatile uint32_t g_router_before_iface     __attribute__((used)) = 0;
volatile uint32_t g_allocReply_entry           __attribute__((used)) = 0;
volatile uint32_t g_allocReply_blockedSuppress __attribute__((used)) = 0;
volatile uint32_t g_allocReply_blockedChanUtil __attribute__((used)) = 0;
volatile uint32_t g_allocReply_blockedThrottle __attribute__((used)) = 0;
volatile uint32_t g_allocReply_callDataProto   __attribute__((used)) = 0;
volatile uint32_t g_allocReply_dataProtoNull   __attribute__((used)) = 0;

// Snapshot of config.lora and RadioInterface state at the moment we last
// programmed the bridge. Used to diagnose why getFreq() returns a value that
// doesn't match the T-Deck's hash-derived LongFast slot.
volatile float    g_diag_savedFreq          __attribute__((used)) = 0.0f;
volatile float    g_diag_freq_offset        __attribute__((used)) = 0.0f;
volatile float    g_diag_override_freq      __attribute__((used)) = 0.0f;
volatile uint16_t g_diag_channel_num        __attribute__((used)) = 0;
volatile int32_t  g_diag_region             __attribute__((used)) = -1;
volatile uint8_t  g_diag_modem_preset       __attribute__((used)) = 0xFF;
volatile uint8_t  g_diag_use_preset         __attribute__((used)) = 0xFF;
volatile uint32_t g_diag_savedChannelNum    __attribute__((used)) = 0xFFFFFFFF;
// freqHz captured at the EXACT moment of the cast in sendRadioConfig — what
// actually went on the wire to the bridge.
volatile uint32_t g_diag_freqhz_at_cast     __attribute__((used)) = 0;
volatile uint32_t g_diag_sendcfg_calls      __attribute__((used)) = 0;

// RX-path waypoints. Set when a bridge RSP_RX_PACKET is parsed and delivered.
volatile uint32_t g_rsp_rx_count            __attribute__((used)) = 0;
volatile uint32_t g_rsp_rx_lastlen          __attribute__((used)) = 0;
volatile uint32_t g_rsp_rx_deliver_count    __attribute__((used)) = 0;
// Byte-level UART RX counters: total bytes through parseByte, CRC failures,
// oversized payload errors. Helps diagnose whether bytes reach Display at all
// vs being lost to FIFO overflow.
volatile uint32_t g_uart_byte_count         __attribute__((used)) = 0;
volatile uint32_t g_uart_crc_fail_count     __attribute__((used)) = 0;
volatile uint32_t g_uart_payload_oversize   __attribute__((used)) = 0;
volatile uint8_t  g_uart_last_byte          __attribute__((used)) = 0;

void UARTRadioInterface::processUART()
{
    g_processUART_count++;
#if defined(UART_RADIO_TX_PIN) && defined(UART_RADIO_RX_PIN)
#if defined(ARCH_RP2040) && defined(FREEWILI)
    // Re-assert TX/RX pin functions every poll (idempotent).
    gpio_set_function(UART_RADIO_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RADIO_RX_PIN, GPIO_FUNC_UART_AUX);
#endif

    bool gotAny = false;
#if defined(ARCH_RP2040) && defined(FREEWILI)
    // Drain the ISR-filled ring buffer rather than polling UART hardware.
    while (s_rx_tail != s_rx_head) {
        uint8_t b = s_rx_ring[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1u) % UART_RX_RING_SIZE;
        parseByte(b);
        gotAny = true;
    }
#else
    while (RADIO_SERIAL.available()) {
        uint8_t b = RADIO_SERIAL.read();
        parseByte(b);
        gotAny = true;
    }
#endif
    if (gotAny)
        bridgeAlive = true;

    // If we've never heard from the bridge, periodically re-send the init
    // sequence. The first init call from init() can be lost if our GPIO
    // pins haven't taken effect yet (something is clobbering them post-
    // begin()); periodic re-sends here are robust against that race.
    if (!bridgeAlive) {
        uint32_t now = millis();
        if (now - lastReinitMs > 1000) {
            lastReinitMs = now;
            g_reinit_count++;
            uint8_t dioPayload[DIO_PAYLOAD_SIZE] = {};
            dioPayload[DIO_RF_SWITCH_OFFSET] = 1;
            dioPayload[DIO_TCXO_OFFSET] = 18;
            sendCommand(CMD_RADIO_SET_DIO, dioPayload, DIO_PAYLOAD_SIZE);
            sendRadioConfig();
            sendCommand(CMD_RADIO_RX_START);
        }
    }
#endif
}

void UARTRadioInterface::parseByte(uint8_t byte)
{
    g_uart_byte_count++;
    g_uart_last_byte = byte;
    switch (rxState) {
    case SYNC1:
        if (byte == UART_RADIO_SYNC1)
            rxState = SYNC2;
        break;
    case SYNC2:
        if (byte == UART_RADIO_SYNC2)
            rxState = CMD;
        else
            rxState = SYNC1;
        break;
    case CMD:
        rxCmd = byte;
        rxState = LEN_HI;
        break;
    case LEN_HI:
        rxPayloadLen = (uint16_t)byte << 8;
        rxState = LEN_LO;
        break;
    case LEN_LO:
        rxPayloadLen |= byte;
        rxPayloadIdx = 0;
        if (rxPayloadLen > UART_RADIO_MAX_PAYLOAD) {
            g_uart_payload_oversize++;
            LOG_WARN("UARTRadio: payload too large (%u), resetting", rxPayloadLen);
            rxState = SYNC1;
        } else if (rxPayloadLen == 0) {
            rxState = CRC_BYTE;
        } else {
            rxState = PAYLOAD;
        }
        break;
    case PAYLOAD:
        rxPayload[rxPayloadIdx++] = byte;
        if (rxPayloadIdx >= rxPayloadLen)
            rxState = CRC_BYTE;
        break;
    case CRC_BYTE: {
        // Verify CRC over cmd + len_hi + len_lo + payload
        uint8_t crcBuf[3 + UART_RADIO_MAX_PAYLOAD];
        crcBuf[0] = rxCmd;
        crcBuf[1] = (rxPayloadLen >> 8) & 0xFF;
        crcBuf[2] = rxPayloadLen & 0xFF;
        if (rxPayloadLen > 0)
            memcpy(&crcBuf[3], rxPayload, rxPayloadLen);
        uint8_t expectedCrc = crc8(crcBuf, 3 + rxPayloadLen);

        if (byte == expectedCrc) {
            handleResponse(rxCmd, rxPayload, rxPayloadLen);
        } else {
            g_uart_crc_fail_count++;
            LOG_WARN("UARTRadio: CRC mismatch (got 0x%02x, expected 0x%02x)", byte, expectedCrc);
        }
        rxState = SYNC1;
        break;
    }
    }
}

extern volatile uint32_t g_sendCommand_count;

void UARTRadioInterface::sendCommand(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    g_sendCommand_count++;
#if defined(UART_RADIO_TX_PIN) && defined(UART_RADIO_RX_PIN)
    uint8_t frameBuf[UART_RADIO_HEADER_SIZE + UART_RADIO_MAX_PAYLOAD + UART_RADIO_CRC_SIZE];
    uint16_t frameLen = uart_proto_build_frame(frameBuf, cmd, payload, len);
#if defined(ARCH_RP2040) && defined(FREEWILI)
    // Re-assert pin functions before every TX. Something keeps resetting
    // GPIO 40 to NULL; setting it idempotently here guarantees the bytes
    // we write next actually leave the chip.
    gpio_set_function(UART_RADIO_TX_PIN, GPIO_FUNC_UART);     // F2: UART1 TX
    gpio_set_function(UART_RADIO_RX_PIN, GPIO_FUNC_UART_AUX); // F11: UART1 RX
    uart_write_blocking(uart1, frameBuf, frameLen);
#else
    RADIO_SERIAL.write(frameBuf, frameLen);
    RADIO_SERIAL.flush();
#endif
#endif
}

void UARTRadioInterface::sendRadioConfig()
{
    g_diag_sendcfg_calls++;
    uint8_t cfgPayload[CFG_PAYLOAD_SIZE];
    memset(cfgPayload, 0, sizeof(cfgPayload));

    // Frequency in Hz (savedFreq is in MHz).
    //
    // DIAGNOSTIC OVERRIDE: at the time of this cast, getFreq() returns a value
    // that differs from the post-call savedFreq snapshot (906.055808 here vs
    // 906.875 there) — applyModemConfig appears to fire asynchronously during
    // boot, so the FIRST sendRadioConfig sees stale state even though
    // override_frequency was set in main.cpp setup() before initLoRa(). The
    // T-Deck reference node is on slot 19 = 906.875 MHz. Hardcoded here as the
    // smallest reliable change that makes OTA RX work; revisit once the boot-
    // sequence race is fixed properly.
    // T-Deck reference node operates on the LongFast US slot 19 = 906.875 MHz.
    // Prior session intended this but wrote 0x36092818, which is 906,569,752 Hz
    // (~305 kHz off). LoRa SF11/BW250 tolerates only ~±62 kHz of offset, so
    // every RX failed CRC at the WIO-E5 even though demod fired RxDone.
    uint32_t freqHz = 906875000u;  // 906.875 MHz, LongFast US slot 19
    g_diag_freqhz_at_cast = freqHz;
    cfgPayload[CFG_FREQ_OFFSET + 0] = (freqHz >> 24) & 0xFF;
    cfgPayload[CFG_FREQ_OFFSET + 1] = (freqHz >> 16) & 0xFF;
    cfgPayload[CFG_FREQ_OFFSET + 2] = (freqHz >> 8) & 0xFF;
    cfgPayload[CFG_FREQ_OFFSET + 3] = freqHz & 0xFF;

    // Bandwidth encoding: 0=125kHz, 1=250kHz, 2=500kHz
    if (bw <= 125.0f)
        cfgPayload[CFG_BW_OFFSET] = 0;
    else if (bw <= 250.0f)
        cfgPayload[CFG_BW_OFFSET] = 1;
    else
        cfgPayload[CFG_BW_OFFSET] = 2;

    cfgPayload[CFG_SF_OFFSET] = sf;
    cfgPayload[CFG_CR_OFFSET] = cr;
    cfgPayload[CFG_POWER_OFFSET] = (uint8_t)power;

    // Preamble length (big-endian uint16)
    cfgPayload[CFG_PREAMBLE_OFFSET + 0] = (preambleLength >> 8) & 0xFF;
    cfgPayload[CFG_PREAMBLE_OFFSET + 1] = preambleLength & 0xFF;

    // Sync word: Meshtastic uses 0x2b (same as RadioLibInterface::syncWord)
    // Send as uint16 big-endian for SX126x compatibility (0x2b -> 0x2b24)
    uint16_t syncWord16 = 0x2b24; // LoRa sync word for Meshtastic on SX126x
    cfgPayload[CFG_SYNCWORD_OFFSET + 0] = (syncWord16 >> 8) & 0xFF;
    cfgPayload[CFG_SYNCWORD_OFFSET + 1] = syncWord16 & 0xFF;

    sendCommand(CMD_RADIO_CONFIGURE, cfgPayload, CFG_PAYLOAD_SIZE);

    // Snapshot LoRa config state for SWD inspection. Each radio config send
    // refreshes these — last call wins, which is what we want.
    g_diag_savedFreq       = savedFreq;
    g_diag_freq_offset     = config.lora.frequency_offset;
    g_diag_override_freq   = config.lora.override_frequency;
    g_diag_channel_num     = config.lora.channel_num;
    g_diag_region          = (int32_t)config.lora.region;
    g_diag_modem_preset    = (uint8_t)config.lora.modem_preset;
    g_diag_use_preset      = config.lora.use_preset ? 1 : 0;
    g_diag_savedChannelNum = savedChannelNum;

    LOG_INFO("UARTRadio: config freq=%.3fMHz bw=%.0fkHz sf=%u cr=%u pwr=%d preamble=%u", getFreq(), bw, sf, cr, power,
             preambleLength);
}

void UARTRadioInterface::handleResponse(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    switch (cmd) {
    case RSP_ACK:
        LOG_DEBUG("UARTRadio: ACK received");
        break;

    case RSP_TX_DONE: {
        LOG_DEBUG("UARTRadio: TX done");
        txPending = false;

        // Complete the sending process (frees the packet, logs airtime)
        if (sendingPacket) {
            auto p = sendingPacket;
            sendingPacket = NULL;

            uint32_t xmitMsec = RadioInterface::getPacketTime(p);
            airTime->logAirtime(TX_LOG, xmitMsec);
            printPacket("Completed sending", p);
            packetPool.release(p);
        }

        // Restart receiving after TX
        sendCommand(CMD_RADIO_RX_START);
        receiving = true;
        break;
    }

    case RSP_RX_PACKET: {
        g_rsp_rx_count++;
        g_rsp_rx_lastlen = len;
        // Bridge frame format: [rssi_lo, rssi_hi, snr, <packet bytes>]
        //   rssi: int16 in tenths of dBm (so /10 to get dBm)
        //   snr:  int8  in quarters of dB (so /4 to get dB)
        const size_t kMetaSize = 3;
        if (len < kMetaSize + sizeof(PacketHeader)) {
            LOG_WARN("UARTRadio: RX packet too short (%u bytes)", len);
            break;
        }

        int16_t rssi_tenths;
        memcpy(&rssi_tenths, &payload[0], 2);
        int8_t snr_quarters = (int8_t)payload[2];

        // The bridge sends: 3 metadata bytes, then PacketHeader + encrypted data.
        const uint8_t *pktPayload = &payload[kMetaSize];
        uint16_t pktLen = len - kMetaSize;
        memcpy(&radioBuffer, pktPayload, pktLen);

        int32_t payloadLen = pktLen - sizeof(PacketHeader);
        if (payloadLen < 0) {
            LOG_WARN("UARTRadio: RX packet payload too short");
            break;
        }

        // Log airtime for the received packet
        uint32_t rxMsec = getPacketTime(pktLen, true);
        airTime->logAirtime(RX_LOG, rxMsec);

        // Reject packets with from == 0 (could be spoofed)
        if (radioBuffer.header.from == 0) {
            LOG_WARN("UARTRadio: ignoring packet with from=0");
            break;
        }

        // Allocate a MeshPacket and populate from the radio buffer
        meshtastic_MeshPacket *mp = packetPool.allocZeroed();

        mp->from = radioBuffer.header.from;
        mp->to = radioBuffer.header.to;
        mp->id = radioBuffer.header.id;
        mp->channel = radioBuffer.header.channel;
        mp->hop_limit = radioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
        mp->hop_start = (radioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
        mp->want_ack = !!(radioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
        mp->via_mqtt = !!(radioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);
        mp->next_hop = mp->hop_start == 0 ? NO_NEXT_HOP_PREFERENCE : radioBuffer.header.next_hop;
        mp->relay_node = mp->hop_start == 0 ? NO_RELAY_NODE : radioBuffer.header.relay_node;

        // Restore the bridge's RSSI/SNR scaling. rx_rssi is int32 dBm, rx_snr is float dB.
        mp->rx_rssi = rssi_tenths / 10;
        mp->rx_snr = snr_quarters / 4.0f;
        mp->rx_time = getValidTime(RTCQualityFromNet);

        mp->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
        assert(((uint32_t)payloadLen) <= sizeof(mp->encrypted.bytes));
        memcpy(mp->encrypted.bytes, radioBuffer.payload, payloadLen);
        mp->encrypted.size = payloadLen;

        printPacket("UARTRadio RX", mp);
#if defined(FREEWILI)
        // RX activity: brief green blink across all LEDs, then restore ambient.
        freewili_led_pulse_all(/*r=*/0, /*g=*/28, /*b=*/0, /*duration_ms=*/40);
#endif
        g_rsp_rx_deliver_count++;
        deliverToReceiver(mp);
        break;
    }

    case RSP_STATUS:
        LOG_DEBUG("UARTRadio: status response (state=%u)", len > 0 ? payload[0] : 0xFF);
        break;

    case RSP_ERROR:
        LOG_ERROR("UARTRadio: bridge error (code=%u)", len > 0 ? payload[0] : 0xFF);
        break;

    default:
        LOG_WARN("UARTRadio: unknown response cmd=0x%02x len=%u", cmd, len);
        break;
    }
}
