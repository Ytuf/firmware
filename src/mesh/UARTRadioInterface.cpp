#include "UARTRadioInterface.h"
#include "MeshTypes.h"
#include "NodeDB.h"
#include "Router.h"
#include "airtime.h"
#include "configuration.h"
#include "gps/RTC.h"
#include "main.h"
#include "meshUtils.h"
#include <Arduino.h>
#include <math.h>

#if defined(ARCH_RP2040)
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#endif

#if defined(ARCH_RP2040) && defined(FREEWILI)
// UART1 RX ring buffer drained by ISR; 32-byte HW FIFO overflows on 115200-baud bursts.
#define UART_RX_RING_SIZE 1024u
static volatile uint8_t  s_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
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
            g_uart_ring_drops++;
        }
    }
}
#endif

#if defined(FREEWILI)
extern "C" void freewili_led_pulse_all(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);
#endif

#define RADIO_SERIAL Serial2

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
    RadioInterface::init();

#if defined(UART_RADIO_TX_PIN) && defined(UART_RADIO_RX_PIN) && defined(UART_RADIO_BAUD)
#if defined(ARCH_RP2040) && defined(FREEWILI)
    // Bypass Arduino-Pico Serial2; HardwareSerial clobbers GPIO functions post-begin().
    uart_init(uart1, UART_RADIO_BAUD);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart1, false, false);
    uart_set_fifo_enabled(uart1, true);
    gpio_set_function(4, GPIO_FUNC_NULL);
    gpio_set_function(5, GPIO_FUNC_NULL);
    gpio_set_function(UART_RADIO_TX_PIN, GPIO_FUNC_UART);     // F2: UART1 TX on GPIO 40
    gpio_set_function(UART_RADIO_RX_PIN, GPIO_FUNC_UART_AUX); // F11: UART1 RX on GPIO 23

    s_rx_head = s_rx_tail = 0;
    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_isr);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);
#else
    RADIO_SERIAL.setTX(UART_RADIO_TX_PIN);
    RADIO_SERIAL.begin(UART_RADIO_BAUD);
#endif

    LOG_INFO("UARTRadioInterface: UART initialized (TX=%d, RX=%d, baud=%d)", UART_RADIO_TX_PIN, UART_RADIO_RX_PIN,
             UART_RADIO_BAUD);

    delay(100);

    uint8_t dioPayload[DIO_PAYLOAD_SIZE];
    dioPayload[DIO_RF_SWITCH_OFFSET] = 1;  // DIO2 as RF switch
    dioPayload[DIO_TCXO_OFFSET] = 18;      // 1.8V TCXO
    sendCommand(CMD_RADIO_SET_DIO, dioPayload, DIO_PAYLOAD_SIZE);
    delay(10);

    sendRadioConfig();
    delay(10);

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

    // Clear stale state if prior send never got RSP_TX_DONE (beginSending asserts !sendingPacket).
    if (sendingPacket) {
        LOG_WARN("UARTRadioInterface: prior send incomplete (no TX_DONE), abandoning packet id=0x%x",
                 sendingPacket->id);
        packetPool.release(sendingPacket);
        sendingPacket = NULL;
        txPending = false;
    }

    size_t totalLen = beginSending(p);

    sendCommand(CMD_RADIO_TX, (const uint8_t *)&radioBuffer, totalLen);
    txPending = true;

#if defined(FREEWILI)
    freewili_led_pulse_all(28, 10, 0, 40);
#endif

    LOG_DEBUG("UARTRadioInterface: TX %u bytes", totalLen);
    return ERRNO_OK;
}

bool UARTRadioInterface::reconfigure()
{
    RadioInterface::reconfigure();

    sendRadioConfig();
    delay(10);

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
    // LoRa airtime per Semtech AN1200.13.
    float bandwidth = bw * 1000.0f;
    float tSym = pow(2.0f, sf) / bandwidth;
    float tPreamble = (preambleLength + 4.25f) * tSym;

    int de = ((1 << sf) / bw >= 16) ? 1 : 0;
    int crcOn = 1;
    int ih = 0;

    float payloadSymbNb = 8.0f + fmax(ceil((8.0f * totalPacketLen - 4.0f * sf + 28.0f + 16.0f * crcOn - 20.0f * ih) /
                                            (4.0f * (sf - 2.0f * de))) *
                                           cr,
                                       0.0f);

    float tPayload = payloadSymbNb * tSym;
    float tPacket = tPreamble + tPayload;

    return (uint32_t)(tPacket * 1000.0f);
}

volatile uint32_t g_processUART_count __attribute__((used)) = 0;
volatile uint32_t g_reinit_count __attribute__((used)) = 0;
volatile uint32_t g_sendCommand_count __attribute__((used)) = 0;

volatile uint32_t g_menu_position_branch __attribute__((used)) = 0;
volatile uint32_t g_send_entry           __attribute__((used)) = 0;
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

volatile float    g_diag_savedFreq          __attribute__((used)) = 0.0f;
volatile float    g_diag_freq_offset        __attribute__((used)) = 0.0f;
volatile float    g_diag_override_freq      __attribute__((used)) = 0.0f;
volatile uint16_t g_diag_channel_num        __attribute__((used)) = 0;
volatile int32_t  g_diag_region             __attribute__((used)) = -1;
volatile uint8_t  g_diag_modem_preset       __attribute__((used)) = 0xFF;
volatile uint8_t  g_diag_use_preset         __attribute__((used)) = 0xFF;
volatile uint32_t g_diag_savedChannelNum    __attribute__((used)) = 0xFFFFFFFF;
volatile uint32_t g_diag_freqhz_at_cast     __attribute__((used)) = 0;
volatile uint32_t g_diag_sendcfg_calls      __attribute__((used)) = 0;

volatile uint32_t g_rsp_rx_count            __attribute__((used)) = 0;
volatile uint32_t g_rsp_rx_lastlen          __attribute__((used)) = 0;
volatile uint32_t g_rsp_rx_deliver_count    __attribute__((used)) = 0;
volatile uint32_t g_uart_byte_count         __attribute__((used)) = 0;
volatile uint32_t g_uart_crc_fail_count     __attribute__((used)) = 0;
volatile uint32_t g_uart_payload_oversize   __attribute__((used)) = 0;
volatile uint8_t  g_uart_last_byte          __attribute__((used)) = 0;

void UARTRadioInterface::processUART()
{
    g_processUART_count++;
#if defined(UART_RADIO_TX_PIN) && defined(UART_RADIO_RX_PIN)
#if defined(ARCH_RP2040) && defined(FREEWILI)
    gpio_set_function(UART_RADIO_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RADIO_RX_PIN, GPIO_FUNC_UART_AUX);
#endif

    bool gotAny = false;
#if defined(ARCH_RP2040) && defined(FREEWILI)
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

    // Periodically re-send init if bridge never responded — GPIO pins may not yet be active.
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
    // Re-assert pin functions before TX; something resets GPIO 40 to NULL.
    gpio_set_function(UART_RADIO_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RADIO_RX_PIN, GPIO_FUNC_UART_AUX);
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

    // Hardcoded — applyModemConfig races with initLoRa and getFreq() returns stale value.
    uint32_t freqHz = 906875000u;  // 906.875 MHz, LongFast US slot 19
    g_diag_freqhz_at_cast = freqHz;
    cfgPayload[CFG_FREQ_OFFSET + 0] = (freqHz >> 24) & 0xFF;
    cfgPayload[CFG_FREQ_OFFSET + 1] = (freqHz >> 16) & 0xFF;
    cfgPayload[CFG_FREQ_OFFSET + 2] = (freqHz >> 8) & 0xFF;
    cfgPayload[CFG_FREQ_OFFSET + 3] = freqHz & 0xFF;

    // Bandwidth: 0=125kHz, 1=250kHz, 2=500kHz
    if (bw <= 125.0f)
        cfgPayload[CFG_BW_OFFSET] = 0;
    else if (bw <= 250.0f)
        cfgPayload[CFG_BW_OFFSET] = 1;
    else
        cfgPayload[CFG_BW_OFFSET] = 2;

    cfgPayload[CFG_SF_OFFSET] = sf;
    cfgPayload[CFG_CR_OFFSET] = cr;
    cfgPayload[CFG_POWER_OFFSET] = (uint8_t)power;

    cfgPayload[CFG_PREAMBLE_OFFSET + 0] = (preambleLength >> 8) & 0xFF;
    cfgPayload[CFG_PREAMBLE_OFFSET + 1] = preambleLength & 0xFF;

    uint16_t syncWord16 = 0x2b24; // LoRa sync word for Meshtastic on SX126x
    cfgPayload[CFG_SYNCWORD_OFFSET + 0] = (syncWord16 >> 8) & 0xFF;
    cfgPayload[CFG_SYNCWORD_OFFSET + 1] = syncWord16 & 0xFF;

    sendCommand(CMD_RADIO_CONFIGURE, cfgPayload, CFG_PAYLOAD_SIZE);

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

        if (sendingPacket) {
            auto p = sendingPacket;
            sendingPacket = NULL;

            uint32_t xmitMsec = RadioInterface::getPacketTime(p);
            airTime->logAirtime(TX_LOG, xmitMsec);
            printPacket("Completed sending", p);
            packetPool.release(p);
        }

        sendCommand(CMD_RADIO_RX_START);
        receiving = true;
        break;
    }

    case RSP_RX_PACKET: {
        g_rsp_rx_count++;
        g_rsp_rx_lastlen = len;
        // Bridge frame: [rssi_lo, rssi_hi (int16 dBm*10), snr (int8 dB*4), <packet>]
        const size_t kMetaSize = 3;
        if (len < kMetaSize + sizeof(PacketHeader)) {
            LOG_WARN("UARTRadio: RX packet too short (%u bytes)", len);
            break;
        }

        int16_t rssi_tenths;
        memcpy(&rssi_tenths, &payload[0], 2);
        int8_t snr_quarters = (int8_t)payload[2];

        const uint8_t *pktPayload = &payload[kMetaSize];
        uint16_t pktLen = len - kMetaSize;
        memcpy(&radioBuffer, pktPayload, pktLen);

        int32_t payloadLen = pktLen - sizeof(PacketHeader);
        if (payloadLen < 0) {
            LOG_WARN("UARTRadio: RX packet payload too short");
            break;
        }

        uint32_t rxMsec = getPacketTime(pktLen, true);
        airTime->logAirtime(RX_LOG, rxMsec);

        if (radioBuffer.header.from == 0) {
            LOG_WARN("UARTRadio: ignoring packet with from=0");
            break;
        }

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

        mp->rx_rssi = rssi_tenths / 10;
        mp->rx_snr = snr_quarters / 4.0f;
        mp->rx_time = getValidTime(RTCQualityFromNet);

        mp->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
        assert(((uint32_t)payloadLen) <= sizeof(mp->encrypted.bytes));
        memcpy(mp->encrypted.bytes, radioBuffer.payload, payloadLen);
        mp->encrypted.size = payloadLen;

        printPacket("UARTRadio RX", mp);
#if defined(FREEWILI)
        freewili_led_pulse_all(0, 28, 0, 40);
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
