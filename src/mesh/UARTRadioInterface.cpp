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

// We use Serial2 for the UART radio bridge.
// Pin configuration comes from the variant header (UART_RADIO_TX_PIN, UART_RADIO_RX_PIN, UART_RADIO_BAUD).

UARTRadioInterface::UARTRadioInterface() : RadioInterface() {}

UARTRadioInterface::~UARTRadioInterface() {}

bool UARTRadioInterface::init()
{
    // Call base class init (sets up observers, calls applyModemConfig)
    RadioInterface::init();

    // Initialize UART to the WIO-E5 bridge
    Serial2.setTX(UART_RADIO_TX_PIN);
    Serial2.setRX(UART_RADIO_RX_PIN);
    Serial2.begin(UART_RADIO_BAUD);

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
    return true;
}

ErrorCode UARTRadioInterface::send(meshtastic_MeshPacket *p)
{
    if (disabled)
        return ERRNO_DISABLED;

    // Encode the MeshPacket into radioBuffer and get the total byte count
    size_t totalLen = beginSending(p);

    // Send the raw radio bytes to the bridge as CMD_RADIO_TX
    sendCommand(CMD_RADIO_TX, (const uint8_t *)&radioBuffer, totalLen);
    txPending = true;

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

void UARTRadioInterface::processUART()
{
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        parseByte(b);
    }
}

void UARTRadioInterface::parseByte(uint8_t byte)
{
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
            LOG_WARN("UARTRadio: CRC mismatch (got 0x%02x, expected 0x%02x)", byte, expectedCrc);
        }
        rxState = SYNC1;
        break;
    }
    }
}

void UARTRadioInterface::sendCommand(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t frameBuf[UART_RADIO_HEADER_SIZE + UART_RADIO_MAX_PAYLOAD + UART_RADIO_CRC_SIZE];
    uint16_t frameLen = uart_proto_build_frame(frameBuf, cmd, payload, len);
    Serial2.write(frameBuf, frameLen);
    Serial2.flush();
}

void UARTRadioInterface::sendRadioConfig()
{
    uint8_t cfgPayload[CFG_PAYLOAD_SIZE];
    memset(cfgPayload, 0, sizeof(cfgPayload));

    // Frequency in Hz (savedFreq is in MHz)
    uint32_t freqHz = (uint32_t)(getFreq() * 1000000.0f);
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
        if (len < sizeof(PacketHeader)) {
            LOG_WARN("UARTRadio: RX packet too short (%u bytes)", len);
            break;
        }

        // The bridge sends raw LoRa payload which is: PacketHeader + encrypted data
        // Copy into radioBuffer for parsing
        memcpy(&radioBuffer, payload, len);

        int32_t payloadLen = len - sizeof(PacketHeader);
        if (payloadLen < 0) {
            LOG_WARN("UARTRadio: RX packet payload too short");
            break;
        }

        // Log airtime for the received packet
        uint32_t rxMsec = getPacketTime(len, true);
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

        // No SNR/RSSI metadata from UART bridge (could be extended later)
        mp->rx_snr = 0;
        mp->rx_rssi = 0;
        mp->rx_time = getValidTime(RTCQualityFromNet);

        mp->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
        assert(((uint32_t)payloadLen) <= sizeof(mp->encrypted.bytes));
        memcpy(mp->encrypted.bytes, radioBuffer.payload, payloadLen);
        mp->encrypted.size = payloadLen;

        printPacket("UARTRadio RX", mp);
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
