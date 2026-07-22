#include "onewili_fwgui.h"
#include <string.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "pico/time.h"
#include "sdfs_wire.h"
#include "sdfs_transport.h"

/* ── Link constants (FwGUI protocol; see the firmware's protocol.md) ───── */
#define OWFW_BAUD          8000000
#define OWFW_PIN_TX        1
#define OWFW_PIN_RX        0
/* Flow-control pins. The pair is wired by NET NAME, not pin number:
 *   net UART0_CTS = MAIN GPIO2 (its CTS input)  <-> DISPLAY GPIO3
 *   net UART0_RTS = MAIN GPIO3 (its RTS output) <-> DISPLAY GPIO2
 * PROVEN on hardware by cleanly isolating each pin over SWD (display halted):
 * driving DISPLAY GPIO3 low asserts MAIN's CTS (UARTFR 0x196->0x197 and the TX
 * FIFO drains); driving DISPLAY GPIO2 low does nothing. So GPIO3 is our RTS to
 * MAIN, GPIO2 is our CTS from MAIN.
 *
 * GPIO3 also happens to be the RP2350's fixed uart0 RTS pin, so we drive it with
 * HARDWARE RTS (uart_set_hw_flow rts=true): the controller holds it asserted
 * (low) whenever the RX FIFO has space, which the DMA drain keeps true, so MAIN
 * stays clear to send. This is what the known-good pre-SDFS image did.
 *
 * The hazard is Wire1: the arduino-pico freewili2 variant declares Wire1 on
 * GPIO2/3 (PIN_WIRE1_SDA/SCL); Wire1.begin() in main() re-muxes GPIO3 off UART
 * to I2C, killing hardware RTS -> MAIN's CTS deasserts -> link dies silently.
 * owfw_reclaim_rts() re-asserts the UART function on GPIO3 from every pump so a
 * later Wire1 use can't strand the link. GPIO2 is left as MAIN's CTS input.
 *
 * If MAIN's CTS is left deasserted, MAIN's uart0 TX is hardware-gated: it
 * silently queues frames it can never send -- looks exactly like "the link is
 * down" and eventually wedges MAIN in a blocking UART write. */
#define OWFW_PIN_CTS       2       /* our CTS <- MAIN's RTS (input) */
#define OWFW_PIN_RTS       3       /* our RTS -> MAIN's CTS (hardware-driven low) */
#define OWFW_EVT_SYNC0     0xB0    /* display->main event frames  */
#define OWFW_EVT_SYNC1     0x1D
#define OWFW_CMD_SYNC0     0xBE    /* main->display command frames */
#define OWFW_CMD_SYNC1     0xBA
#define OWFW_EVT_TERM      0x18    /* FWGUI_EVENT_M_TERM_INPUT (24) */
#define OWFW_MARKER        0x01    /* OneWili chunk marker          */
#define OWFW_CHUNK_MAX     56      /* text bytes per event frame    */
#define OWFW_CMD_RESPONSE  0x5D    /* FWGUI_API_ONEWILL_RESPONSE    */
#define OWFW_CMD_BINARY    0x5E    /* FWGUI_API_ONEWILL_BINARY      */
#define OWFW_CMD_SDFS      0x5F    /* FWGUI_API_SDFS_DATA           */
#define OWFW_EVT_SDFS      42      /* FWGUI_EVENT_SDFS_REQUEST      */
#define OWFW_STREAM_MAX    1024    /* per-stream buffer             */
#define OWFW_FRAME_MAX     512     /* incoming command frame payload cap */
#define OWFW_SDFS_SLOTS    8       /* queued inbound SDFS frames (mirrors Main's) */

/* ── Per-stream byte FIFO ──────────────────────────────────────────────── */
typedef struct {
    uint8_t  buf[OWFW_STREAM_MAX];
    uint32_t head, count;
} owfw_fifo;

static owfw_fifo g_text;     /* 0x5D: responses + "[*" text events */
static owfw_fifo g_binary;   /* 0x5E: binary WILI event bytes      */
static uint32_t  g_dropped;

/* 0x5F SDFS frames need their boundaries preserved (sdfs_transport_t.recv is
 * frame-oriented, not a byte stream), so this lane is a queue of whole frames
 * rather than an owfw_fifo. Mirrors Main's serial_comm_main.c ring. */
typedef struct { uint8_t buf[SDFS_MAX_FRAME]; uint16_t len; } owfw_sdfs_slot;
static owfw_sdfs_slot g_sdfs[OWFW_SDFS_SLOTS];
static uint16_t       g_sdfs_head, g_sdfs_tail;

/* DIAG (SWD-observable): raw transport health from MAIN. */
volatile uint32_t g_owfw_bytes = 0;      /* total bytes pumped from uart0     */
volatile uint32_t g_owfw_sync0 = 0;      /* 0xBE sync starts seen             */
volatile uint32_t g_owfw_frames_ok = 0;  /* frames that passed checksum       */
volatile uint32_t g_owfw_resp = 0;       /* 0x5D response frames accepted     */
volatile uint32_t g_owfw_sdfs_rx = 0;    /* 0x5F SDFS frames queued           */
volatile uint32_t g_owfw_sdfs_tx = 0;    /* event-42 SDFS frames sent         */
volatile uint32_t g_owfw_sdfs_drop = 0;  /* 0x5F frames dropped (queue full)  */
/* Counts how often GPIO2 had to be taken back from another peripheral. Any
 * non-zero value is Wire1 (or something else) re-muxing the link's RTS pin. */
volatile uint32_t g_owfw_rts_reclaims = 0;

/* DMA-drained RX ring: keeps the 32-byte HW RX FIFO empty so a large MAIN->Display
 * frame (an ~87-byte wifiscan event) is not overrun while this cooperative task is
 * between its ~20ms poll passes. Small frames fit the FIFO and survived without it;
 * large ones did not. Mirrors the MAIN-side bUseHwRxDMA ring. */
#define OWFW_DMA_RING_BITS  12
#define OWFW_DMA_RING_SIZE  (1u << OWFW_DMA_RING_BITS)
__attribute__((aligned(OWFW_DMA_RING_SIZE))) static uint8_t g_dma_ring[OWFW_DMA_RING_SIZE];
static int      g_dma_chan = -1;
static uint32_t g_dma_read_total = 0;

static void owfw_dma_start(void) {
    g_dma_chan = dma_claim_unused_channel(false);
    if (g_dma_chan < 0) return;   /* no free channel: fall back to polled reads */
    dma_channel_config c = dma_channel_get_default_config(g_dma_chan);
    channel_config_set_read_increment(&c, false);            /* fixed source: UART DR   */
    channel_config_set_write_increment(&c, true);            /* walk the ring           */
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, false));
    channel_config_set_ring(&c, true, OWFW_DMA_RING_BITS);   /* wrap the write address  */
    /* NORMAL 0x0FFFFFFF-byte transfer: bytes-written = 0x0FFFFFFF - remaining count. */
    dma_channel_configure(g_dma_chan, &c, g_dma_ring, &uart_get_hw(uart0)->dr,
                          0x0FFFFFFFu, true);
    g_dma_read_total = 0;
}

static void fifo_push_frame(owfw_fifo* f, const uint8_t* p, uint32_t n) {
    if (n > OWFW_STREAM_MAX - f->count) { g_dropped++; return; }  /* drop-newest, whole frame */
    for (uint32_t i = 0; i < n; i++) {
        f->buf[(f->head + f->count) % OWFW_STREAM_MAX] = p[i];
        f->count++;
    }
}

/* Re-assert hardware UART RTS ownership of GPIO3 if Wire1 (or anything) has
 * re-muxed it off the UART function. Cheap to re-check on every pump; only
 * counts a reclaim when it actually had to act. This board has no real I2C on
 * GPIO2/3 (the sensor bus is I2C1 on GPIO26/27), so taking GPIO3 back from
 * Wire1 costs nothing real. Nothing in the loop issues a Wire1 transaction
 * (only setup + one-time freewili_audio_init do), so this never fights a live
 * transfer. */
static void owfw_reclaim_rts(void) {
    if (gpio_get_function(OWFW_PIN_RTS) != GPIO_FUNC_UART) {
        gpio_set_function(OWFW_PIN_RTS, GPIO_FUNC_UART);
        uart_set_hw_flow(uart0, false, true);   /* RTS back on GPIO3 */
        g_owfw_rts_reclaims++;
    }
}

static void sdfs_rx_push(const uint8_t* p, uint16_t n) {
    if (n == 0 || n > SDFS_MAX_FRAME) { g_owfw_sdfs_drop++; return; }
    uint16_t next = (uint16_t)((g_sdfs_head + 1u) % OWFW_SDFS_SLOTS);
    if (next == g_sdfs_tail) { g_owfw_sdfs_drop++; return; }   /* full: drop */
    memcpy(g_sdfs[g_sdfs_head].buf, p, n);
    g_sdfs[g_sdfs_head].len = n;
    g_sdfs_head = next;
    g_owfw_sdfs_rx++;
}

static uint32_t fifo_pop(owfw_fifo* f, uint8_t* out, uint32_t cap) {
    uint32_t n = f->count < cap ? f->count : cap;
    for (uint32_t i = 0; i < n; i++) {
        out[i] = f->buf[f->head];
        f->head = (f->head + 1) % OWFW_STREAM_MAX;
        f->count--;
    }
    return n;
}

/* ── RX: BE BA command-frame parser ────────────────────────────────────── */
/* frame: BE BA | len u16le | cmd u8 | payload[len] | cksum u16le
 * checksum = 16-bit additive sum of sync(2)+length(2)+cmd(1)+payload. */
typedef enum { RX_SYNC0, RX_SYNC1, RX_LEN0, RX_LEN1, RX_CMD, RX_PAYLOAD, RX_CK0, RX_CK1 } owfw_rx_state;

static struct {
    owfw_rx_state st;
    uint16_t len, got, sum, ck;
    uint8_t  cmd;
    uint8_t  payload[OWFW_FRAME_MAX];
    int      overlong;           /* payload > OWFW_FRAME_MAX: parse, discard */
} g_rx;

static void rx_byte(uint8_t b) {
    switch (g_rx.st) {
    case RX_SYNC0:
        if (b == OWFW_CMD_SYNC0) { g_rx.sum = b; g_rx.st = RX_SYNC1; g_owfw_sync0++; }
        break;
    case RX_SYNC1:
        if (b == OWFW_CMD_SYNC1) { g_rx.sum += b; g_rx.st = RX_LEN0; }
        else g_rx.st = (b == OWFW_CMD_SYNC0) ? RX_SYNC1 : RX_SYNC0;
        break;
    case RX_LEN0: g_rx.sum += b; g_rx.len = b;               g_rx.st = RX_LEN1; break;
    case RX_LEN1:
        g_rx.sum += b; g_rx.len |= (uint16_t)(b << 8);
        g_rx.got = 0;
        g_rx.overlong = g_rx.len > OWFW_FRAME_MAX;
        g_rx.st = RX_CMD;
        break;
    case RX_CMD:
        g_rx.sum += b; g_rx.cmd = b;
        g_rx.st = g_rx.len ? RX_PAYLOAD : RX_CK0;
        break;
    case RX_PAYLOAD:
        g_rx.sum += b;
        if (!g_rx.overlong) g_rx.payload[g_rx.got] = b;
        if (++g_rx.got >= g_rx.len) g_rx.st = RX_CK0;
        break;
    case RX_CK0: g_rx.ck = b; g_rx.st = RX_CK1; break;
    case RX_CK1:
        g_rx.ck |= (uint16_t)(b << 8);
        if (g_rx.ck == g_rx.sum && !g_rx.overlong) {
            g_owfw_frames_ok++;
            if (g_rx.cmd == OWFW_CMD_RESPONSE) { g_owfw_resp++; fifo_push_frame(&g_text, g_rx.payload, g_rx.len); }
            else if (g_rx.cmd == OWFW_CMD_BINARY) fifo_push_frame(&g_binary, g_rx.payload, g_rx.len);
            else if (g_rx.cmd == OWFW_CMD_SDFS) sdfs_rx_push(g_rx.payload, g_rx.len);
            /* every other command code (GUI traffic) is discarded */
        }
        g_rx.st = RX_SYNC0;
        break;
    }
}

static void owfw_pump(void) {
    owfw_reclaim_rts();        /* keep GPIO3=UART RTS so MAIN stays clear to send */
    if (g_dma_chan < 0) {
        /* Fallback (no DMA channel): bounded polled drain. */
        for (uint32_t i = 0; i < OWFW_STREAM_MAX && uart_is_readable(uart0); i++)
            { g_owfw_bytes++; rx_byte((uint8_t)uart_getc(uart0)); }
        return;
    }
    /* DMA has been draining the FIFO into g_dma_ring; consume what it wrote. */
    uint32_t remaining = dma_channel_hw_addr(g_dma_chan)->transfer_count;
    uint32_t total_written = 0x0FFFFFFFu - remaining;
    uint32_t avail = total_written - g_dma_read_total;
    if (avail > OWFW_DMA_RING_SIZE) {           /* fell behind: keep newest ring-full */
        g_dma_read_total = total_written - OWFW_DMA_RING_SIZE;
        avail = OWFW_DMA_RING_SIZE;
    }
    if (avail > OWFW_STREAM_MAX) avail = OWFW_STREAM_MAX;   /* bound work per pump */
    for (uint32_t i = 0; i < avail; i++) {
        uint8_t b = g_dma_ring[g_dma_read_total & (OWFW_DMA_RING_SIZE - 1)];
        g_dma_read_total++;
        g_owfw_bytes++;
        rx_byte(b);
    }
}

/* ── TX: wrap command bytes into marked M_TERM_INPUT event frames ──────── */
/* frame: B0 1D | len u16le | payload | cksum u16le, where payload =
 * event code + marker + count + text and len counts payload EXCLUDING the
 * event code (per protocol.md); checksum covers sync+length+payload. */
static void owfw_send_chunk(const uint8_t* text, uint8_t n) {
    uint8_t f[2 + 2 + 3 + OWFW_CHUNK_MAX + 2];
    uint16_t len = (uint16_t)(2 + n);            /* marker + count + text */
    uint32_t k = 0;
    f[k++] = OWFW_EVT_SYNC0; f[k++] = OWFW_EVT_SYNC1;
    f[k++] = (uint8_t)(len & 0xFF); f[k++] = (uint8_t)(len >> 8);
    f[k++] = OWFW_EVT_TERM;
    f[k++] = OWFW_MARKER;
    f[k++] = n;
    memcpy(&f[k], text, n); k += n;
    uint16_t sum = 0;
    for (uint32_t i = 0; i < k; i++) sum = (uint16_t)(sum + f[i]);
    f[k++] = (uint8_t)(sum & 0xFF); f[k++] = (uint8_t)(sum >> 8);
    uart_write_blocking(uart0, f, k);
}

static int owfw_write(void* ctx, const uint8_t* data, size_t len) {
    (void)ctx;
    size_t off = 0;
    while (off < len) {
        size_t n = len - off;
        if (n > OWFW_CHUNK_MAX) n = OWFW_CHUNK_MAX;
        owfw_send_chunk(data + off, (uint8_t)n);
        off += n;
    }
    return (int)len;
}

static int owfw_read_stream(owfw_fifo* f, uint8_t* buf, size_t cap, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    for (;;) {
        owfw_pump();
        if (f->count) return (int)fifo_pop(f, buf, (uint32_t)cap);
        if (timeout_ms == 0 || time_reached(deadline)) return 0;
    }
}

/* ── SDFS data plane (event-42 up / 0x5F down) ─────────────────────────── */
/* Uplink frame: B0 1D | len u16le | 42 | sdfs_frame[len] | cksum u16le.
 * Same envelope as owfw_send_chunk but with the SDFS event code and NO 0x01
 * text marker / count byte and no 56-byte cap: this lane is binary-clean.
 * `len` counts the payload EXCLUDING the event code, so it is exactly the SDFS
 * frame length -- Main's processEventData hands pData[1..] to
 * serial_comm_main_rx with iLength-1 (rmpLib/fwgui_sdcard.cpp). */
static int owfw_sdfs_send(void* ctx, const uint8_t* frame, size_t len) {
    (void)ctx;
    if (len == 0 || len > SDFS_MAX_FRAME) return -1;
    uint8_t f[2 + 2 + 1 + SDFS_MAX_FRAME + 2];
    uint32_t k = 0;
    f[k++] = OWFW_EVT_SYNC0; f[k++] = OWFW_EVT_SYNC1;
    f[k++] = (uint8_t)(len & 0xFF); f[k++] = (uint8_t)(len >> 8);
    f[k++] = OWFW_EVT_SDFS;
    memcpy(&f[k], frame, len); k += len;
    uint16_t sum = 0;
    for (uint32_t i = 0; i < k; i++) sum = (uint16_t)(sum + f[i]);
    f[k++] = (uint8_t)(sum & 0xFF); f[k++] = (uint8_t)(sum >> 8);
    uart_write_blocking(uart0, f, k);
    g_owfw_sdfs_tx++;
    return 0;
}

static int owfw_sdfs_recv(void* ctx, uint8_t* buf, size_t cap, size_t* len) {
    (void)ctx;
    owfw_pump();                                  /* the client's only pump */
    if (g_sdfs_tail == g_sdfs_head) return 0;     /* idle */
    owfw_sdfs_slot* s = &g_sdfs[g_sdfs_tail];
    int rc = 1;
    if (s->len > cap) rc = -1;                    /* too big: discard, report */
    else { memcpy(buf, s->buf, s->len); *len = s->len; }
    g_sdfs_tail = (uint16_t)((g_sdfs_tail + 1u) % OWFW_SDFS_SLOTS);
    return rc;
}

static int owfw_read_text(void* ctx, uint8_t* buf, size_t cap, uint32_t timeout_ms) {
    (void)ctx; return owfw_read_stream(&g_text, buf, cap, timeout_ms);
}
static int owfw_read_binary(void* ctx, uint8_t* buf, size_t cap, uint32_t timeout_ms) {
    (void)ctx; return owfw_read_stream(&g_binary, buf, cap, timeout_ms);
}

/* ── Public API ────────────────────────────────────────────────────────── */
/* Bring up the physical FwGUI link (uart0 + pins + hardware RTS + RX DMA) and
 * clear the demux FIFOs. Idempotent: SD-backed persistence (freewili_persist)
 * needs the link up BEFORE NodeDB loads config at boot, which is well before
 * ow_open_fwgui runs for the wardrive. Both call this; the first wins, the
 * second is a no-op, so neither leaks a second DMA channel nor resets a
 * live link's state. */
static bool s_link_up = false;
void ow_fwgui_link_ensure(void) {
    if (s_link_up)
        return;
    memset(&g_rx, 0, sizeof g_rx);
    memset(&g_text, 0, sizeof g_text);
    memset(&g_binary, 0, sizeof g_binary);
    memset(g_sdfs, 0, sizeof g_sdfs);
    g_sdfs_head = g_sdfs_tail = 0;
    g_dropped = 0;
    uart_init(uart0, 8000000);   /* OWFW_BAUD */
    gpio_set_function(OWFW_PIN_TX,  GPIO_FUNC_UART);
    gpio_set_function(OWFW_PIN_RX,  GPIO_FUNC_UART);
    gpio_set_function(OWFW_PIN_CTS, GPIO_FUNC_UART);   /* MAIN's RTS in (unused) */
    gpio_set_function(OWFW_PIN_RTS, GPIO_FUNC_UART);   /* our RTS -> MAIN's CTS   */
    /* Hardware RTS on GPIO3, CTS off: MAIN's TX is not gated by our CTS (we drain
     * fast via DMA), but our hardware RTS holds GPIO3 asserted so MAIN's CTS stays
     * clear-to-send. Matches the known-good pre-SDFS image. */
    uart_set_hw_flow(uart0, false, true);
    owfw_dma_start();                        /* continuously drain the RX FIFO into g_dma_ring */
    s_link_up = true;
}

ow_status ow_open_fwgui(ow_device* dev) {
    ow_fwgui_link_ensure();
    ow_transport t;
    t.ctx = 0;
    t.write = owfw_write;
    t.read = owfw_read_text;
    return ow_open(dev, &t);
}

ow_transport ow_fwgui_binary_transport(void) {
    ow_transport t;
    t.ctx = 0;
    t.write = 0;                  /* the binary stream is read-only */
    t.read = owfw_read_binary;
    return t;
}

sdfs_transport_t ow_fwgui_sdfs_transport(void) {
    sdfs_transport_t t;
    t.send = owfw_sdfs_send;
    t.recv = owfw_sdfs_recv;
    t.ctx  = 0;
    return t;
}

uint32_t ow_fwgui_dropped_frames(void) { return g_dropped; }
