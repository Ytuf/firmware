/* FwGUI display-link transport for the OneWili C API (WiliBSP / RP2350B).
 * Talks to the FreeWili 2 main CPU over UART0 (Rx=GPIO0, Tx=GPIO1,
 * RTS=GPIO2, CTS=GPIO3) at 8,000,000 baud with hardware flow control.
 * Single link per board: the state behind these transports is static. */
#ifndef ONEWILI_FWGUI_H
#define ONEWILI_FWGUI_H
#include "onewili.h"
#include "sdfs_transport.h"

/* onewili_fwgui.c is built as C (arm-none-eabi-gcc); the freewili variant's
 * consumer (freewili_wifi.cpp) is C++. Upstream wilibsp's onewili.h guards
 * its declarations with extern "C" but this file didn't -- without it, a C++
 * includer mangles these names and the link fails against the C object file.
 * Fixed here in the vendored copy only; not touched in freewilimain's wilibsp. */
#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the physical FwGUI link (uart0 + pins + hardware RTS + RX DMA) once.
 * Idempotent. Call this before using ow_fwgui_sdfs_transport() outside of
 * ow_open_fwgui (e.g. SD-backed persistence at boot, before the wardrive opens
 * the link). ow_open_fwgui calls it too. */
void ow_fwgui_link_ensure(void);

/* Inits UART0 + pins, installs the command transport, and calls ow_open
 * (which performs the 0x02 reset handshake). Call once at startup. */
ow_status ow_open_fwgui(ow_device* dev);

/* The binary-event stream (FWGUI_API_ONEWILL_BINARY frames) as a transport
 * for ow_binary_open/ow_binary_poll. Valid after ow_open_fwgui. */
ow_transport ow_fwgui_binary_transport(void);

/* The SDFS data plane (FwGUI event-42 up / command 0x5F down) as an
 * sdfs_transport_t, for binding an sdfs_client to MAIN's SD card.
 * recv() pumps the shared link, so it also advances the text/binary lanes.
 * Valid after ow_open_fwgui. */
sdfs_transport_t ow_fwgui_sdfs_transport(void);

/* Frames dropped because a stream buffer was full (poll more often). */
uint32_t ow_fwgui_dropped_frames(void);

#ifdef __cplusplus
}
#endif

#endif /* ONEWILI_FWGUI_H */
