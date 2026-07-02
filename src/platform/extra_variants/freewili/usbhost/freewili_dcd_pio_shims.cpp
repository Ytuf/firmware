// FreeWili 2 — missing-symbol shims for TinyUSB's PIO-USB device driver.
//
// usbd.c references dcd_sof_enable / dcd_edpt_iso_alloc / dcd_edpt_iso_activate,
// which dcd_pio_usb.c does not define. Without these, the linker hunts the
// precompiled framework archive (libpico.a), drags in the pico-sdk's NATIVE
// dcd_rp2040.o to satisfy them, and every other dcd_* symbol becomes multiply
// defined. Project objects are linked before archives, so defining the three
// gaps here keeps libpico.a's member out entirely.
//
// All three are safe no-ops for this build: the PIO-USB device exposes only
// CDC (no isochronous endpoints), and nothing consumes SOF callbacks.

#include "configuration.h"

#if defined(FREEWILI) && defined(USE_TINYUSB_HOST)

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUD_RPI_PIO_USB

#include "device/dcd.h"

extern "C" {

// The core's main() hardcodes TinyUSB_Device_Init(0), but dcd_pio_usb tags
// every event it posts with rhport = root_id + 1 = 1 (dcd_pio_usb.c:178).
// A device stack registered on rhport 0 silently drops those events — the
// PC sees the attach (pull-up) but every descriptor request times out
// ("Device Descriptor Request Failed", observed live). Redirect the init to
// rhport 1 so usbd and the PIO dcd agree; rhport 0 stays purely HOST.
// Wired up via -Wl,--wrap=TinyUSB_Device_Init in platformio.ini.
void __real_TinyUSB_Device_Init(uint8_t rhport);
void __wrap_TinyUSB_Device_Init(uint8_t rhport)
{
    (void)rhport;
    __real_TinyUSB_Device_Init(1);
}

// tusb.c's tusb_int_handler() references this strongly, but it can never run:
// it dispatches here only when _tusb_rhport_role[rhport]==DEVICE, and the
// native controller IRQ only raises tusb_int_handler(0) whose role is HOST —
// the PIO device's events flow through pio-usb's own IRQ into the usbd queue.
void dcd_int_handler(uint8_t rhport)
{
    (void)rhport;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
    (void)rhport;
    (void)en;
}

bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size)
{
    (void)rhport;
    (void)ep_addr;
    (void)largest_packet_size;
    return false; // no isochronous endpoints on the PIO-USB CDC device
}

bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const *ep_desc)
{
    (void)rhport;
    (void)ep_desc;
    return false;
}

} // extern "C"

#endif // CFG_TUD_ENABLED && CFG_TUD_RPI_PIO_USB
#endif // FREEWILI && USE_TINYUSB_HOST
