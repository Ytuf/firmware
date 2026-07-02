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

// tusb.c's tusb_int_handler() references this strongly, but it can never run:
// it dispatches here only when _tusb_rhport_role[rhport]==DEVICE, and after
// boot completes role[0]==HOST (tuh_init overwrites the core's early device
// registration) while nothing raises tusb_int_handler for rhport 1 — the PIO
// device's events flow through pio-usb's own IRQ into the usbd queue instead.
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
