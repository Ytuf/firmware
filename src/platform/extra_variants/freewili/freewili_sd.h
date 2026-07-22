#pragma once

// FreeWili wardrive -> SD card (Display side).
//
// The only real removable card on this board hangs off MAIN's SDIO (FatFs
// volume "1:"). MAIN already runs an SDFS server on the Display link (request
// frames up as FwGUI event-42, response frames down as command 0x5F), so the
// Display reaches the card by speaking SDFS over that link -- see
// ow_fwgui_sdfs_transport() in wilibsp/src/onewili_fwgui.c.
//
// Paths are volume-relative: MAIN's storage adapter prepends SDFS_VOL_PREFIX
// ("1:") itself (sdfslib/host/sdfs_storage_fatfs.c sdfs_make_path), so we send
// "/wardrive/survey.csv", NOT "1:/wardrive/survey.csv".

#include <stdint.h>
#include "freewili_wifi.h"

// Queue one CSV row for a newly discovered AP. Pure memcpy into a staging
// buffer -- never touches the link, never blocks. Safe to call from upsertAp.
void freewili_sd_note_ap(const FreewiliWifiAp &ap);

// Do at most one bounded flush of staged rows to the card. Blocking SDFS round
// trips happen here and nowhere else, so this must be called from the wardrive
// OSThread tick (after its event drain), never from a parse/insert path.
void freewili_sd_service();
