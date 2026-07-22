#pragma once

// SD-backed persistence for the FreeWili 2 (Display side).
//
// Meshtastic's normal persistence (config/nodedb/keypair) lives in LittleFS on
// the RP2350's internal flash, but that is disabled on this port because
// arduino-pico's LittleFS hangs on flash write (NodeDB.cpp saveProto/saveToDisk
// FREEWILI stubs). This module persists those proto blobs to the real microSD
// card instead, reached over the same SDFS link to MAIN that the wardrive uses.
//
// NodeDB::saveProto / loadProto route through here for the files we persist.
// Paths are the Meshtastic LittleFS names (e.g. "/prefs/config.proto"); this
// module maps them under an SD directory. Blobs are small (config < 1 KB).

#include <stdint.h>
#include <stddef.h>

// Bring up the SDFS link (idempotent) so a boot-time load works before the
// wardrive opens the link. Call once, early in setup(), BEFORE `new NodeDB`.
void freewili_persist_init();

// Read a persisted blob into buf. Returns bytes read (0 if absent/failed).
// Blocking (paced SDFS round trips); call only from the main loop / boot.
size_t freewili_persist_read(const char *littlefs_path, uint8_t *buf, size_t cap);

// Write a blob (truncating any existing file). Returns true on a byte-count-
// verified success. Blocking. Creates the SD directory on demand.
bool freewili_persist_write(const char *littlefs_path, const uint8_t *buf, size_t len);

// SWD-observable diagnostics.
extern "C" {
extern volatile uint32_t g_fwpersist_reads_ok;
extern volatile uint32_t g_fwpersist_reads_fail;
extern volatile uint32_t g_fwpersist_writes_ok;
extern volatile uint32_t g_fwpersist_writes_fail;
extern volatile int32_t  g_fwpersist_last_status;
extern volatile uint32_t g_fwpersist_last_read_len;
extern volatile uint32_t g_fwpersist_last_write_len;
}
