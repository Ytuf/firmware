#pragma once

// Pure parser for the OneWili "[*wifiscan ...]" event body. Factored out of
// freewili_wifi.cpp (which pulls in Arduino/pico headers) so it can be
// exercised in a plain host build. Depends only on freewili_wifi.h.
//
// MAIN builds the event via fwMenuWifi.cpp addEventData("wifiscan", szText,
// true), which rpConsole::printEventResponse wraps as:
//   [*wifiscan <ts> <seq> <bssid> <rssi> <channel> <band> <authmode> <ssid> <ok>]
// wilibsp's ow_poll_text_line() strips the "[*wifiscan " prefix and the
// trailing "]" before handing back args, so parseWifiscanEvent only ever
// sees the part from <ts> onward.

#include "freewili_wifi.h"

// args: "<ts> <seq> <bssid> <rssi> <channel> <band> <authmode> <ssid...> <ok>".
// ts/seq/ok are consumed but not returned (the AP table timestamps locally).
// ssid is bounded-split from the back (last token is <ok>), not tokenized on
// spaces, since an SSID may contain spaces or be empty. Returns false on any
// malformed frame; *out is left untouched in that case.
bool parseWifiscanEvent(const char *args, FreewiliWifiAp &out);
