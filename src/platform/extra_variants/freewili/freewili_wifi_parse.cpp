#include "freewili_wifi_parse.h"

#include <string.h>
#include <stdlib.h>

namespace
{

int hexNibble(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

// "aa:bb:cc:dd:ee:ff" (exactly 17 chars) -> 6 bytes.
bool parseBssid(const char *tok, size_t len, uint8_t bssid[6])
{
    if (len != 17)
        return false;
    for (int i = 0; i < 6; i++) {
        int hi = hexNibble(tok[i * 3]);
        int lo = hexNibble(tok[i * 3 + 1]);
        if (hi < 0 || lo < 0)
            return false;
        if (i < 5 && tok[i * 3 + 2] != ':')
            return false;
        bssid[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

// Advances *cur past the next single-space-delimited token; *tok/*toklen
// receive it (not NUL-terminated -- bounded by toklen). False if *cur runs
// out before a delimiting space is found.
bool nextToken(const char **cur, const char **tok, size_t *toklen)
{
    const char *start = *cur;
    const char *sp = strchr(start, ' ');
    if (!sp)
        return false;
    *tok = start;
    *toklen = (size_t)(sp - start);
    *cur = sp + 1;
    return true;
}

// Parses a bounded decimal token into *out. Rejects empty or trailing-junk
// tokens (a plain strtol would silently accept "12abc").
bool tokenToLong(const char *tok, size_t toklen, long *out)
{
    char buf[16];
    if (toklen == 0 || toklen >= sizeof(buf))
        return false;
    memcpy(buf, tok, toklen);
    buf[toklen] = 0;
    char *end = nullptr;
    *out = strtol(buf, &end, 10);
    return end && *end == 0;
}

} // namespace

bool parseWifiscanEvent(const char *args, FreewiliWifiAp &out)
{
    if (!args)
        return false;

    const char *cur = args;
    const char *tok;
    size_t toklen;
    long value;

    if (!nextToken(&cur, &tok, &toklen)) // ts (unused)
        return false;
    if (!nextToken(&cur, &tok, &toklen)) // seq (unused)
        return false;

    const char *bssidTok;
    size_t bssidLen;
    if (!nextToken(&cur, &bssidTok, &bssidLen)) // bssid
        return false;

    if (!nextToken(&cur, &tok, &toklen) || !tokenToLong(tok, toklen, &value)) // rssi
        return false;
    long rssi = value;

    if (!nextToken(&cur, &tok, &toklen) || !tokenToLong(tok, toklen, &value)) // channel
        return false;
    long channel = value;

    if (!nextToken(&cur, &tok, &toklen) || !tokenToLong(tok, toklen, &value)) // band
        return false;
    long band = value;

    if (!nextToken(&cur, &tok, &toklen) || !tokenToLong(tok, toklen, &value)) // authmode
        return false;
    long authmode = value;

    // Remainder is "<ssid...> <ok>". Bounded-split from the back: the SSID
    // (everything up to the last space) may contain spaces or be empty; see
    // fwMenuWifi.cpp's "SSID last so a consumer can bounded-split it" note.
    size_t remLen = strlen(cur);
    if (remLen == 0)
        return false;
    const char *lastSpace = nullptr;
    for (const char *p = cur + remLen - 1; p >= cur; p--) {
        if (*p == ' ') {
            lastSpace = p;
            break;
        }
    }
    if (!lastSpace)
        return false;

    uint8_t bssid[6];
    if (!parseBssid(bssidTok, bssidLen, bssid))
        return false;

    size_t ssidLen = (size_t)(lastSpace - cur);
    if (ssidLen >= FW_WIFI_SSID_LEN)
        ssidLen = FW_WIFI_SSID_LEN - 1;

    memcpy(out.bssid, bssid, 6);
    out.rssi = (int8_t)rssi;
    out.channel = (uint8_t)channel;
    out.band = (uint8_t)band;
    out.authmode = (uint8_t)authmode;
    memcpy(out.ssid, cur, ssidLen);
    out.ssid[ssidLen] = 0;
    return true;
}
