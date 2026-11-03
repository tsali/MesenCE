/* HARBOR: the ships round an airport, for a NES Radar that shows ships
 * (docs/PROTOCOL.md).
 *
 * It is RADAR (radar.c) with ships in place of aircraft: the same ops, the
 * same states and flags, and the same nine-byte motion and twenty-five-byte
 * names, byte for byte as nes-radar's ROM has them. What is its own is
 * here: the fetch of AISHub's web service, and what of a vessel goes where
 * in the records; and in radar.c, the ways a place may be chosen.
 *
 * PLACE (RADAR's AIRPORT, op 0) takes ID, KIND, the place, and a RANGE or
 * not. KIND 0 is an airport's four letters, found through Wikidata as
 * RADAR finds one; 1 a US zip code's five digits, found through Zippopotam
 * as WEATHER finds one; 2 a latitude and a longitude, four bytes each, low
 * first, in millionths of a degree, which is found at once. RANGE is the
 * scope's edge in nautical miles, 1 to NESP_HARBOR_RANGE_MAX, and 0 or none
 * is NESP_HARBOR_RANGE_NM. RADAR's own ID, LETTERS[4] is KIND 0 at 20 nm.
 * The same place again with another range only draws the scene again.
 * LAST is the last place found, as PLACE takes it after its ID.
 *
 * AISHub's reply is a list: a header, {"ERROR":false,...,"RECORDS":n}, and
 * then a list of vessels. radar.c reads it as it comes and hands each
 * object here whole; a vessel longer than NESP_RADAR_ITEM is passed over.
 * A header that says ERROR, or none at all, is a fetch that failed.
 *
 * AISHub answers one request a minute, so there is one a minute at most,
 * NESP_HARBOR_POLL_MS, whatever airport it is for; a scene is stale when
 * nothing good has come for NESP_HARBOR_STALE_MS.
 *
 * The key (an AISHub username) is the board's, never the console's: the
 * hal's setting "aishub_key". With none, the state is NO_KEY and nothing
 * is fetched.
 *
 * The scope's edge is RANGE, NESP_HARBOR_RANGE_NM (20 nm) unless PLACE
 * says, not RADAR's 9, and AISHub is asked for a box a little larger than
 * that circle. A vessel is
 * left out if it is an aid to navigation (an MMSI of 99xxxxxxx), a base
 * station (00xxxxxxx, so under 10000000), or has no position (91 and 181
 * are AIS's "not available"). A ship that is moored or at anchor stays: a
 * ship standing still is still a ship. The nearest eight are kept, the
 * nearest in slot 0, as RADAR keeps aircraft.
 *
 * The records, nes-radar's, with a ship's things in an aircraft's places:
 *
 *   MOTION   SLOT_FLAGS  the slot; altitude is never known (bit 4), and
 *                        track (bit 3) and speed (bit 5) when AIS has none
 *            X, Y        on the scope, RANGE to its edge
 *            TRACK       COG, or HEADING when there is no COG (360) and
 *                        HEADING is not 511, as 0..255 for 0..<360 degrees
 *            ALT         0, and not known
 *            SPEED       SOG in knots (102.3 is "not available")
 *            RATE        $80, not known
 *            DISTANCE    the true distance, in tenths of a nautical mile,
 *                        clipped at 99 as nes-radar's field is: a ship
 *                        9.9 nm off or more says 99
 *
 *   RECORD   CALLSIGN[8] NAME; or with none, the radio call sign; or with
 *                        neither, the MMSI's last eight digits
 *            TYPE[4]     the AIS ship type as a word: FISH, TOW, DRDG,
 *                        DIVE, MIL, SAIL, PLSR, HSC, PILT, SAR, TUG, TNDR,
 *                        ANTP, LAW, MED, SPCL, PASS, CRGO, TNKR, OTHR, WIG
 *            REG[6]      the radio call sign (CALLSIGN), cut to six. The
 *                        MMSI, nine digits, does not fit
 *            SQUAWK[4]   NAVSTAT as a word: UWAY, ANCH, NUC, RMAN, CBD,
 *                        MOOR, AGRD, FISH, SAIL, SART; blank for the rest
 *            CATEGORY[2] the AIS ship type's two digits (70 is cargo),
 *                        blank when it is 0 or not one
 *
 * all in capitals, digits, spaces and hyphens, as RADAR's are.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "nesp_services.h"
#include "scope.h"

_Static_assert(NESP_HARBOR_PLACE == NESP_RADAR_AIRPORT && NESP_HARBOR_SCENE == NESP_RADAR_SCENE
                   && NESP_HARBOR_IDENTITY == NESP_RADAR_IDENTITY
                   && NESP_HARBOR_LAST == NESP_RADAR_LAST,
               "HARBOR's ops are RADAR's");

#define SOURCE "https://data.aishub.net/ws.php?username="
#define KEY_NAME "aishub_key"
#define MARGIN_NM 2.0              /* the box asked for is this much more than the range */
#define NO_COURSE 360.0             /* AIS's "not available" */
#define NO_HEADING 511.0
#define NO_SOG 102.3

/* --- the key --------------------------------------------------------------------- */

static bool is_key_letter(char letter)
{
    return (letter >= 'A' && letter <= 'Z') || (letter >= 'a' && letter <= 'z')
           || (letter >= '0' && letter <= '9') || letter == '_' || letter == '-' || letter == '.';
}

/* Whether the board has a key, which is then key_text. One with anything in
 * it that would need escaping in an address is none. */
bool harbor_key(nesp_radar_t *radar)
{
    char text[NESP_HARBOR_KEY_MAX + 1];
    bool keyed = radar->hal->setting != NULL
                 && radar->hal->setting(radar->hal->ctx, KEY_NAME, text, sizeof(text));
    if (keyed) {
        text[sizeof(text) - 1] = 0;
        size_t len = strlen(text);
        keyed = len > 0;
        for (size_t index = 0; index < len && keyed; index++) {
            keyed = is_key_letter(text[index]);
        }
    }
    radar->keyed = keyed;
    if (keyed) {
        memcpy(radar->key_text, text, sizeof(radar->key_text));
    } else {
        memset(radar->key_text, 0, sizeof(radar->key_text));
    }
    return keyed;
}

/* --- the fetch ------------------------------------------------------------------- */

static double between(double value, double least, double most)
{
    return value < least ? least : value > most ? most : value;
}

/* AISHub's address for the vessels in a box round the airport, a little
 * larger than the scope's circle, in the last ten minutes. The box stops
 * at the poles and at 180 degrees: it does not wrap. */
void harbor_url(const nesp_radar_t *radar, char *url, size_t size)
{
    double north = (radar->range_nm + MARGIN_NM) / 60.0;
    double across = cos(scope_radians(radar->here[0]));
    double east = across > 0.01 ? north / across : 180.0;
    double lon_min = -180.0, lon_max = 180.0;
    if (east < 180.0) {
        lon_min = between(radar->here[1] - east, -180.0, 180.0);
        lon_max = between(radar->here[1] + east, -180.0, 180.0);
    }
    snprintf(url, size,
             SOURCE "%s&format=1&output=json&compress=0&latmin=%.6f&latmax=%.6f"
                    "&lonmin=%.6f&lonmax=%.6f&interval=10",
             radar->key_text, between(radar->here[0] - north, -90.0, 90.0),
             between(radar->here[0] + north, -90.0, 90.0), lon_min, lon_max);
}

/* --- the reply ------------------------------------------------------------------- */

/* The header: whether AISHub says all is well. */
void harbor_header(nesp_radar_t *radar, const char *item, size_t len)
{
    scope_value_t value;
    if (!scope_member(item, len, "ERROR", &value) || value.string) {
        return;
    }
    if (value.len == 5 && memcmp(value.at, "false", 5) == 0) {
        radar->vouched = true;
    } else {
        radar->refused = true;
    }
}

/* A text of a vessel's, with no "@" (AIS's padding) or spaces after it.
 * False if it has none, or nothing but those. */
static bool text_of(const char *item, size_t len, const char *name, scope_value_t *out)
{
    if (!scope_member(item, len, name, out) || !out->string) {
        return false;
    }
    while (out->len > 0 && (out->at[out->len - 1] == '@' || out->at[out->len - 1] == ' ')) {
        out->len--;
    }
    return out->len > 0;
}

static void put_word(const char *word, uint8_t *out, size_t width)
{
    scope_value_t value = {word, strlen(word), true};
    scope_put_name(&value, out, width);
}

/* The AIS ship type (ITU-R M.1371, table 53) as a word of four. */
static const char *type_word(long type)
{
    static const char *const special[] = {
        "FISH", "TOW", "TOW", "DRDG", "DIVE", "MIL", "SAIL", "PLSR",    /* 30 to 37 */
    };
    static const char *const service[] = {
        "PILT", "SAR", "TUG", "TNDR", "ANTP", "LAW", "", "", "MED", "SPCL",  /* 50 to 59 */
    };
    if (type >= 20 && type <= 29) {
        return "WIG";
    }
    if (type >= 30 && type <= 37) {
        return special[type - 30];
    }
    if (type >= 40 && type <= 49) {
        return "HSC";
    }
    if (type >= 50 && type <= 59) {
        return service[type - 50];
    }
    if (type >= 60 && type <= 69) {
        return "PASS";
    }
    if (type >= 70 && type <= 79) {
        return "CRGO";
    }
    if (type >= 80 && type <= 89) {
        return "TNKR";
    }
    if (type >= 90 && type <= 99) {
        return "OTHR";
    }
    return "";
}

/* The navigational status as a word of four. */
static const char *status_word(long status)
{
    static const char *const words[] = {
        "UWAY", "ANCH", "NUC", "RMAN", "CBD", "MOOR", "AGRD", "FISH",
        "SAIL", "", "", "", "", "", "SART", "",
    };
    return status >= 0 && status <= 15 ? words[status] : "";
}

/* One vessel of the reply, whole. */
void harbor_vessel(nesp_radar_t *radar, const char *item, size_t len)
{
    double mmsi, latitude, longitude, value;
    if (!scope_number(item, len, "MMSI", &mmsi) || mmsi < 10000000.0 || mmsi >= 990000000.0) {
        return;                     /* a base station, an aid to navigation, or nothing */
    }
    if (!scope_number(item, len, "LATITUDE", &latitude)
        || !scope_number(item, len, "LONGITUDE", &longitude) || !(latitude >= -90.0)
        || !(latitude <= 90.0) || !(longitude >= -180.0) || !(longitude <= 180.0)) {
        return;                     /* 91 and 181 are "not available" */
    }
    nesp_radar_target_t *target = scope_spot(radar, latitude, longitude, radar->range_nm);
    if (target == NULL) {
        return;
    }
    uint8_t *motion = target->motion;
    if ((scope_number(item, len, "COG", &value) && value >= 0.0 && value < NO_COURSE)
        || (scope_number(item, len, "HEADING", &value) && value >= 0.0 && value < 360.0)) {
        motion[3] = (uint8_t)((unsigned long)scope_rounded(value * 256 / 360, 0, 65536) & 0xFF);
    } else {
        motion[0] |= NO_TRACK;
    }
    motion[0] |= NO_ALTITUDE;
    if (scope_number(item, len, "SOG", &value) && value >= 0.0 && value < NO_SOG) {
        motion[6] = (uint8_t)scope_rounded(value, 0, 255);
    } else {
        motion[0] |= NO_SPEED;
    }
    motion[7] = NO_RATE;

    uint8_t *names = target->names;
    scope_value_t name, call;
    bool has_call = text_of(item, len, "CALLSIGN", &call);
    if (text_of(item, len, "NAME", &name)) {
        scope_put_name(&name, names + 1, 8);
    } else if (has_call) {
        scope_put_name(&call, names + 1, 8);
    } else {
        char digits[16];
        snprintf(digits, sizeof(digits), "%09ld", scope_rounded(mmsi, 0, 999999999));
        put_word(digits + strlen(digits) - 8, names + 1, 8);
    }
    long type = scope_number(item, len, "TYPE", &value) ? scope_rounded(value, -1, 1000) : 0;
    put_word(type_word(type), names + 9, 4);
    if (has_call) {
        scope_put_name(&call, names + 13, 6);
    } else {
        put_word("", names + 13, 6);
    }
    long status = scope_number(item, len, "NAVSTAT", &value) ? scope_rounded(value, -1, 16) : 15;
    put_word(status_word(status), names + 19, 4);
    char category[4] = "";
    if (type >= 1 && type <= 99) {
        snprintf(category, sizeof(category), "%02ld", type);
    }
    put_word(category, names + 23, 2);
}

/* One place of Zippopotam's answer for a zip code: the first is where the
 * zip code is, written as Wikidata writes a point, for radar.c to read. */
void harbor_zip(nesp_radar_t *radar, const char *item, size_t len)
{
    double latitude, longitude;
    if (radar->spotted || !scope_number(item, len, "latitude", &latitude)
        || !scope_number(item, len, "longitude", &longitude)) {
        return;
    }
    snprintf(radar->spot, sizeof(radar->spot), "%.7f %.7f", longitude, latitude);
    radar->spotted = true;
}

void nesp_harbor_service(nesp_service_t *service, nesp_harbor_t *harbor, const nesp_hal_t *hal)
{
    scope_service(service, harbor, hal, true);
}
