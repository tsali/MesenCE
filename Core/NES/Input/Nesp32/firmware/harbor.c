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
 *
 * DETAILS is a ship's card, as AIS sites show one: NESP_HARBOR_DETAIL_LINES
 * lines of NESP_HARBOR_DETAIL_WIDTH characters, in capitals, digits, spaces
 * and + - . / :, a hyphen for what AIS does not say. What it says is kept
 * with the ship (nesp_harbor_ship_t), a few bytes a field and not the
 * vessel's text, and goes with the ship wherever the scene puts it; a line
 * is written out only when it is asked for. The flag is the country of the
 * MMSI's first three digits.
 *
 * The map (MAP and PAN) is land and water under the scope, made from
 * OpenStreetMap's standard tiles. The scope's square, 144 points across,
 * is 36 by 36 cells of four points, and a cell is water when most of its
 * nine samples are OpenStreetMap's water colour, or the blue of a line it
 * draws over the sea. A sample is a point
 * of the scope put on the earth as the box asked of AISHub is: a degree of
 * latitude is sixty miles, and a degree of longitude that less by the
 * cosine of the middle's latitude. The tiles are of the largest zoom at
 * which the square is on no more than two by two of them, and only those
 * the samples are on are fetched, one after another. Each is read as it
 * comes (png.c), a row at a time, and never held: a row with samples on
 * it is counted into the cells, and the rest are passed over.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nesp_png.h"
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

/* What of a vessel its card says (below). */
static void keep_ship(nesp_harbor_ship_t *ship, const char *item, size_t len, double mmsi,
                      long type, long status);

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
    keep_ship(&target->ship, item, len, mmsi, type, status);
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

/* --- the card ------------------------------------------------------------------ */

#define WIDTH NESP_HARBOR_DETAIL_WIDTH
#define NO_SOG_TENTHS 1023
#define NO_COG_TENTHS 3600
#define NO_STATUS 15

/* A text of a vessel's, as the card keeps it: in the card's characters,
 * spaces after it, and all spaces if it has none. */
static void keep_text(const char *item, size_t len, const char *name, char *out, size_t width)
{
    scope_value_t value;
    if (text_of(item, len, name, &value)) {
        scope_put_text(&value, (uint8_t *)out, width);
    } else {
        memset(out, ' ', width);
    }
}

/* Seconds since 1970 of AISHub's TIME, "2026-11-02 23:04:31 GMT". 0 if it
 * is not one. */
static uint32_t heard_at(const char *item, size_t len)
{
    static const char shape[] = "dddd-dd-dd dd:dd:dd";
    scope_value_t value;
    if (!scope_member(item, len, "TIME", &value) || !value.string
        || value.len < sizeof(shape) - 1) {
        return 0;
    }
    long field[6] = {0};
    for (size_t at = 0, which = 0; at < sizeof(shape) - 1; at++) {
        char letter = value.at[at];
        if (shape[at] != 'd') {
            if (letter != shape[at]) {
                return 0;
            }
            which++;
        } else if (letter >= '0' && letter <= '9') {
            field[which] = field[which] * 10 + (letter - '0');
        } else {
            return 0;
        }
    }
    long year = field[0], month = field[1], day = field[2];
    if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 || field[3] > 23
        || field[4] > 59 || field[5] > 60) {
        return 0;
    }
    /* Days since 1970 of a day of the Gregorian calendar. */
    year -= month <= 2;
    long era = year / 400;
    long of_era = year - era * 400;
    long of_year = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    long days = era * 146097 + of_era * 365 + of_era / 4 - of_era / 100 + of_year - 719468;
    int64_t seconds = (int64_t)days * 86400 + field[3] * 3600 + field[4] * 60 + field[5];
    return seconds > 0 && seconds <= (int64_t)UINT32_MAX ? (uint32_t)seconds : 0;
}

/* What of a vessel its card says. */
static void keep_ship(nesp_harbor_ship_t *ship, const char *item, size_t len, double mmsi,
                      long type, long status)
{
    double value, other;
    ship->mmsi = (uint32_t)scope_rounded(mmsi, 0, 999999999);
    ship->imo = scope_number(item, len, "IMO", &value) && value >= 1.0 && value <= 999999999.0
                    ? (uint32_t)scope_rounded(value, 1, 999999999)
                    : 0;
    ship->heard = heard_at(item, len);
    long a = scope_number(item, len, "A", &value) ? scope_rounded(value, 0, 511) : 0;
    long b = scope_number(item, len, "B", &other) ? scope_rounded(other, 0, 511) : 0;
    ship->length = (uint16_t)(a + b);
    long c = scope_number(item, len, "C", &value) ? scope_rounded(value, 0, 63) : 0;
    long d = scope_number(item, len, "D", &other) ? scope_rounded(other, 0, 63) : 0;
    ship->beam = (uint8_t)(c + d);
    ship->draught = scope_number(item, len, "DRAUGHT", &value) && value > 0.0
                        ? (uint8_t)scope_rounded(value * 10, 0, 255)
                        : 0;
    ship->sog = scope_number(item, len, "SOG", &value) && value >= 0.0 && value < NO_SOG
                    ? (uint16_t)scope_rounded(value * 10, 0, NO_SOG_TENTHS - 1)
                    : NO_SOG_TENTHS;
    ship->cog = scope_number(item, len, "COG", &value) && value >= 0.0 && value < NO_COURSE
                    ? (uint16_t)scope_rounded(value * 10, 0, NO_COG_TENTHS - 1)
                    : NO_COG_TENTHS;
    ship->heading = scope_number(item, len, "HEADING", &value) && value >= 0.0 && value < 360.0
                        ? (uint16_t)scope_rounded(value, 0, 359)
                        : (uint16_t)NO_HEADING;
    ship->type = (uint8_t)(type >= 0 && type <= 255 ? type : 0);
    ship->status = (uint8_t)(status >= 0 && status <= NO_STATUS ? status : NO_STATUS);
    keep_text(item, len, "NAME", ship->name, sizeof(ship->name));
    keep_text(item, len, "CALLSIGN", ship->call, sizeof(ship->call));
    keep_text(item, len, "DEST", ship->dest, sizeof(ship->dest));
    keep_text(item, len, "ETA", ship->eta, sizeof(ship->eta));
}

/* The AIS ship type (ITU-R M.1371, table 53) in words. */
static const char *type_words(long type)
{
    static const char *const special[] = {
        "FISHING", "TOWING", "TOWING", "DREDGING", "DIVING", "MILITARY", "SAILING",
        "PLEASURE CRAFT",                                               /* 30 to 37 */
    };
    static const char *const service[] = {
        "PILOT", "SEARCH AND RESCUE", "TUG", "PORT TENDER", "ANTI-POLLUTION",
        "LAW ENFORCEMENT", NULL, NULL, "MEDICAL", "SPECIAL CRAFT",     /* 50 to 59 */
    };
    if (type >= 20 && type <= 29) {
        return "WING IN GROUND";
    }
    if (type >= 30 && type <= 37) {
        return special[type - 30];
    }
    if (type >= 40 && type <= 49) {
        return "HIGH SPEED CRAFT";
    }
    if (type >= 50 && type <= 59) {
        return service[type - 50];
    }
    if (type >= 60 && type <= 69) {
        return "PASSENGER";
    }
    if (type >= 70 && type <= 79) {
        return "CARGO";
    }
    if (type >= 80 && type <= 89) {
        return "TANKER";
    }
    if (type >= 90 && type <= 99) {
        return "OTHER";
    }
    return NULL;
}

/* The navigational status in words. 11 and 12 are for regional use, as
 * ITU-R M.1371-5 has them: a tug towing astern, and pushing ahead or
 * towing alongside. */
static const char *status_words(long status)
{
    static const char *const words[] = {
        "UNDER WAY", "AT ANCHOR", "NOT UNDER COMMAND", "RESTRICTED", "CONSTRAINED", "MOORED",
        "AGROUND", "FISHING", "SAILING", NULL, NULL, "TOWING ASTERN", "PUSHING AHEAD", NULL,
        "AIS-SART", NULL,
    };
    return status >= 0 && status <= NO_STATUS ? words[status] : NULL;
}

/* The countries of the Maritime Identification Digits, the first three of
 * a ship's MMSI: ITU's table (ITU-R M.585, and ITU's list of MIDs), less
 * those of a few small territories. It is where the MMSI was assigned,
 * which is the flag a ship flies, as the ship's own radio says it: an
 * MMSI mistyped into a radio says another country, or none. */
static const struct {
    uint16_t first, last;
    const char *country;
} countries[] = {
    {201, 201, "ALBANIA"}, {202, 202, "ANDORRA"}, {203, 203, "AUSTRIA"},
    {204, 204, "PORTUGAL"}, {205, 205, "BELGIUM"}, {206, 206, "BELARUS"},
    {207, 207, "BULGARIA"}, {208, 208, "VATICAN"}, {209, 210, "CYPRUS"},
    {211, 211, "GERMANY"}, {212, 212, "CYPRUS"}, {213, 213, "GEORGIA"},
    {214, 214, "MOLDOVA"}, {215, 215, "MALTA"}, {216, 216, "ARMENIA"},
    {218, 218, "GERMANY"}, {219, 220, "DENMARK"}, {224, 225, "SPAIN"},
    {226, 228, "FRANCE"}, {229, 229, "MALTA"}, {230, 230, "FINLAND"},
    {231, 231, "FAROE IS"}, {232, 235, "UNITED KINGDOM"}, {236, 236, "GIBRALTAR"},
    {237, 237, "GREECE"}, {238, 238, "CROATIA"}, {239, 241, "GREECE"},
    {242, 242, "MOROCCO"}, {243, 243, "HUNGARY"}, {244, 246, "NETHERLANDS"},
    {247, 247, "ITALY"}, {248, 249, "MALTA"}, {250, 250, "IRELAND"},
    {251, 251, "ICELAND"}, {252, 252, "LIECHTENSTEIN"}, {253, 253, "LUXEMBOURG"},
    {254, 254, "MONACO"}, {255, 255, "PORTUGAL"}, {256, 256, "MALTA"},
    {257, 259, "NORWAY"}, {261, 261, "POLAND"}, {262, 262, "MONTENEGRO"},
    {263, 263, "PORTUGAL"}, {264, 264, "ROMANIA"}, {265, 266, "SWEDEN"},
    {267, 267, "SLOVAKIA"}, {268, 268, "SAN MARINO"}, {269, 269, "SWITZERLAND"},
    {270, 270, "CZECHIA"}, {271, 271, "TURKEY"}, {272, 272, "UKRAINE"},
    {273, 273, "RUSSIA"}, {274, 274, "NORTH MACEDONIA"}, {275, 275, "LATVIA"},
    {276, 276, "ESTONIA"}, {277, 277, "LITHUANIA"}, {278, 278, "SLOVENIA"},
    {279, 279, "SERBIA"},
    {301, 301, "ANGUILLA"}, {303, 303, "UNITED STATES"}, {304, 305, "ANTIGUA"},
    {307, 307, "ARUBA"}, {308, 309, "BAHAMAS"}, {310, 310, "BERMUDA"},
    {311, 311, "BAHAMAS"}, {312, 312, "BELIZE"}, {314, 314, "BARBADOS"},
    {316, 316, "CANADA"}, {319, 319, "CAYMAN IS"}, {321, 321, "COSTA RICA"},
    {323, 323, "CUBA"}, {325, 325, "DOMINICA"}, {327, 327, "DOMINICAN REP"},
    {329, 329, "GUADELOUPE"}, {330, 330, "GRENADA"}, {331, 331, "GREENLAND"},
    {332, 332, "GUATEMALA"}, {334, 334, "HONDURAS"}, {336, 336, "HAITI"},
    {338, 338, "UNITED STATES"}, {339, 339, "JAMAICA"}, {341, 341, "ST KITTS NEVIS"},
    {343, 343, "ST LUCIA"}, {345, 345, "MEXICO"}, {347, 347, "MARTINIQUE"},
    {348, 348, "MONTSERRAT"}, {350, 350, "NICARAGUA"}, {351, 357, "PANAMA"},
    {358, 358, "PUERTO RICO"}, {359, 359, "EL SALVADOR"}, {361, 361, "ST PIERRE"},
    {362, 362, "TRINIDAD TOBAGO"}, {364, 364, "TURKS CAICOS IS"},
    {366, 369, "UNITED STATES"}, {370, 374, "PANAMA"}, {375, 377, "ST VINCENT"},
    {378, 378, "BRIT VIRGIN IS"}, {379, 379, "US VIRGIN IS"},
    {401, 401, "AFGHANISTAN"}, {403, 403, "SAUDI ARABIA"}, {405, 405, "BANGLADESH"},
    {408, 408, "BAHRAIN"}, {410, 410, "BHUTAN"}, {412, 414, "CHINA"},
    {416, 416, "TAIWAN"}, {417, 417, "SRI LANKA"}, {419, 419, "INDIA"},
    {422, 422, "IRAN"}, {423, 423, "AZERBAIJAN"}, {425, 425, "IRAQ"},
    {428, 428, "ISRAEL"}, {431, 432, "JAPAN"}, {434, 434, "TURKMENISTAN"},
    {436, 436, "KAZAKHSTAN"}, {437, 437, "UZBEKISTAN"}, {438, 438, "JORDAN"},
    {440, 441, "SOUTH KOREA"}, {443, 443, "PALESTINE"}, {445, 445, "NORTH KOREA"},
    {447, 447, "KUWAIT"}, {450, 450, "LEBANON"}, {451, 451, "KYRGYZSTAN"},
    {453, 453, "MACAO"}, {455, 455, "MALDIVES"}, {457, 457, "MONGOLIA"},
    {459, 459, "NEPAL"}, {461, 461, "OMAN"}, {463, 463, "PAKISTAN"},
    {466, 466, "QATAR"}, {468, 468, "SYRIA"}, {470, 471, "UAE"},
    {472, 472, "TAJIKISTAN"}, {473, 473, "YEMEN"}, {475, 475, "YEMEN"},
    {477, 477, "HONG KONG"}, {478, 478, "BOSNIA"},
    {503, 503, "AUSTRALIA"}, {506, 506, "MYANMAR"}, {508, 508, "BRUNEI"},
    {510, 510, "MICRONESIA"}, {511, 511, "PALAU"}, {512, 512, "NEW ZEALAND"},
    {514, 515, "CAMBODIA"}, {518, 518, "COOK IS"}, {520, 520, "FIJI"},
    {525, 525, "INDONESIA"}, {529, 529, "KIRIBATI"}, {531, 531, "LAOS"},
    {533, 533, "MALAYSIA"}, {536, 536, "N MARIANA IS"}, {538, 538, "MARSHALL IS"},
    {540, 540, "NEW CALEDONIA"}, {542, 542, "NIUE"}, {544, 544, "NAURU"},
    {546, 546, "FRENCH POLYNESIA"}, {548, 548, "PHILIPPINES"},
    {553, 553, "PAPUA NEW GUINEA"}, {555, 555, "PITCAIRN"}, {557, 557, "SOLOMON IS"},
    {559, 559, "AMERICAN SAMOA"}, {561, 561, "SAMOA"}, {563, 566, "SINGAPORE"},
    {567, 567, "THAILAND"}, {570, 570, "TONGA"}, {572, 572, "TUVALU"},
    {574, 574, "VIETNAM"}, {576, 577, "VANUATU"}, {578, 578, "WALLIS FUTUNA"},
    {601, 601, "SOUTH AFRICA"}, {603, 603, "ANGOLA"}, {605, 605, "ALGERIA"},
    {609, 609, "BURUNDI"}, {610, 610, "BENIN"}, {611, 611, "BOTSWANA"},
    {613, 613, "CAMEROON"}, {615, 615, "CONGO"}, {617, 617, "CAPE VERDE"},
    {619, 619, "IVORY COAST"}, {621, 621, "DJIBOUTI"}, {622, 622, "EGYPT"},
    {624, 624, "ETHIOPIA"}, {625, 625, "ERITREA"}, {626, 626, "GABON"},
    {627, 627, "GHANA"}, {629, 629, "GAMBIA"}, {630, 630, "GUINEA-BISSAU"},
    {631, 631, "EQUATORIAL GUINEA"}, {632, 632, "GUINEA"}, {633, 633, "BURKINA FASO"},
    {634, 634, "KENYA"}, {636, 637, "LIBERIA"}, {638, 638, "SOUTH SUDAN"},
    {642, 642, "LIBYA"}, {644, 644, "LESOTHO"}, {645, 645, "MAURITIUS"},
    {647, 647, "MADAGASCAR"}, {649, 649, "MALI"}, {650, 650, "MOZAMBIQUE"},
    {654, 654, "MAURITANIA"}, {655, 655, "MALAWI"}, {656, 656, "NIGER"},
    {657, 657, "NIGERIA"}, {659, 659, "NAMIBIA"}, {660, 660, "REUNION"},
    {661, 661, "RWANDA"}, {662, 662, "SUDAN"}, {663, 663, "SENEGAL"},
    {664, 664, "SEYCHELLES"}, {665, 665, "ST HELENA"}, {666, 666, "SOMALIA"},
    {667, 667, "SIERRA LEONE"}, {668, 668, "SAO TOME"}, {669, 669, "ESWATINI"},
    {670, 670, "CHAD"}, {671, 671, "TOGO"}, {672, 672, "TUNISIA"},
    {674, 674, "TANZANIA"}, {675, 675, "UGANDA"}, {676, 676, "DR CONGO"},
    {677, 677, "TANZANIA"}, {678, 678, "ZAMBIA"}, {679, 679, "ZIMBABWE"},
    {701, 701, "ARGENTINA"}, {710, 710, "BRAZIL"}, {720, 720, "BOLIVIA"},
    {725, 725, "CHILE"}, {730, 730, "COLOMBIA"}, {735, 735, "ECUADOR"},
    {740, 740, "FALKLAND IS"}, {745, 745, "FRENCH GUIANA"}, {750, 750, "GUYANA"},
    {755, 755, "PARAGUAY"}, {760, 760, "PERU"}, {765, 765, "SURINAME"},
    {770, 770, "URUGUAY"}, {775, 775, "VENEZUELA"},
};

/* The country of a ship's MMSI, of nine digits the first 2 to 7. NULL if
 * it is not a ship's, or its MID is not in the table. */
static const char *country_of(uint32_t mmsi)
{
    uint32_t first = mmsi / 100000000u;
    if (first < 2 || first > 7) {
        return NULL;
    }
    uint32_t mid = mmsi / 1000000u;
    for (size_t index = 0; index < sizeof(countries) / sizeof(countries[0]); index++) {
        if (mid >= countries[index].first && mid <= countries[index].last) {
            return countries[index].country;
        }
    }
    return NULL;
}

/* How long a text the card keeps is, less the spaces after it. */
static int kept_len(const char *text, size_t width)
{
    while (width > 0 && text[width - 1] == ' ') {
        width--;
    }
    return (int)width;
}

void harbor_card(const nesp_harbor_ship_t *ship, double latitude, double longitude,
                 uint8_t line, bool timed, uint32_t now, uint8_t *out)
{
    char text[64];
    char part[32];
    const char *words;
    int len;
    switch (line) {
    case 0:                         /* its name, or what it has for one */
        if ((len = kept_len(ship->name, sizeof(ship->name))) > 0) {
            snprintf(text, sizeof(text), "%.*s", len, ship->name);
        } else if ((len = kept_len(ship->call, sizeof(ship->call))) > 0) {
            snprintf(text, sizeof(text), "%.*s", len, ship->call);
        } else {
            snprintf(text, sizeof(text), "MMSI %09lu", (unsigned long)ship->mmsi);
        }
        break;
    case 1:
        snprintf(text, sizeof(text), "MMSI %09lu", (unsigned long)ship->mmsi);
        break;
    case 2:
        if (ship->imo != 0) {
            snprintf(text, sizeof(text), "IMO %lu", (unsigned long)ship->imo);
        } else {
            snprintf(text, sizeof(text), "IMO -");
        }
        break;
    case 3:
        len = kept_len(ship->call, sizeof(ship->call));
        snprintf(text, sizeof(text), "CALL %.*s", len > 0 ? len : 1, len > 0 ? ship->call : "-");
        break;
    case 4:
        words = country_of(ship->mmsi);
        snprintf(text, sizeof(text), "FLAG %s", words != NULL ? words : "-");
        break;
    case 5:
        words = type_words(ship->type);
        snprintf(text, sizeof(text), "TYPE %s", words != NULL ? words : "-");
        break;
    case 6:
        words = status_words(ship->status);
        snprintf(text, sizeof(text), "STATUS %s", words != NULL ? words : "-");
        break;
    case 7:
        len = kept_len(ship->dest, sizeof(ship->dest));
        snprintf(text, sizeof(text), "DEST %.*s", len > 0 ? len : 1, len > 0 ? ship->dest : "-");
        break;
    case 8:                         /* "00-00 00:00" is AIS's none */
        len = kept_len(ship->eta, sizeof(ship->eta));
        if (len == 0 || (len == 11 && memcmp(ship->eta, "00-00 00:00", 11) == 0)) {
            snprintf(text, sizeof(text), "ETA -");
        } else {
            snprintf(text, sizeof(text), "ETA %.*s", len, ship->eta);
        }
        break;
    case 9:
        if (ship->length == 0 && ship->beam == 0) {
            snprintf(text, sizeof(text), "SIZE -");
            break;
        }
        if (ship->length != 0) {
            snprintf(part, sizeof(part), "%u", (unsigned)ship->length);
        } else {
            snprintf(part, sizeof(part), "-");
        }
        if (ship->beam != 0) {
            snprintf(text, sizeof(text), "SIZE %s X %u M", part, (unsigned)ship->beam);
        } else {
            snprintf(text, sizeof(text), "SIZE %s X - M", part);
        }
        break;
    case 10:
        if (ship->draught != 0) {
            snprintf(text, sizeof(text), "DRAFT %u.%u M", (unsigned)ship->draught / 10,
                     (unsigned)ship->draught % 10);
        } else {
            snprintf(text, sizeof(text), "DRAFT -");
        }
        break;
    case 11:
        if (ship->sog < NO_SOG_TENTHS) {
            snprintf(part, sizeof(part), "SOG %u.%u KN", (unsigned)ship->sog / 10,
                     (unsigned)ship->sog % 10);
        } else {
            snprintf(part, sizeof(part), "SOG -");
        }
        if (ship->cog < NO_COG_TENTHS) {
            snprintf(text, sizeof(text), "%s COG %u", part, ((unsigned)ship->cog + 5) / 10 % 360);
        } else {
            snprintf(text, sizeof(text), "%s COG -", part);
        }
        break;
    case 12:
        if (ship->heading < 360) {
            snprintf(text, sizeof(text), "HEADING %u", (unsigned)ship->heading);
        } else {
            snprintf(text, sizeof(text), "HEADING -");
        }
        break;
    case 13:
        snprintf(text, sizeof(text), "LAT %.4f %c", fabs(latitude), latitude < 0 ? 'S' : 'N');
        break;
    case 14:
        snprintf(text, sizeof(text), "LON %.4f %c", fabs(longitude), longitude < 0 ? 'W' : 'E');
        break;
    case 15: {                      /* how long ago AISHub heard it, by the hal's clock */
        int64_t ago = (int64_t)now - (int64_t)ship->heard;
        if (!timed || ship->heard == 0) {
            snprintf(text, sizeof(text), "HEARD -");
        } else if (ago < 60) {
            snprintf(text, sizeof(text), "HEARD JUST NOW");
        } else if (ago < 3600) {
            snprintf(text, sizeof(text), "HEARD %ld MIN AGO", (long)(ago / 60));
        } else {
            snprintf(text, sizeof(text), "HEARD %ld H AGO", (long)(ago / 3600));
        }
        break;
    }
    default:
        text[0] = 0;
        break;
    }
    scope_value_t value = {text, strlen(text), false};
    scope_put_text(&value, out, WIDTH);
}

/* --- the map ------------------------------------------------------------------- */

#define TILES "https://tile.openstreetmap.org/"
#define CELLS (NESP_HARBOR_MAP_ROWS * 2)   /* across the scope, and down it */
#define SAMPLES 3                   /* across a cell, and down it */
#define LINE (CELLS * SAMPLES)
#define TILE 256                    /* pixels across a tile, and down it */
#define ZOOM_MAX 19                 /* OpenStreetMap's deepest */
#define MERCATOR_MAX 85.0511287798  /* the latitude where the tiles end */
/* OpenStreetMap's water (#aad3df), and how far from it a colour may be
 * in each of red, green and blue and still be water. */
#define WATER_RED 170
#define WATER_GREEN 211
#define WATER_BLUE 223
#define NEAR 16
/* And the lines OpenStreetMap draws over the sea, of boundaries and ferries:
 * blue-grey and purple, blue the most by this much. Its land is never so. */
#define BLUER 10

/* The decoder's memory and what it has counted, asked of the hal the first
 * time a map is made. */
struct harbor_map_work {
    nesp_png_t png;
    uint8_t water[CELLS][CELLS];    /* samples of each cell that were water */
    uint8_t seen[CELLS][CELLS];     /* and samples of it */
};

/* A sample's point on the scope, across it or down it: three in each cell,
 * at a point from its sides and in its middle. */
static double sample_point(unsigned sample)
{
    return 4.0 * (sample / SAMPLES) + 1.0 + sample % SAMPLES;
}

/* Where a longitude and a latitude are among all the tiles of a zoom, in
 * pixels from the west and from the north. */
static double world_x(double longitude, unsigned zoom)
{
    return (longitude + 180.0) / 360.0 * TILE * (double)(1ul << zoom);
}

static double world_y(double latitude, unsigned zoom)
{
    double radians = scope_radians(between(latitude, -MERCATOR_MAX, MERCATOR_MAX));
    return (1.0 - asinh(tan(radians)) / PI) / 2.0 * TILE * (double)(1ul << zoom);
}

/* The tile a pixel is on, west and north of 0 too. */
static int32_t tile_of(int32_t pixel)
{
    return pixel >= 0 ? pixel / TILE : -((TILE - 1 - pixel) / TILE);
}

/* A map of the place chosen, at its range: the zoom, the samples and the
 * tiles. False if none can be made, near a pole or with no memory. */
bool harbor_map_plan(nesp_radar_t *radar)
{
    if (radar->map_work == NULL) {
        /* Kept: the fetch's task may be in the middle of it whenever this
         * one would give it back. */
        radar->map_work = radar->hal->alloc(radar->hal->ctx, sizeof(*radar->map_work));
        if (radar->map_work == NULL) {
            return false;
        }
        memset(radar->map_work, 0, sizeof(*radar->map_work));
    }
    double latitude = radar->map_latitude / 1e6, longitude = radar->map_longitude / 1e6;
    double across = cos(scope_radians(latitude));
    if (across <= 0.01) {
        return false;
    }
    double north = radar->map_range / EDGE / 60.0;  /* degrees in a point */
    double east = north / across;
    unsigned zoom = ZOOM_MAX + 1;
    int32_t span_x, span_y;
    do {
        zoom--;
        span_x = tile_of((int32_t)floor(world_x(longitude + east * (LAST + 1 - MIDDLE), zoom)))
                 - tile_of((int32_t)floor(world_x(longitude - east * MIDDLE, zoom)));
        span_y = tile_of((int32_t)floor(world_y(latitude - north * (LAST + 1 - MIDDLE), zoom)))
                 - tile_of((int32_t)floor(world_y(latitude + north * MIDDLE, zoom)));
    } while (zoom > 0 && (span_x > 1 || span_y > 1));
    if (span_x > 1) {
        return false;               /* wider than the world */
    }
    int32_t bottom = (int32_t)(TILE << zoom) - 1;
    for (unsigned sample = 0; sample < LINE; sample++) {
        double point = sample_point(sample);
        radar->map_x[sample] = (int32_t)floor(world_x(longitude + east * (point - MIDDLE), zoom));
        int32_t y = (int32_t)floor(world_y(latitude - north * (point - MIDDLE), zoom));
        radar->map_y[sample] = y < 0 ? 0 : y > bottom ? bottom : y;
    }
    radar->map_zoom = (uint8_t)zoom;
    radar->map_tiles = 0;
    for (int32_t y = tile_of(radar->map_y[0]); y <= tile_of(radar->map_y[LINE - 1]); y++) {
        for (int32_t x = tile_of(radar->map_x[0]); x <= tile_of(radar->map_x[LINE - 1]); x++) {
            radar->map_tile_at[radar->map_tiles][0] = x;
            radar->map_tile_at[radar->map_tiles][1] = y;
            radar->map_tiles++;
        }
    }
    return true;
}

/* The address of the tile being fetched. One west of the first or east of
 * the last is the world's other side's. */
void harbor_map_url(const nesp_radar_t *radar, char *url, size_t size)
{
    long count = 1l << radar->map_zoom;
    long x = ((radar->map_tile_at[radar->map_tile][0] % count) + count) % count;
    snprintf(url, size, TILES "%u/%ld/%ld.png", (unsigned)radar->map_zoom, x,
             (long)radar->map_tile_at[radar->map_tile][1]);
}

static bool is_water(const uint8_t *rgb)
{
    if (abs(rgb[0] - WATER_RED) <= NEAR && abs(rgb[1] - WATER_GREEN) <= NEAR
        && abs(rgb[2] - WATER_BLUE) <= NEAR) {
        return true;
    }
    return rgb[2] >= rgb[0] + BLUER && rgb[2] >= rgb[1] + BLUER;
}

/* A row of a tile: its samples, counted into their cells. */
static void sampled(void *ctx, uint32_t y, const uint8_t *pixels)
{
    nesp_radar_t *radar = ctx;
    struct harbor_map_work *work = radar->map_work;
    uint8_t tile = radar->map_tile < radar->map_tiles ? radar->map_tile : 0;
    int32_t west = radar->map_tile_at[tile][0] * TILE;
    int32_t at = radar->map_tile_at[tile][1] * TILE + (int32_t)y;
    for (unsigned down = 0; down < LINE && radar->map_y[down] <= at; down++) {
        if (radar->map_y[down] != at) {
            continue;
        }
        for (unsigned across = 0; across < LINE; across++) {
            int32_t x = radar->map_x[across] - west;
            if (x < 0 || x >= (int32_t)work->png.width) {
                continue;
            }
            uint8_t rgb[3];
            nesp_png_colour(&work->png, pixels, (uint32_t)x, rgb);
            unsigned row = down / SAMPLES, column = across / SAMPLES;
            work->seen[row][column]++;
            work->water[row][column] = (uint8_t)(work->water[row][column] + is_water(rgb));
        }
    }
}

/* More of a tile. False: it is not one this reads, and no more is wanted. */
bool harbor_map_body(nesp_radar_t *radar, uint32_t id, const uint8_t *data, size_t len)
{
    struct harbor_map_work *work = radar->map_work;
    if (radar->map_parsing != id) {     /* the first of a new tile */
        radar->map_parsing = id;
        nesp_png_init(&work->png, sampled, radar);
        if (radar->map_tile == 0) {
            memset(work->water, 0, sizeof(work->water));
            memset(work->seen, 0, sizeof(work->seen));
        }
    }
    return nesp_png_feed(&work->png, data, len);
}

/* The tile has come whole, and was a tile. */
bool harbor_map_whole(const nesp_radar_t *radar, uint32_t id)
{
    const nesp_png_t *png = &radar->map_work->png;
    return radar->map_parsing == id && nesp_png_whole(png) && png->width == TILE
           && png->height == TILE;
}

/* Every tile has come: the codes of the map, four cells to a tile of the
 * console's, a set bit land. A cell no sample was on is water. */
void harbor_map_codes(nesp_radar_t *radar)
{
    const struct harbor_map_work *work = radar->map_work;
    memset(radar->map_codes, 0, sizeof(radar->map_codes));
    for (unsigned row = 0; row < CELLS; row++) {
        for (unsigned column = 0; column < CELLS; column++) {
            if (work->water[row][column] * 2 > work->seen[row][column]
                || work->seen[row][column] == 0) {
                continue;
            }
            unsigned tile = column / 2;
            unsigned bit = 1u << ((row % 2) * 2 + column % 2);
            radar->map_codes[row / 2][tile / 2] |= (uint8_t)(tile % 2 == 0 ? bit << 4 : bit);
        }
    }
}

void nesp_harbor_service(nesp_service_t *service, nesp_harbor_t *harbor, const nesp_hal_t *hal)
{
    scope_service(service, harbor, hal, true);
}
