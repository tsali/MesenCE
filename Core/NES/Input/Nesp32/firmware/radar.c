/* RADAR: the aircraft round an airport, for NES Radar's scope
 * (docs/PROTOCOL.md).
 *
 * The console names an airport by its four letters. The nesp32 asks
 * Wikidata where that is, once, and keeps the answer over power-off; then asks adsb.fi every few seconds for the
 * aircraft near it, and keeps the nearest eight as a scene: for each, nine bytes of where it is and how
 * it moves, and twenty-five of its names, both as nes-radar's ROM has them.
 * The console asks for the scene, and is sent it only if it is not the one
 * it has; the names it asks for a slot at a time, when they have changed.
 *
 * AIRPORT is both "this airport" and "how does it stand". The console gives
 * each choice an id and may send the same request as often as it likes: an
 * id or letters that are new choose the airport, and the same again only
 * answers. So a lost request or reply needs no handling of its own.
 *
 * Two tasks meet here. The link's task runs handle() and tick(), and the
 * scene the console is told of is its alone. The fetch's task runs body()
 * and done(), reads the reply as it comes and keeps the place, or the
 * nearest aircraft, in it; when the reply has ended it says so (`landed`, under the hal's lock),
 * and touches nothing more until the link's task has taken them and begun
 * another fetch. `generation` is which fetch is wanted: a new airport takes
 * a new one, and whatever an older fetch brings is dropped.
 *
 * The clock is tick()'s, so the times are too: a fetch every
 * NESP_RADAR_POLL_MS while the console goes on asking, and a scene that is
 * stale when nothing good has come for NESP_RADAR_STALE_MS.
 *
 * HARBOR (harbor.c) is this service again with ships in place of aircraft:
 * `ships` is set, the fetch is of AISHub, once a minute, with a key the
 * board keeps, and the reply's vessels are read by harbor.c. Its map's
 * tiles are fetched in the same way, one at a time between AISHub's
 * fetches, and read by harbor.c too: `mapping` says which a fetch is.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nesp_services.h"
#include "scope.h"

#define AGENT "nesp32-radar/0.1 (NES Radar on a nesp32; +https://github.com/Armitage64/nesp32)"
#define SOURCE "https://opendata.adsb.fi/api/v3"
/* Where a US zip code is, for HARBOR, from the site WEATHER asks
 * (weather.c): {"places": [{"latitude": "34.0901", "longitude":
 * "-118.4065", ...}], ...}, and a 404 for one there is none of. */
#define ZIP_WHERE "https://api.zippopotam.us/us/"
#define NO_SUCH_ZIP "THE SITE SAID 404"
#define HARBOR_AGENT "nesp32-harbor/0.1 (NES Radar, for ships, on a nesp32; +https://github.com/Armitage64/nesp32)"
/* OpenStreetMap's tiles are asked for as NES Harbor's, the ROM that shows
 * the map. */
#define MAP_AGENT "nesp32-harbor/0.1 (+https://github.com/tsali/nes-harbor)"
/* Where an airport is: Wikidata's item with these letters as its ICAO code
 * (P239), and that item's coordinates (P625). The query is
 *   SELECT ?p WHERE { ?a wdt:P239 "KSBA"; wdt:P625 ?p } LIMIT 1
 * and the answer has the place as "Point(longitude latitude)". */
#define WHERE "https://query.wikidata.org/sparql?format=json&query=" \
              "SELECT%20%3Fp%20WHERE%20%7B%20%3Fa%20wdt%3AP239%20%22"
#define WHERE_END "%22%3B%20wdt%3AP625%20%3Fp%20%7D%20LIMIT%201"
static const char point[] = "Point(";

/* The scope: nine nautical miles to its edge, which is EDGE (71) points
 * from its MIDDLE at (72, 72). An aircraft slower than a landing one is on the
 * ground, whatever it says. */
#define RANGE_NM 9.0
#define ASKED_NM 10
#define SLOWEST_KT 40.0

#define LANDED_WELL 1
#define LANDED_BADLY 2
#define LANDED_NONE 3               /* HARBOR: no place has that zip code */

/* What is known of where the airport is. */
#define PLACE_UNKNOWN 0             /* not asked yet, or being asked */
#define PLACE_KNOWN 1
#define PLACE_NONE 2                /* no airport has those letters */
#define PLACE_FAILED 3              /* the asking failed */

static void lock(nesp_radar_t *radar)
{
    radar->hal->lock(radar->hal->ctx);
}

static void unlock(nesp_radar_t *radar)
{
    radar->hal->unlock(radar->hal->ctx);
}

/* --- the fetch's task: one aircraft ------------------------------------------ */

static bool is_space(char letter)
{
    return letter == ' ' || (letter >= '\t' && letter <= '\r');
}

/* Past a string that begins at `at`, a quote. Returns where the character
 * after its closing quote is. */
static size_t past_string(const char *item, size_t len, size_t at)
{
    at++;
    while (at < len && item[at] != '"') {
        at += item[at] == '\\' ? 2 : 1;
    }
    return at < len ? at + 1 : len;
}

/* What an aircraft has by this name, at its top: what is inside a list or
 * another object of its own is passed over. False if it has none, or null. */
bool scope_member(const char *item, size_t len, const char *name, scope_value_t *out)
{
    size_t name_len = strlen(name);
    size_t at = 1;                  /* past its brace */
    while (at < len) {
        while (at < len && item[at] != '"') {
            if (item[at] == '}') {
                return false;
            }
            at++;
        }
        size_t key = at + 1;
        size_t end = past_string(item, len, at);
        if (end >= len) {
            return false;
        }
        bool wanted = end - 1 - key == name_len && memcmp(item + key, name, name_len) == 0;
        at = end;
        while (at < len && (is_space(item[at]) || item[at] == ':')) {
            at++;
        }
        if (at >= len) {
            return false;
        }
        size_t begins = at;
        bool string = item[at] == '"';
        if (string) {
            at = past_string(item, len, at);
        } else if (item[at] == '{' || item[at] == '[') {
            unsigned depth = 0;
            do {
                if (item[at] == '"') {
                    at = past_string(item, len, at);
                    continue;
                }
                if (item[at] == '{' || item[at] == '[') {
                    depth++;
                } else if (item[at] == '}' || item[at] == ']') {
                    depth--;
                }
                at++;
            } while (at < len && depth > 0);
            wanted = false;         /* nothing wanted is a list */
        } else {
            while (at < len && item[at] != ',' && item[at] != '}' && !is_space(item[at])) {
                at++;
            }
        }
        if (wanted) {
            if (string) {
                out->at = item + begins + 1;
                out->len = at - begins - 2;
            } else {
                out->at = item + begins;
                out->len = at - begins;
            }
            out->string = string;
            return string || !(out->len == 4 && memcmp(out->at, "null", 4) == 0);
        }
    }
    return false;
}

/* A number, or a string that is one. */
bool scope_number(const char *item, size_t len, const char *name, double *out)
{
    scope_value_t value;
    char text[32];
    if (!scope_member(item, len, name, &value)) {
        return false;
    }
    while (value.len > 0 && is_space(value.at[0])) {
        value.at++;
        value.len--;
    }
    while (value.len > 0 && is_space(value.at[value.len - 1])) {
        value.len--;
    }
    if (value.len == 0 || value.len >= sizeof(text)) {
        return false;
    }
    memcpy(text, value.at, value.len);
    text[value.len] = 0;
    if (strpbrk(text, "xXnN") != NULL) {    /* hexadecimal, or not a number at all */
        return false;
    }
    char *end;
    double result = strtod(text, &end);
    if (end != text + value.len || !isfinite(result)) {
        return false;
    }
    *out = result;
    return true;
}

static bool is_text(const char *item, size_t len, const char *name, const char *text)
{
    scope_value_t value;
    size_t text_len = strlen(text);
    if (!scope_member(item, len, name, &value) || !value.string || value.len != text_len) {
        return false;
    }
    for (size_t index = 0; index < text_len; index++) {
        char letter = value.at[index];
        if (letter >= 'A' && letter <= 'Z') {
            letter = (char)(letter - 'A' + 'a');
        }
        if (letter != text[index]) {
            return false;
        }
    }
    return true;
}

static int hex_digit(char letter)
{
    if (letter >= '0' && letter <= '9') {
        return letter - '0';
    }
    letter = (char)(letter | 0x20);
    return letter >= 'a' && letter <= 'f' ? letter - 'a' + 10 : -1;
}

/* A character as the scope shows it, or a ship's card: capitals, and a
 * hyphen on the scope, or a space on a card, for one it has not. */
static char shown_as(char letter, bool card)
{
    if (letter >= 'a' && letter <= 'z') {
        letter = (char)(letter - 'a' + 'A');
    }
    bool shown = (letter >= 'A' && letter <= 'Z') || (letter >= '0' && letter <= '9')
                 || letter == ' ' || letter == '-';
    if (card) {
        shown = shown || letter == '+' || letter == '.' || letter == '/' || letter == ':';
    }
    return shown ? letter : card ? ' ' : '-';
}

/* A value as a name on the scope, or as a line of a ship's card: no spaces
 * round it, capitals, cut to the width and padded with spaces. A character
 * that is not ASCII is one character. On the scope that has none for it is
 * a hyphen, and on a card a space. */
static void put_value(const scope_value_t *value, uint8_t *out, size_t width, bool card)
{
    char text[64];
    size_t count = 0;
    size_t at = 0;
    while (at < value->len && count < sizeof(text)) {
        unsigned char letter = (unsigned char)value->at[at++];
        if (value->string && letter == '\\' && at < value->len) {
            char escape = value->at[at++];
            switch (escape) {
            case 'n': letter = '\n'; break;
            case 't': letter = '\t'; break;
            case 'r': letter = '\r'; break;
            case 'f': letter = '\f'; break;
            case 'b': letter = 0x7F; break;
            case 'u': {
                unsigned code = 0;
                for (int digit = 0; digit < 4 && at < value->len; digit++) {
                    int hex = hex_digit(value->at[at]);
                    if (hex < 0) {
                        break;
                    }
                    code = code << 4 | (unsigned)hex;
                    at++;
                }
                if (code >= 0xDC00 && code <= 0xDFFF) {
                    continue;       /* the second half of a character already counted */
                }
                letter = code < 0x80 ? (unsigned char)code : 0x7F;
                break;
            }
            default: letter = (unsigned char)escape; break;
            }
        } else if (letter >= 0x80) {
            if (letter < 0xC0) {
                continue;           /* the rest of a character already counted */
            }
            letter = 0x7F;
        }
        text[count++] = (char)letter;
    }
    if (card) {                     /* a card's spaces are those it has no character for too */
        for (size_t index = 0; index < count; index++) {
            text[index] = shown_as(text[index], true);
        }
    }
    size_t first = 0;
    while (first < count && is_space(text[first])) {
        first++;
    }
    while (count > first && is_space(text[count - 1])) {
        count--;
    }
    for (size_t index = 0; index < width; index++) {
        out[index] = (uint8_t)(first + index < count ? shown_as(text[first + index], card) : ' ');
    }
}

void scope_put_name(const scope_value_t *value, uint8_t *out, size_t width)
{
    put_value(value, out, width, false);
}

void scope_put_text(const scope_value_t *value, uint8_t *out, size_t width)
{
    put_value(value, out, width, true);
}

/* The first of these names that an aircraft has anything by, as a name on
 * the scope; four hyphens if it has none. */
static void put_first(const char *item, size_t len, const char *const *names, size_t count,
                      uint8_t *out, size_t width)
{
    static const scope_value_t none = {"----", 4, true};
    scope_value_t value;
    for (size_t index = 0; index < count; index++) {
        if (scope_member(item, len, names[index], &value) && value.len > 0) {
            scope_put_name(&value, out, width);
            return;
        }
    }
    scope_put_name(&none, out, width);
}

static void put_one(const char *item, size_t len, const char *name, uint8_t *out, size_t width)
{
    static const scope_value_t none = {"", 0, true};
    scope_value_t value;
    scope_put_name(scope_member(item, len, name, &value) ? &value : &none, out, width);
}

double scope_radians(double degrees)
{
    return degrees * (PI / 180.0);
}

/* To the nearest whole number, a half to the even one, and kept between
 * two. */
long scope_rounded(double value, long least, long most)
{
    double whole = nearbyint(value);
    if (whole < (double)least) {
        return least;
    }
    return whole > (double)most ? most : (long)whole;
}

/* How far a place is from the airport, round the earth, in nautical miles,
 * and which way, in degrees from north. */
static double how_far(const nesp_radar_t *radar, double latitude, double longitude,
                      double *bearing)
{
    double from = scope_radians(radar->here[0]);
    double to = scope_radians(latitude);
    double north = scope_radians(latitude - radar->here[0]);
    double east = scope_radians(longitude - radar->here[1]);
    double half = sin(north / 2) * sin(north / 2)
                  + cos(from) * cos(to) * sin(east / 2) * sin(east / 2);
    half = half < 0.0 ? 0.0 : half > 1.0 ? 1.0 : half;
    *bearing = fmod(atan2(sin(east) * cos(to),
                          cos(from) * sin(to) - sin(from) * cos(to) * cos(east))
                    * (180.0 / PI) + 360.0, 360.0);
    return EARTH_NM * 2 * atan2(sqrt(half), sqrt(1 - half));
}

/* x and y on the scope, whose edge is range_nm from its middle. */
static void put_xy(uint8_t *motion, double distance, double bearing, double range_nm)
{
    double out = distance * (EDGE / range_nm);
    out = out > EDGE ? EDGE : out;
    motion[1] = (uint8_t)scope_rounded(MIDDLE + nearbyint(sin(scope_radians(bearing)) * out), 0, LAST);
    motion[2] = (uint8_t)scope_rounded(MIDDLE - nearbyint(cos(scope_radians(bearing)) * out), 0, LAST);
}

/* A place on the earth, put on the scope if it is within range_nm of the
 * airport and among the nearest so far: its target, in the order of their
 * distance, with x, y and distance set and the rest of its motion zeros.
 * NULL if it is not. The scope's edge is range_nm from its middle. */
nesp_radar_target_t *scope_spot(nesp_radar_t *radar, double latitude, double longitude,
                                double range_nm)
{
    double bearing;
    double distance = how_far(radar, latitude, longitude, &bearing);
    if (distance > range_nm) {
        return NULL;
    }
    radar->total++;
    size_t place = radar->near_count;
    while (place > 0 && distance < radar->near[place - 1].distance) {
        place--;
    }
    if (place >= TARGETS) {
        return NULL;
    }
    size_t moved = (radar->near_count < TARGETS ? radar->near_count : TARGETS - 1) - place;
    memmove(&radar->near[place + 1], &radar->near[place], moved * sizeof(radar->near[0]));
    if (radar->near_count < TARGETS) {
        radar->near_count++;
    }
    nesp_radar_target_t *target = &radar->near[place];
    target->distance = distance;
    target->latitude = latitude;
    target->longitude = longitude;
    memset(&target->ship, 0, sizeof(target->ship));
    uint8_t *motion = target->motion;
    memset(motion, 0, MOTION);
    put_xy(motion, distance, bearing, range_nm);
    motion[8] = (uint8_t)scope_rounded(distance * 10, 0, 99);
    return target;
}

/* One aircraft of the reply, whole. It is kept if it is flying, within
 * range, and among the nearest so far. */
static void aircraft(nesp_radar_t *radar, const char *item, size_t len)
{
    static const char *const calls[] = {"flight", "r", "hex"};
    static const char *const types[] = {"t"};
    double speed, latitude, longitude, value;
    if (is_text(item, len, "alt_baro", "ground")) {
        return;
    }
    bool has_speed = scope_number(item, len, "gs", &speed);
    if (has_speed && speed < SLOWEST_KT) {
        return;
    }
    if (!scope_number(item, len, "lat", &latitude) || !scope_number(item, len, "lon", &longitude)) {
        return;
    }
    nesp_radar_target_t *target = scope_spot(radar, latitude, longitude, RANGE_NM);
    if (target == NULL) {
        return;
    }
    uint8_t *motion = target->motion;
    if (scope_number(item, len, "track", &value) || scope_number(item, len, "true_heading", &value)) {
        motion[3] = (uint8_t)((unsigned long)scope_rounded(value * 256 / 360, -65536, 65536) & 0xFF);
    } else {
        motion[0] |= NO_TRACK;
    }
    if (scope_number(item, len, "alt_baro", &value)) {
        long hundreds = scope_rounded(value / 100, 0, 65535);
        motion[4] = (uint8_t)hundreds;
        motion[5] = (uint8_t)(hundreds >> 8);
    } else {
        motion[0] |= NO_ALTITUDE;
    }
    if (has_speed) {
        motion[6] = (uint8_t)scope_rounded(speed, 0, 255);
    } else {
        motion[0] |= NO_SPEED;
    }
    if (scope_number(item, len, "baro_rate", &value) || scope_number(item, len, "geom_rate", &value)) {
        motion[7] = (uint8_t)(scope_rounded(value / 100, -99, 99) & 0xFF);
    } else {
        motion[7] = NO_RATE;
    }

    uint8_t *names = target->names;
    put_first(item, len, calls, sizeof(calls) / sizeof(calls[0]), names + 1, 8);
    put_first(item, len, types, sizeof(types) / sizeof(types[0]), names + 9, 4);
    put_one(item, len, "r", names + 13, 6);
    put_one(item, len, "squawk", names + 19, 4);
    put_one(item, len, "category", names + 23, 2);
}

/* --- the fetch's task: the reply ---------------------------------------------- */

/* The reply is an object, and the aircraft are the objects in its list
 * named "ac". It is read a character at a time and never held whole: only
 * the aircraft being read is, and one too long for that is passed over.
 *
 * HARBOR's, AISHub's, is a list: a header object, then a list whose
 * objects are the ships. The header is read as a ship is. */
static void take(nesp_radar_t *radar, const uint8_t *data, size_t len)
{
    bool vessels = radar->ships && !radar->lookup;   /* AISHub's reply */
    bool zip = radar->ships && radar->lookup && radar->kind == NESP_PLACE_ZIP;  /* Zippopotam's */
    for (size_t index = 0; index < len; index++) {
        char letter = (char)data[index];
        if (radar->lookup && !zip && !radar->spotted) {
            /* The place is the first thing written as a point. */
            if (radar->matched < sizeof(point) - 1) {
                radar->matched = letter == point[radar->matched] ? (uint8_t)(radar->matched + 1)
                                                                 : (uint8_t)(letter == point[0]);
            } else if (letter == ')') {
                radar->spot[radar->spot_len] = 0;
                radar->spotted = true;
            } else if (radar->spot_len < NESP_RADAR_SPOT - 1) {
                radar->spot[radar->spot_len++] = letter;
            }
        }
        if (radar->taking) {
            if (radar->item_len < NESP_RADAR_ITEM) {
                radar->item[radar->item_len++] = letter;
            } else {
                radar->long_item = true;
            }
        }
        if (radar->in_string) {
            if (radar->escaped) {
                radar->escaped = false;
            } else if (letter == '\\') {
                radar->escaped = true;
            } else if (letter == '"') {
                radar->in_string = false;
            } else if (radar->depth == 1 && radar->word_len < NESP_RADAR_KEY) {
                radar->word[radar->word_len++] = letter;
            }
            continue;
        }
        switch (letter) {
        case '"':
            radar->in_string = true;
            if (radar->depth == 1) {
                radar->word_len = 0;
            }
            break;
        case ':':
            if (radar->depth == 1) {
                memcpy(radar->key, radar->word, radar->word_len);
                radar->key_len = radar->word_len;
            }
            break;
        case '[':
        case '{':
            if (radar->depth == 0) {
                radar->rooted = letter == (vessels ? '[' : '{');
            } else if (vessels && radar->depth == 1) {
                radar->listed = letter == '[';
                if (letter == '{') {        /* the header */
                    radar->taking = true;
                    radar->long_item = false;
                    radar->item_depth = 1;
                    radar->item[0] = letter;
                    radar->item_len = 1;
                }
            } else if (radar->depth == 1 && letter == '[' && zip) {
                radar->listed = radar->key_len == 6 && memcmp(radar->key, "places", 6) == 0;
            } else if (radar->depth == 1 && letter == '[') {
                radar->listed = (radar->key_len == 2 && memcmp(radar->key, "ac", 2) == 0)
                                || (radar->key_len == 8
                                    && memcmp(radar->key, "aircraft", 8) == 0);
            } else if (radar->depth == 2 && letter == '{' && radar->listed) {
                radar->taking = true;
                radar->long_item = false;
                radar->item_depth = 2;
                radar->item[0] = letter;
                radar->item_len = 1;
            }
            if (radar->depth < 0xFF) {
                radar->depth++;
            }
            break;
        case ']':
        case '}':
            if (radar->depth > 0) {
                radar->depth--;
            }
            if (radar->taking && radar->depth == radar->item_depth) {
                radar->taking = false;
                if (radar->long_item) {
                    /* passed over */
                } else if (zip) {
                    harbor_zip(radar, radar->item, radar->item_len);
                } else if (!vessels) {
                    aircraft(radar, radar->item, radar->item_len);
                } else if (radar->item_depth == 1) {
                    harbor_header(radar, radar->item, radar->item_len);
                } else {
                    harbor_vessel(radar, radar->item, radar->item_len);
                }
            } else if (radar->depth == 1) {
                radar->listed = false;
            } else if (radar->depth == 0) {
                radar->whole = radar->rooted;
            }
            break;
        default:
            break;
        }
    }
}

static bool current(nesp_radar_t *radar, uint32_t id)
{
    lock(radar);
    bool same = radar->generation == id;
    unlock(radar);
    return same;
}

static bool body(void *ctx, uint32_t id, const uint8_t *data, size_t len)
{
    nesp_radar_t *radar = ctx;
    if (!current(radar, id)) {
        return false;
    }
    if (radar->mapping) {
        return harbor_map_body(radar, id, data, len);
    }
    if (radar->parsing != id) {     /* the first of a new reply */
        radar->parsing = id;
        radar->depth = 0;
        radar->in_string = false;
        radar->escaped = false;
        radar->rooted = false;
        radar->listed = false;
        radar->whole = false;
        radar->taking = false;
        radar->word_len = 0;
        radar->key_len = 0;
        radar->total = 0;
        radar->near_count = 0;
        radar->matched = 0;
        radar->spotted = false;
        radar->spot_len = 0;
        radar->vouched = false;
        radar->refused = false;
    }
    take(radar, data, len);
    return true;
}

static void done(void *ctx, uint32_t id, bool ok, const char *why)
{
    nesp_radar_t *radar = ctx;
    bool well = ok && radar->parsing == id && radar->whole;
    if (radar->mapping) {
        well = ok && harbor_map_whole(radar, id);
    } else if (radar->ships && !radar->lookup) {
        well = well && radar->vouched && !radar->refused;   /* AISHub said all was well */
    }
    /* Zippopotam has no such zip code: a 404, which the fetch gives as these
     * words, as WEATHER has it. */
    bool none = !ok && radar->ships && radar->lookup && radar->kind == NESP_PLACE_ZIP
                && why != NULL && strcmp(why, NO_SUCH_ZIP) == 0;
    lock(radar);
    if (radar->generation == id) {
        radar->landed = well ? LANDED_WELL : none ? LANDED_NONE : LANDED_BADLY;
    }
    unlock(radar);
}

/* --- the link's task ----------------------------------------------------------- */

/* The next of a count that is never 0: what a console that has nothing
 * says it has. */
static uint8_t next(uint8_t serial)
{
    return serial == 0xFF ? 1 : (uint8_t)(serial + 1);
}

static bool online(const nesp_radar_t *radar)
{
    nesp_wifi_t wifi;
    memset(&wifi, 0, sizeof(wifi));
    radar->hal->wifi(radar->hal->ctx, &wifi);
    return wifi.state == NESP_WIFI_JOINED;
}

/* How it stands, as the console is told. */
static uint8_t state(const nesp_radar_t *radar)
{
    if (!radar->chosen) {
        return NESP_SCOPE_IDLE;
    }
    if (radar->ships && !radar->keyed) {
        return NESP_SCOPE_NO_KEY;
    }
    if (radar->place == PLACE_NONE) {
        return NESP_SCOPE_NOT_FOUND;
    }
    if (radar->place == PLACE_FAILED) {
        return NESP_SCOPE_NO_LOOKUP;
    }
    if (radar->held) {
        return NESP_SCOPE_READY;
    }
    if (!online(radar)) {
        return NESP_SCOPE_NO_WIFI;
    }
    return radar->place == PLACE_KNOWN ? NESP_SCOPE_WAITING : NESP_SCOPE_LOOKING;
}

/* The place chosen, for the nesp32's screen. */
static void place_text(const nesp_radar_t *radar, char *out, size_t size)
{
    if (radar->kind == NESP_PLACE_ZIP) {
        snprintf(out, size, "%.5s", (const char *)radar->named);
    } else if (radar->kind == NESP_PLACE_POINT) {
        snprintf(out, size, "%.2f,%.2f", radar->latitude / 1e6, radar->longitude / 1e6);
    } else {
        snprintf(out, size, "%.4s", radar->code);
    }
}

static void show(nesp_radar_t *radar)
{
    char text[48];
    char place[24];
    place_text(radar, place, sizeof(place));
    if (radar->chosen && radar->ships && !radar->keyed) {
        snprintf(text, sizeof(text), "%s: no AISHub key", place);
    } else if (!radar->chosen || radar->place != PLACE_KNOWN) {
        text[0] = 0;
    } else if (!radar->held) {
        snprintf(text, sizeof(text), "%s", place);
    } else {
        snprintf(text, sizeof(text), "%s: %u %s", place, (unsigned)radar->count,
                 radar->ships ? "ships" : "aircraft");
    }
    radar->hal->show(radar->hal->ctx, 0, text);
}

/* No airport's aircraft are wanted any more, or another's are. */
static void drop(nesp_radar_t *radar)
{
    radar->hal->fetch_stop(radar->hal->ctx);
    lock(radar);
    radar->generation++;
    radar->landed = 0;
    unlock(radar);
    radar->fetching = false;
    radar->mapping = false;
    radar->map_making = false;      /* the map held stays, for the same place again */
    radar->fetched = false;
    radar->good = false;
    radar->held = false;
    radar->count = 0;
    radar->flags = 0;
}

/* A fetch has ended, well or badly: the scene is what it brought, or the
 * one before with the flags to say what it is worth. */
static void publish(nesp_radar_t *radar, bool well, uint32_t now_ms)
{
    if (well) {
        uint8_t count = radar->near_count;
        bool renamed = !radar->held;
        for (uint8_t slot = 0; slot < TARGETS; slot++) {
            uint8_t names[NAMES];
            memset(names, ' ', NAMES);
            if (slot < count) {
                memcpy(radar->motion[slot], radar->near[slot].motion, MOTION);
                radar->motion[slot][0] |= slot;
                memcpy(names, radar->near[slot].names, NAMES);
                radar->held_where[slot][0] = radar->near[slot].latitude;
                radar->held_where[slot][1] = radar->near[slot].longitude;
                radar->held_where[slot][2] = radar->near[slot].distance;
                radar->held_ship[slot] = radar->near[slot].ship;
            }
            names[0] = slot;
            renamed = renamed || memcmp(names, radar->names[slot], NAMES) != 0;
            memcpy(radar->names[slot], names, NAMES);
        }
        radar->count = count;
        radar->flags = radar->total > TARGETS ? NESP_SCENE_TRUNCATED : 0;
        radar->good = true;
        radar->good_at = now_ms;
        if (renamed) {
            radar->names_serial = next(radar->names_serial);
        }
    } else {
        if (!radar->held) {
            for (uint8_t slot = 0; slot < TARGETS; slot++) {
                memset(radar->names[slot], ' ', NAMES);
                radar->names[slot][0] = slot;
            }
            radar->names_serial = next(radar->names_serial);
        }
        radar->flags = (uint8_t)((radar->flags & NESP_SCENE_TRUNCATED) | NESP_SCENE_DOWN);
        uint32_t stale_ms = radar->ships ? NESP_HARBOR_STALE_MS : NESP_RADAR_STALE_MS;
        if (!radar->good || now_ms - radar->good_at >= stale_ms) {
            radar->flags |= NESP_SCENE_STALE;
        }
    }
    radar->held = true;
    radar->serial = next(radar->serial);
    show(radar);
}

/* A fetch of this address, for the fetch's task to read as a place, as
 * aircraft or ships, or as a tile of the map. False if it could not be
 * started. */
static bool begin(nesp_radar_t *radar, const char *url, bool lookup, bool map)
{
    lock(radar);
    radar->generation++;
    radar->landed = 0;
    uint32_t id = radar->generation;
    unlock(radar);
    radar->lookup = lookup;         /* nothing is being read: the last fetch has landed */
    radar->mapping = map;
    const char *agent = map ? MAP_AGENT : radar->ships ? HARBOR_AGENT : AGENT;
    radar->fetching = online(radar)
                      && radar->hal->fetch(radar->hal->ctx, url, agent, id, &radar->sink);
    return radar->fetching;
}

static void fetch(nesp_radar_t *radar, uint32_t now_ms)
{
    char url[256];
    radar->fetched = true;
    radar->fetch_at = now_ms;
    radar->here[0] = radar->latitude / 1e6;
    radar->here[1] = radar->longitude / 1e6;
    if (radar->ships) {
        if (!harbor_key(radar)) {
            show(radar);
            return;                 /* NO_KEY: nothing to ask with */
        }
        harbor_url(radar, url, sizeof(url));
        /* AISHub counts a request it refuses, so this is the minute's. */
        if (online(radar)) {
            radar->sent = true;
            radar->sent_at = now_ms;
        }
    } else {
        snprintf(url, sizeof(url), SOURCE "/lat/%.6f/lon/%.6f/dist/%d", radar->here[0],
                 radar->here[1], ASKED_NM);
    }
    if (!begin(radar, url, false, false)) {
        publish(radar, false, now_ms);
    }
}

/* Ask where the airport is. */
static void look(nesp_radar_t *radar)
{
    char url[sizeof(WHERE) + 4 + sizeof(WHERE_END)];
    if (radar->kind == NESP_PLACE_ZIP) {
        snprintf(url, sizeof(url), ZIP_WHERE "%.5s", (const char *)radar->named);
        if (!begin(radar, url, true, false)) {
            radar->place = PLACE_FAILED;
        }
        return;
    }
    memcpy(url, WHERE, sizeof(WHERE) - 1);
    memcpy(url + sizeof(WHERE) - 1, radar->code, 4);
    memcpy(url + sizeof(WHERE) + 3, WHERE_END, sizeof(WHERE_END));
    if (!begin(radar, url, true, false)) {
        radar->place = PLACE_FAILED;
    }
}

/* The asking has ended: the place it brought, if it brought one that is a
 * place on the earth. A whole answer with none is no such airport. */
/* HARBOR: this place, as it was chosen, is the last found. */
static void remember(nesp_radar_t *radar)
{
    radar->found = true;
    radar->found_kind = radar->kind;
    memcpy(radar->found_named, radar->named, sizeof(radar->named));
    radar->found_range = radar->range;
    radar->found_latitude = radar->latitude;
    radar->found_longitude = radar->longitude;
}

static void located(nesp_radar_t *radar, uint8_t landed)
{
    double east, north;
    char *end;
    if (landed == LANDED_NONE) {
        radar->place = PLACE_NONE;
        return;
    }
    if (landed != LANDED_WELL) {
        radar->place = PLACE_FAILED;
        return;
    }
    radar->place = PLACE_NONE;
    if (!radar->spotted) {
        return;
    }
    east = strtod(radar->spot, &end);
    if (end == radar->spot || *end != ' ') {
        return;
    }
    const char *second = end;
    north = strtod(second, &end);
    if (end == second || *end != 0 || !(north >= -90.0 && north <= 90.0)
        || !(east >= -180.0 && east <= 180.0)) {
        return;
    }
    radar->latitude = (int32_t)nearbyint(north * 1e6);
    radar->longitude = (int32_t)nearbyint(east * 1e6);
    radar->place = PLACE_KNOWN;
    if (radar->ships) {
        remember(radar);
    }
    if (radar->kind != NESP_PLACE_AIRPORT) {
        show(radar);
        return;
    }
    if (!radar->known || memcmp(radar->known_code, radar->code, 4) != 0
        || radar->known_latitude != radar->latitude
        || radar->known_longitude != radar->longitude) {
        radar->known = true;
        memcpy(radar->known_code, radar->code, 4);
        radar->known_latitude = radar->latitude;
        radar->known_longitude = radar->longitude;
        if (radar->hal->airport_keep != NULL) {
            radar->hal->airport_keep(radar->hal->ctx, radar->code, radar->latitude,
                                     radar->longitude);
        }
    }
    show(radar);
}

/* --- the link's task: HARBOR's map ---------------------------------------------- */

/* The map is of the place chosen and its range, or none is held: one held
 * or being made of another is let go. */
static void aim(nesp_radar_t *radar)
{
    uint8_t range = (uint8_t)radar->range_nm;
    if (radar->place != PLACE_KNOWN
        || (radar->map_latitude == radar->latitude && radar->map_longitude == radar->longitude
            && radar->map_range == range)) {
        return;
    }
    if (radar->fetching && radar->mapping) {
        radar->hal->fetch_stop(radar->hal->ctx);
        lock(radar);
        radar->generation++;
        radar->landed = 0;
        unlock(radar);
        radar->fetching = false;
    }
    radar->mapping = false;
    radar->map_making = false;
    radar->map_serial = 0;
    radar->map_latitude = radar->latitude;
    radar->map_longitude = radar->longitude;
    radar->map_range = range;
}

/* A map is held for the place chosen and its range. */
static bool map_held(const nesp_radar_t *radar)
{
    return radar->map_serial != 0 && radar->place == PLACE_KNOWN
           && radar->map_latitude == radar->latitude && radar->map_longitude == radar->longitude
           && radar->map_range == (uint8_t)radar->range_nm;
}

/* A tile has come, well or badly. When the last has come well, the map is
 * held, with a serial it has not had before; when one comes badly, there
 * is no map, and none is asked for again for a minute. */
static void map_landed(nesp_radar_t *radar, bool well, uint32_t now_ms)
{
    radar->mapping = false;
    if (!radar->map_making) {
        return;
    }
    if (well && ++radar->map_tile < radar->map_tiles) {
        return;                     /* the next tile, at the next tick */
    }
    radar->map_making = false;
    if (!well) {
        radar->map_failed = true;
        radar->map_failed_at = now_ms;
        return;
    }
    harbor_map_codes(radar);
    radar->map_failed = false;
    radar->map_last = next(radar->map_last);
    radar->map_serial = radar->map_last;
}

/* The next tile of a map the console has asked for and none is held of,
 * if nothing else is being fetched. */
static void map_next(nesp_radar_t *radar, uint32_t now_ms)
{
    if (!radar->map_wanted || radar->fetching || radar->place != PLACE_KNOWN
        || radar->map_serial != 0
        || (radar->map_failed && now_ms - radar->map_failed_at < NESP_HARBOR_MAP_RETRY_MS)
        || !online(radar)) {
        return;
    }
    if (!radar->map_making) {
        radar->map_making = true;
        radar->map_tile = 0;
        if (!harbor_map_plan(radar)) {
            map_landed(radar, false, now_ms);
            return;
        }
    }
    char url[96];
    harbor_map_url(radar, url, sizeof(url));
    if (!begin(radar, url, false, true)) {
        map_landed(radar, false, now_ms);
    }
}

static bool tick(void *ctx, uint32_t now_ms)
{
    nesp_radar_t *radar = ctx;
    if (radar->asked || !radar->timed) {
        radar->asked = false;
        radar->timed = true;
        radar->asked_at = now_ms;
    }
    if (!radar->chosen) {
        return false;
    }
    if (radar->ships) {
        aim(radar);
    }
    if (radar->fetching) {
        lock(radar);
        uint8_t landed = radar->landed;
        radar->landed = 0;
        unlock(radar);
        if (landed == 0) {
            return false;
        }
        radar->fetching = false;
        if (radar->lookup) {
            located(radar, landed);
        } else if (radar->mapping) {
            map_landed(radar, landed == LANDED_WELL, now_ms);
        } else {
            publish(radar, landed == LANDED_WELL, now_ms);
        }
    }
    if (now_ms - radar->asked_at >= NESP_RADAR_UNASKED_MS) {
        return false;               /* nobody is looking */
    }
    if (radar->ships && !radar->keyed) {
        map_next(radar, now_ms);    /* a map needs no key */
        return false;               /* NO_KEY: until AIRPORT finds one */
    }
    if (radar->place == PLACE_UNKNOWN) {
        if (online(radar)) {
            look(radar);
        }
    } else if (radar->place == PLACE_KNOWN) {
        bool due;
        if (radar->ships) {
            /* AISHub's minute is the nesp32's, whatever airport the last
             * request was for. */
            due = !radar->sent || now_ms - radar->sent_at >= NESP_HARBOR_POLL_MS;
        } else {
            due = !radar->fetched || now_ms - radar->fetch_at >= NESP_RADAR_POLL_MS;
        }
        if (due) {
            fetch(radar, now_ms);
        }
    }
    if (radar->ships) {
        map_next(radar, now_ms);    /* AISHub first: a tile waits for its minute's fetch */
    }
    return false;
}

static size_t named_len(uint8_t kind)
{
    return kind == NESP_PLACE_AIRPORT ? 4 : kind == NESP_PLACE_ZIP ? 5 : kind == NESP_PLACE_POINT ? 8 : 0;
}

static int32_t le32(const uint8_t *bytes)
{
    return (int32_t)((uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16
                     | (uint32_t)bytes[3] << 24);
}

/* The place a request names: RADAR's ID, LETTERS[4]; or HARBOR's ID, KIND,
 * the place, and a RANGE or not, or RADAR's again, which is an airport at
 * the range HARBOR has unless it is told. False: it is not one. */
static bool chosen_place(const nesp_radar_t *radar, const nesp_request_t *request,
                         uint8_t *kind, uint8_t *named, uint8_t *range)
{
    const uint8_t *data = request->data;
    size_t len = request->len;
    memset(named, 0, 8);
    *kind = NESP_PLACE_AIRPORT;
    *range = 0;
    if (len == 5 && data[1] >= 'A' && data[1] <= 'Z') {
        memcpy(named, data + 1, 4);
    } else {
        if (!radar->ships || len < 2) {
            return false;
        }
        *kind = data[1];
        size_t count = named_len(*kind);
        if (count == 0 || (len != 2 + count && len != 3 + count)) {
            return false;
        }
        memcpy(named, data + 2, count);
        *range = len == 3 + count ? data[2 + count] : 0;
        if (*range > NESP_HARBOR_RANGE_MAX) {
            return false;
        }
    }
    for (size_t index = 0; index < named_len(*kind); index++) {
        if (*kind == NESP_PLACE_AIRPORT && (named[index] < 'A' || named[index] > 'Z')) {
            return false;
        }
        if (*kind == NESP_PLACE_ZIP && (named[index] < '0' || named[index] > '9')) {
            return false;
        }
    }
    if (*kind == NESP_PLACE_POINT) {
        int32_t latitude = le32(named), longitude = le32(named + 4);
        if (latitude < -90000000 || latitude > 90000000 || longitude < -180000000
            || longitude > 180000000) {
            return false;
        }
    }
    return true;
}

/* HARBOR: the place is the same and the range is not, or the middle has
 * moved (PAN). The scene held is drawn again to it at once, less the ships
 * beyond it, the nearest first again; ships that were beyond the last come
 * with the next fetch. */
static void zoom(nesp_radar_t *radar)
{
    if (!radar->held) {
        return;
    }
    uint8_t motion[TARGETS][MOTION];
    uint8_t names[TARGETS][NAMES];
    double where[TARGETS][3];
    nesp_harbor_ship_t ship[TARGETS];
    uint8_t count = 0;
    for (uint8_t slot = 0; slot < radar->count; slot++) {
        double bearing;
        double distance = how_far(radar, radar->held_where[slot][0], radar->held_where[slot][1],
                                  &bearing);
        if (distance > radar->range_nm) {
            continue;
        }
        uint8_t place = count;      /* after any as near: the order they had */
        while (place > 0 && distance < where[place - 1][2]) {
            place--;
        }
        size_t moved = (size_t)(count - place);
        memmove(motion[place + 1], motion[place], moved * MOTION);
        memmove(names[place + 1], names[place], moved * NAMES);
        memmove(where[place + 1], where[place], moved * sizeof(where[0]));
        memmove(&ship[place + 1], &ship[place], moved * sizeof(ship[0]));
        memcpy(motion[place], radar->motion[slot], MOTION);
        put_xy(motion[place], distance, bearing, radar->range_nm);
        motion[place][8] = (uint8_t)scope_rounded(distance * 10, 0, 99);
        memcpy(names[place], radar->names[slot], NAMES);
        where[place][0] = radar->held_where[slot][0];
        where[place][1] = radar->held_where[slot][1];
        where[place][2] = distance;
        ship[place] = radar->held_ship[slot];
        count++;
    }
    for (uint8_t slot = 0; slot < TARGETS; slot++) {
        if (slot < count) {
            motion[slot][0] = (uint8_t)((motion[slot][0] & ~7u) | slot);
        } else {
            memset(names[slot], ' ', NAMES);
        }
        names[slot][0] = slot;
    }
    if (memcmp(names, radar->names, sizeof(names)) != 0) {
        memcpy(radar->names, names, sizeof(names));
        radar->names_serial = next(radar->names_serial);
    }
    memcpy(radar->motion, motion, (size_t)count * MOTION);
    memcpy(radar->held_where, where, (size_t)count * sizeof(where[0]));
    memcpy(radar->held_ship, ship, (size_t)count * sizeof(ship[0]));
    radar->count = count;
    radar->serial = next(radar->serial);
    show(radar);
}

static void airport(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    uint8_t kind, named[8], range;
    if (!chosen_place(radar, request, &kind, named, &range)) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    bool was_keyed = radar->keyed;
    if (radar->ships) {
        harbor_key(radar);          /* asked each time: it may have been set since */
    }
    bool same = radar->chosen && radar->kind == kind && memcmp(radar->named, named, 8) == 0;
    if (same && radar->range != range) {
        radar->airport_id = request->data[0];
        radar->range = range;
        radar->range_nm = range != 0 ? range : NESP_HARBOR_RANGE_NM;
        if (radar->place == PLACE_KNOWN) {
            remember(radar);
        }
        zoom(radar);
    } else if (!same || radar->airport_id != request->data[0]) {
        drop(radar);
        radar->chosen = true;
        radar->airport_id = request->data[0];
        radar->kind = kind;
        memcpy(radar->named, named, 8);
        radar->range = range;
        radar->range_nm = !radar->ships ? RANGE_NM : range != 0 ? range : NESP_HARBOR_RANGE_NM;
        if (kind == NESP_PLACE_AIRPORT) {
            memcpy(radar->code, named, 4);
        }
        radar->place = PLACE_UNKNOWN;
        if (kind == NESP_PLACE_POINT) {
            radar->latitude = le32(named);
            radar->longitude = le32(named + 4);
            radar->place = PLACE_KNOWN;
            remember(radar);
        } else if (radar->ships && radar->found && radar->found_kind == kind
                   && memcmp(radar->found_named, named, 8) == 0) {
            radar->latitude = radar->found_latitude;
            radar->longitude = radar->found_longitude;
            radar->place = PLACE_KNOWN;
            remember(radar);
        } else if (kind == NESP_PLACE_AIRPORT && radar->known
                   && memcmp(radar->known_code, named, 4) == 0) {
            radar->latitude = radar->known_latitude;
            radar->longitude = radar->known_longitude;
            radar->place = PLACE_KNOWN;
            if (radar->ships) {
                remember(radar);
            }
        }
        show(radar);
    } else if (radar->keyed != was_keyed) {
        show(radar);
    }
    radar->asked = true;
    reply->data[0] = state(radar);
    reply->data[1] = radar->airport_id;
    reply->len = 2;
    reply->once = true;             /* the same request again asks how it stands now */
}

/* Is the request about the airport that is chosen? If not the reply says
 * IDLE of the airport it named, and the console chooses it again. */
static bool about(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    reply->once = true;
    reply->data[1] = request->data[0];
    if (radar->chosen && radar->airport_id == request->data[0]) {
        radar->asked = true;
        reply->data[0] = state(radar);
        return true;
    }
    reply->data[0] = NESP_SCOPE_IDLE;
    return false;
}

static void scene(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 2 || reply->max < 6 + TARGETS * MOTION) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->len = 4;
    reply->data[2] = 0;
    reply->data[3] = 0;
    if (!about(radar, request, reply) || reply->data[0] != NESP_SCOPE_READY) {
        return;
    }
    reply->data[2] = radar->serial;
    reply->data[3] = radar->names_serial;
    if (request->data[1] == radar->serial) {
        return;                     /* the console has it */
    }
    reply->data[4] = radar->flags;
    reply->data[5] = radar->count;
    memcpy(reply->data + 6, radar->motion, (size_t)radar->count * MOTION);
    reply->len = (uint8_t)(6 + radar->count * MOTION);
}

static void identity(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 2 || request->data[1] >= TARGETS || reply->max < 3 + NAMES) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->len = 3;
    reply->data[2] = 0;
    if (!about(radar, request, reply) || reply->data[0] != NESP_SCOPE_READY) {
        return;
    }
    reply->data[2] = radar->names_serial;
    memcpy(reply->data + 3, radar->names[request->data[1]], NAMES);
    reply->len = 3 + NAMES;
}

/* The letters of the last airport found, for the console to begin with:
 * four bytes, or none if nothing is kept. */
static void last(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 0) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    if (radar->ships && radar->found) {
        /* HARBOR: KIND, the place and RANGE, as PLACE takes them after its ID. */
        size_t count = named_len(radar->found_kind);
        reply->data[0] = radar->found_kind;
        memcpy(reply->data + 1, radar->found_named, count);
        reply->data[1 + count] = radar->found_range;
        reply->len = (uint8_t)(2 + count);
    } else if (radar->ships && radar->known) {
        reply->data[0] = NESP_PLACE_AIRPORT;
        memcpy(reply->data + 1, radar->known_code, 4);
        reply->data[5] = 0;
        reply->len = 6;
    } else if (radar->known) {
        memcpy(reply->data, radar->known_code, 4);
        reply->len = 4;
    }
    reply->once = true;
}

/* HARBOR: a row of the map of the place chosen, if one is held. */
static void map(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 2 || request->data[1] >= NESP_HARBOR_MAP_ROWS
        || reply->max < 3 + NESP_HARBOR_MAP_CODES) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->len = 3;
    reply->data[2] = 0;
    if (!about(radar, request, reply)) {
        return;
    }
    radar->map_wanted = true;
    if (!map_held(radar)) {
        return;
    }
    reply->data[2] = radar->map_serial;
    memcpy(reply->data + 3, radar->map_codes[request->data[1]], NESP_HARBOR_MAP_CODES);
    reply->len = 3 + NESP_HARBOR_MAP_CODES;
}

/* HARBOR: a line of the card of the ship in a slot. As IDENTITY, with the
 * names' serial, and with a ship there, the line's number and its text. */
static void details(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 3 || request->data[1] >= TARGETS
        || request->data[2] >= NESP_HARBOR_DETAIL_LINES
        || reply->max < 4 + NESP_HARBOR_DETAIL_WIDTH) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->len = 3;
    reply->data[2] = 0;
    if (!about(radar, request, reply) || reply->data[0] != NESP_SCOPE_READY) {
        return;
    }
    reply->data[2] = radar->names_serial;
    uint8_t slot = request->data[1];
    if (slot >= radar->count) {
        return;
    }
    uint32_t now = 0;
    bool timed = radar->hal->time != NULL && radar->hal->time(radar->hal->ctx, &now);
    reply->data[3] = request->data[2];
    harbor_card(&radar->held_ship[slot], radar->held_where[slot][0], radar->held_where[slot][1],
                request->data[2], timed, now, reply->data + 4);
    reply->len = 4 + NESP_HARBOR_DETAIL_WIDTH;
}

static void put_le32(uint8_t *bytes, int32_t value)
{
    uint32_t bits = (uint32_t)value;
    for (int index = 0; index < 4; index++) {
        bytes[index] = (uint8_t)(bits >> (8 * index));
    }
}

/* HARBOR: the point DX and DY quarters of the range east and north of the
 * middle, chosen as PLACE would choose it, at the same range. The scene
 * held is drawn again to it at once; the next fetch is of the box round
 * it, at AISHub's minute. With no middle yet, nothing is done. */
static void pan(nesp_radar_t *radar, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 3) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->once = true;             /* the same again asks how it stands */
    reply->len = 2;
    if (!radar->chosen) {
        reply->data[0] = NESP_SCOPE_IDLE;
        reply->data[1] = request->data[0];
        return;
    }
    radar->asked = true;
    if (radar->place == PLACE_KNOWN && radar->airport_id != request->data[0]) {
        double step = radar->range_nm / 4.0 / 60.0;     /* degrees of latitude */
        double latitude = radar->latitude / 1e6, longitude = radar->longitude / 1e6;
        double across = cos(scope_radians(latitude));
        if (across > 0.01) {
            longitude += (int8_t)request->data[1] * step / across;
            longitude = fmod(fmod(longitude + 180.0, 360.0) + 360.0, 360.0) - 180.0;
        }
        latitude += (int8_t)request->data[2] * step;
        latitude = latitude < -90.0 ? -90.0 : latitude > 90.0 ? 90.0 : latitude;
        radar->airport_id = request->data[0];
        radar->kind = NESP_PLACE_POINT;
        radar->latitude = (int32_t)nearbyint(latitude * 1e6);
        radar->longitude = (int32_t)nearbyint(longitude * 1e6);
        put_le32(radar->named, radar->latitude);
        put_le32(radar->named + 4, radar->longitude);
        remember(radar);
        radar->here[0] = radar->latitude / 1e6;
        radar->here[1] = radar->longitude / 1e6;
        zoom(radar);
        if (!radar->held) {
            show(radar);
        }
    }
    reply->data[0] = state(radar);
    reply->data[1] = radar->airport_id;
}

static void handle(void *ctx, const nesp_request_t *request, nesp_reply_t *reply)
{
    nesp_radar_t *radar = ctx;
    switch (request->op) {
    case NESP_RADAR_LAST:
        last(radar, request, reply);
        break;
    case NESP_RADAR_AIRPORT:
        airport(radar, request, reply);
        break;
    case NESP_RADAR_SCENE:
        scene(radar, request, reply);
        break;
    case NESP_RADAR_IDENTITY:
        identity(radar, request, reply);
        break;
    case NESP_HARBOR_MAP:
        if (radar->ships) {
            map(radar, request, reply);
        } else {
            reply->status = NESP_ST_NO_OP;
        }
        break;
    case NESP_HARBOR_PAN:
        if (radar->ships) {
            pan(radar, request, reply);
        } else {
            reply->status = NESP_ST_NO_OP;
        }
        break;
    case NESP_HARBOR_DETAILS:
        if (radar->ships) {
            details(radar, request, reply);
        } else {
            reply->status = NESP_ST_NO_OP;
        }
        break;
    default:
        reply->status = NESP_ST_NO_OP;
        break;
    }
}

static void activate(void *ctx)
{
    nesp_radar_t *radar = ctx;
    drop(radar);
    radar->chosen = false;
    radar->asked = false;
    radar->map_wanted = false;
    if (!radar->known && radar->hal->airport_last != NULL) {
        /* Nothing found since the power came on: what was kept from before. */
        radar->known = radar->hal->airport_last(radar->hal->ctx, radar->known_code,
                                                &radar->known_latitude,
                                                &radar->known_longitude);
    }
    radar->timed = false;
}

/* Another ROM is on the console, or this one was reset. */
static void deactivate(void *ctx)
{
    nesp_radar_t *radar = ctx;
    drop(radar);
    radar->chosen = false;
    radar->hal->show(radar->hal->ctx, 0, "");
}

void scope_service(nesp_service_t *service, nesp_radar_t *radar, const nesp_hal_t *hal,
                   bool ships)
{
    memset(radar, 0, sizeof(*radar));
    radar->hal = hal;
    radar->ships = ships;
    radar->sink.ctx = radar;
    radar->sink.body = body;
    radar->sink.done = done;
    memset(service, 0, sizeof(*service));
    service->id = ships ? NESP_SVC_HARBOR : NESP_SVC_RADAR;
    service->name = ships ? "HARBOR" : "RADAR";
    service->activate = activate;
    service->deactivate = deactivate;
    service->handle = handle;
    service->tick = tick;
    service->ctx = radar;
}

void nesp_radar_service(nesp_service_t *service, nesp_radar_t *radar, const nesp_hal_t *hal)
{
    scope_service(service, radar, hal, false);
}
