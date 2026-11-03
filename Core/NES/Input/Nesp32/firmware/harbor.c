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
