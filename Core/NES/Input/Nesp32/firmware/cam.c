/* CAM: NES Cam, a camera's latest frame for a console to load into its
 * CHR-RAM (docs/PROTOCOL.md).
 *
 * A frame is a NES pattern table: NESP_CAM_FRAME_BYTES (4096) bytes, 256
 * tiles of 16 bytes in the NES's own two planes, the tiles 16 by 16 in
 * rows across a 128-pixel square. Something else (a web page with a
 * camera) makes the frames and serves the latest at an address; the
 * nesp32 fetches it from there, over and over, while a console watches,
 * and gives it to the console in NESP_CAM_CHUNKS chunks of
 * NESP_CAM_CHUNK_BYTES. The nesp32 makes nothing of the bytes: a reply of
 * exactly 4096 is a frame, and anything else is a fetch that failed.
 *
 * The address is the board's, never the console's: the hal's setting
 * "nescam_url", an http or https address. It is read when a console
 * begins to watch and before every fetch, so a change takes at the next.
 *
 * WATCH says how it stands and which frame is held, by its serial; HAVE,
 * the serial of the frame the console holds whole, says it is done with
 * that one. WATCH is what begins the fetching, and keeps it going. CHUNK
 * is a piece of the frame held, by serial and index; a chunk of a frame
 * that is no longer held is answered with the serial of the one that is,
 * and nothing more, and the console starts over on it.
 *
 * Two quicker ways to the same frame. TILES gives only the tiles of the
 * frame held that differ from the frame the console has: the board keeps
 * that one, `base`, copied from the frame held whenever the console says
 * it has it whole (WATCH's HAVE, TILES's HAVE, the last CHUNK, or TILES
 * finding nothing left to send). SMALL is the frame held at a quarter of
 * the size, made once for each frame held, when it is first asked for.
 *
 * TURBO is for a board on the expansion port (the hal's fast_send): its
 * reply is short, and the tiles go as a burst of bytes to the port's front
 * end, which the console reads a whole byte a read, straight into its
 * PPU. The burst ends with the console's TAG and its complement, which is
 * all the console checks: a byte lost or not there in time moves them.
 * While a console loads by TURBO the fetches come sooner (CAM_TURBO_MIN_MS).
 *
 * There are two frames' worth of memory, and the base and the small frame
 * besides, asked of the hal when a console first watches, all or none,
 * and kept, as TUNER keeps its decoder's: the fetch's task
 * may be in the middle of one whenever it would be given back. One is the
 * frame held, which the console is given; a fetch fills the other. A
 * frame that comes whole waits there until the console has had the one
 * held to its last chunk (CHUNK's or SMALL's, or TILES's last tile), or
 * says (HAVE) that it holds it, or has asked for none of it for
 * NESP_CAM_STALL_MS; then the two change places, and
 * the serial with them. So the console is never given a frame that
 * changes under it, and one that loads frames as fast as it can is never
 * cut off in the middle of one by the next.
 *
 * Two tasks meet here, as in weather.c. The link's task runs handle() and
 * tick(), and has the frames but the one being filled. The fetch's task
 * runs body() and done(): it fills `target`, and when the reply has ended
 * says so (`landed`, under the hal's lock). `generation` is which fetch
 * is wanted.
 *
 * The clock is tick()'s. One fetch at a time, and only while a console
 * watches (it has asked within NESP_CAM_UNASKED_MS). The next begins once
 * the frame it brought is held, NESP_CAM_MIN_MS at the soonest after the
 * last began, or NESP_CAM_RETRY_MS after one that failed.
 */
#include <string.h>

#include "nesp_services.h"

#define FRAME NESP_CAM_FRAME_BYTES
#define CHUNK NESP_CAM_CHUNK_BYTES
#define CHUNKS NESP_CAM_CHUNKS
#define HEAD 3                      /* STATE, SERIAL, INDEX */
#define TILE NESP_CAM_TILE_BYTES
#define TILES (FRAME / TILE)
#define TILES_HEAD 5                /* STATE, SERIAL, NEXT, DONE, COUNT */
#define ENTRY (1 + TILE)            /* a tile's index, and the tile */
#define SMALL NESP_CAM_SMALL_BYTES
#define SMALL_CHUNKS NESP_CAM_SMALL_CHUNKS
#define SIDE 16                     /* tiles across a frame */
#define SMALL_SIDE 8                /* and across a small one */
#define TURBO_HEAD 4                /* STATE, SERIAL, FROM, COUNT */
#define SETTING "nescam_url"
#define AGENT "nesp32-cam/0.1 (NES Cam on a nesp32; +https://github.com/Armitage64/nesp32)"

#define LANDED_WELL 1
#define LANDED_BADLY 2

_Static_assert(CHUNK * CHUNKS == FRAME, "a frame is its chunks");
_Static_assert(HEAD + CHUNK <= NESP_PAYLOAD_MAX, "a chunk fits a reply");
_Static_assert(TILES == SIDE * SIDE && TILES == 256, "a tile's index is a byte");
_Static_assert(TILES_HEAD + NESP_CAM_TILES_MAX * ENTRY <= NESP_PAYLOAD_MAX, "the tiles fit a reply");
_Static_assert(TILES_HEAD + (NESP_CAM_TILES_MAX + 1) * ENTRY > NESP_PAYLOAD_MAX, "and no more do");
_Static_assert(SMALL == SMALL_SIDE * SMALL_SIDE * TILE, "a small frame is a quarter of a frame");
_Static_assert(CHUNK * SMALL_CHUNKS == SMALL, "a small frame is its chunks");

static void lock(nesp_cam_t *cam)
{
    cam->hal->lock(cam->hal->ctx);
}

static void unlock(nesp_cam_t *cam)
{
    cam->hal->unlock(cam->hal->ctx);
}

/* --- the fetch's task ---------------------------------------------------------- */

/* The frame the fetch with this id fills, or NULL if it is not wanted. */
static uint8_t *target_of(nesp_cam_t *cam, uint32_t id)
{
    lock(cam);
    uint8_t *target = cam->generation == id ? cam->target : NULL;
    unlock(cam);
    return target;
}

/* The fetch's task begins on a reply: nothing of it has come. */
static void opened(nesp_cam_t *cam, uint32_t id)
{
    if (cam->parsing != id) {
        cam->parsing = id;
        cam->got = 0;
        cam->too_long = false;
    }
}

static bool body(void *ctx, uint32_t id, const uint8_t *data, size_t len)
{
    nesp_cam_t *cam = ctx;
    uint8_t *target = target_of(cam, id);
    if (target == NULL) {
        return false;
    }
    opened(cam, id);
    if (len > FRAME - cam->got) {
        cam->too_long = true;
        return false;               /* not a frame */
    }
    memcpy(target + cam->got, data, len);
    cam->got += (uint32_t)len;
    return true;
}

static void done(void *ctx, uint32_t id, bool ok, const char *why)
{
    (void)why;
    nesp_cam_t *cam = ctx;
    if (target_of(cam, id) == NULL) {
        return;
    }
    opened(cam, id);
    uint8_t how = ok && !cam->too_long && cam->got == FRAME ? LANDED_WELL : LANDED_BADLY;
    lock(cam);
    if (cam->generation == id) {
        cam->landed = how;
    }
    unlock(cam);
}

/* --- the link's task ----------------------------------------------------------- */

static uint8_t next(uint8_t serial)
{
    return serial == 0xFF ? 1 : (uint8_t)(serial + 1);
}

static bool online(const nesp_cam_t *cam)
{
    nesp_wifi_t wifi;
    memset(&wifi, 0, sizeof(wifi));
    cam->hal->wifi(cam->hal->ctx, &wifi);
    return wifi.state == NESP_WIFI_JOINED;
}

/* The camera's address, into url, which holds NESP_CAM_URL_MAX and a 0.
 * False if the board has none, or one that is not an http or https
 * address of printable characters. */
static bool source(nesp_cam_t *cam, char *url)
{
    bool sourced = cam->hal->setting != NULL
                   && cam->hal->setting(cam->hal->ctx, SETTING, url, NESP_CAM_URL_MAX + 1);
    if (sourced) {
        url[NESP_CAM_URL_MAX] = 0;
        size_t len = strlen(url);
        sourced = (strncmp(url, "http://", 7) == 0 && len > 7)
                  || (strncmp(url, "https://", 8) == 0 && len > 8);
        for (size_t index = 0; index < len && sourced; index++) {
            sourced = url[index] > ' ' && url[index] < 0x7F;
        }
    }
    cam->sourced = sourced;
    return sourced;
}

/* How it stands, as the console is told. */
static uint8_t state(const nesp_cam_t *cam)
{
    if (!cam->watched) {
        return NESP_CAM_IDLE;
    }
    if (!cam->sourced) {
        return NESP_CAM_NO_SOURCE;
    }
    if (!online(cam)) {
        return NESP_CAM_NO_WIFI;
    }
    if (cam->failed || cam->no_memory) {
        return NESP_CAM_FAILED;
    }
    return cam->holding ? NESP_CAM_READY : NESP_CAM_WAITING;
}

/* The serial of the frame held, or 0 for none. */
static uint8_t held_serial(const nesp_cam_t *cam)
{
    return cam->holding ? cam->serial : 0;
}

/* The memory for the two frames, the base and the small frame: all of it,
 * or none. */
static void have_memory(nesp_cam_t *cam)
{
    uint8_t **blocks[] = {&cam->frames[0], &cam->frames[1], &cam->base, &cam->small};
    const size_t sizes[] = {FRAME, FRAME, FRAME, SMALL};
    size_t had = 0;
    while (had < 4) {
        *blocks[had] = cam->hal->alloc(cam->hal->ctx, sizes[had]);
        if (*blocks[had] == NULL) {
            break;
        }
        had++;
    }
    if (had < 4) {
        while (had > 0) {
            had--;
            cam->hal->release(cam->hal->ctx, *blocks[had]);
            *blocks[had] = NULL;
        }
    }
}

/* A console has begun to watch: the memory for the frames, if it has not
 * been had, and the address as it is now. */
static void begin_watching(nesp_cam_t *cam)
{
    if (cam->frames[0] == NULL) {
        have_memory(cam);
    }
    cam->no_memory = cam->frames[0] == NULL;
    char url[NESP_CAM_URL_MAX + 1];
    source(cam, url);
    cam->watched = true;
    cam->watching = true;
}

/* The frame that came is held now, and the one that was is the next to be
 * filled. */
static void hold(nesp_cam_t *cam, uint32_t now_ms)
{
    cam->held = (uint8_t)(1 - cam->held);
    cam->holding = true;
    cam->serial = next(cam->serial);
    if (cam->based && cam->base_serial == cam->serial) {
        cam->based = false;         /* 255 frames on: the serial is another frame's now */
    }
    if (cam->partial) {
        cam->mixed = true;          /* the console was part of the way to the last */
    }
    cam->partial = false;
    cam->small_made = false;
    cam->waiting = false;
    cam->finished = false;
    cam->chunk_at = now_ms;
}

/* The console holds the frame held whole: it is the base now. */
static void keep(nesp_cam_t *cam)
{
    if (!cam->based || cam->base_serial != cam->serial) {
        memcpy(cam->base, cam->frames[cam->held], FRAME);
        cam->base_serial = cam->serial;
        cam->based = true;
    }
    cam->partial = false;
    cam->mixed = false;
    cam->finished = true;
}

static void fetch(nesp_cam_t *cam, uint32_t now_ms)
{
    char url[NESP_CAM_URL_MAX + 1];
    cam->fetched = true;
    cam->fetch_at = now_ms;
    if (!source(cam, url)) {
        return;                     /* looked for again after NESP_CAM_RETRY_MS */
    }
    lock(cam);
    cam->generation++;
    cam->landed = 0;
    cam->target = cam->frames[1 - cam->held];
    uint32_t id = cam->generation;
    unlock(cam);
    cam->fetching = cam->hal->fetch(cam->hal->ctx, url, AGENT, id, &cam->sink);
    if (!cam->fetching) {
        cam->failed = true;
    }
}

/* No fetch is wanted any more: the console has gone. */
static void drop(nesp_cam_t *cam)
{
    if (cam->fetching) {
        cam->hal->fetch_stop(cam->hal->ctx);
    }
    lock(cam);
    cam->generation++;
    cam->landed = 0;
    cam->target = NULL;
    unlock(cam);
    cam->fetching = false;
    cam->fetched = false;
    cam->holding = false;
    cam->based = false;
    cam->partial = false;
    cam->mixed = false;
    cam->small_made = false;
    cam->waiting = false;
    cam->finished = false;
    cam->watched = false;
    cam->watching = false;
    cam->sourced = false;
    cam->failed = false;
    cam->no_memory = false;
    cam->asked = false;
    cam->chunked = false;
    cam->turbo = false;
}

static bool tick(void *ctx, uint32_t now_ms)
{
    nesp_cam_t *cam = ctx;
    if (cam->asked) {
        cam->asked = false;
        cam->asked_at = now_ms;
    }
    if (cam->chunked) {
        cam->chunked = false;
        cam->chunk_at = now_ms;
    }
    if (cam->fetching) {
        lock(cam);
        uint8_t landed = cam->landed;
        cam->landed = 0;
        unlock(cam);
        if (landed != 0) {
            cam->fetching = false;
            cam->failed = landed != LANDED_WELL;
            if (landed == LANDED_WELL && !cam->holding) {
                hold(cam, now_ms);  /* the first: there is nothing to finish */
            } else if (landed == LANDED_WELL) {
                cam->waiting = true;
            }
        }
    }
    if (cam->waiting && (cam->finished || now_ms - cam->chunk_at >= NESP_CAM_STALL_MS)) {
        hold(cam, now_ms);
    }
    if (cam->watching && now_ms - cam->asked_at >= NESP_CAM_UNASKED_MS) {
        cam->watching = false;      /* nobody is looking */
    }
    if (!cam->watching || cam->fetching || cam->waiting || cam->frames[0] == NULL
        || !online(cam)) {
        return false;
    }
    uint32_t pause = cam->failed || !cam->sourced ? NESP_CAM_RETRY_MS
                     : cam->turbo ? NESP_CAM_TURBO_MIN_MS : NESP_CAM_MIN_MS;
    if (!cam->fetched || now_ms - cam->fetch_at >= pause) {
        fetch(cam, now_ms);
    }
    return false;
}

static void watch(nesp_cam_t *cam, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 1) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    if (!cam->watching) {
        begin_watching(cam);
    }
    cam->asked = true;
    if (cam->holding && request->data[0] == cam->serial) {
        keep(cam);                  /* the console holds it whole */
    }
    reply->data[0] = state(cam);
    reply->data[1] = held_serial(cam);
    reply->len = 2;
    reply->once = true;             /* the same request again asks how it stands now */
}

static void chunk(nesp_cam_t *cam, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 2 || request->data[1] >= CHUNKS || reply->max < HEAD + CHUNK) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->once = true;
    if (cam->watching) {
        cam->asked = true;          /* loading a frame is watching; it is not beginning to */
    }
    reply->data[0] = state(cam);
    reply->data[1] = held_serial(cam);
    reply->len = 2;
    if (!cam->holding || request->data[0] != cam->serial) {
        return;                     /* not the frame held: the console starts over */
    }
    uint8_t index = request->data[1];
    reply->data[2] = index;
    memcpy(reply->data + HEAD, cam->frames[cam->held] + (size_t)index * CHUNK, CHUNK);
    reply->len = HEAD + CHUNK;
    cam->chunked = true;
    if (index == CHUNKS - 1) {
        keep(cam);                  /* chunks are loaded in order: the console holds it whole */
    }
}

/* Whether the console's tile differs from the frame held's. `before` is
 * the frame the console has, or NULL with `blank` for one of nothing but
 * zeros, or NULL alone for one the board does not know: every tile. */
static bool changed(const uint8_t *now, const uint8_t *before, bool blank, unsigned tile)
{
    static const uint8_t zeros[TILE];
    const uint8_t *was = before != NULL ? before + (size_t)tile * TILE : blank ? zeros : NULL;
    return was == NULL || memcmp(now + (size_t)tile * TILE, was, TILE) != 0;
}

static void tiles(nesp_cam_t *cam, const nesp_request_t *request, nesp_reply_t *reply)
{
    if ((request->len != 4 && request->len != 5) || reply->max < TILES_HEAD + ENTRY
        || (request->len == 5 && request->data[4] == 0)) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->once = true;
    if (cam->watching) {
        cam->asked = true;
    }
    uint8_t have = request->data[0];
    uint8_t want = request->data[1];
    if (cam->holding && have == cam->serial) {
        keep(cam);                  /* the console holds it whole */
    }
    reply->data[0] = state(cam);
    reply->data[1] = held_serial(cam);
    reply->len = 2;
    if (!cam->holding || want != cam->serial) {
        return;                     /* not the frame held: the console starts toward it */
    }
    const uint8_t *now = cam->frames[cam->held];
    const uint8_t *before = NULL;   /* the frame the console has, if the board knows it */
    bool blank = false;
    if (!cam->mixed) {
        if (have == 0) {
            blank = true;
        } else if (have == cam->serial) {
            before = now;
        } else if (cam->based && have == cam->base_serial) {
            before = cam->base;
        }
    }
    unsigned limit = request->data[3] == 0 ? TILES : request->data[3];
    unsigned room = (unsigned)(reply->max - TILES_HEAD) / ENTRY;
    if (room > NESP_CAM_TILES_MAX) {
        room = NESP_CAM_TILES_MAX;
    }
    if (request->len == 5 && request->data[4] < room) {
        room = request->data[4];    /* the console's own limit: a reply it can take in a frame */
    }
    unsigned count = 0;
    unsigned tile = request->data[2];
    for (; tile < limit; tile++) {
        if (!changed(now, before, blank, tile)) {
            continue;
        }
        if (count == room) {
            break;                  /* the next to send: NEXT */
        }
        uint8_t *entry = reply->data + TILES_HEAD + count * ENTRY;
        entry[0] = (uint8_t)tile;
        memcpy(entry + 1, now + (size_t)tile * TILE, TILE);
        count++;
    }
    bool done = tile >= limit;
    reply->data[2] = (uint8_t)(done ? limit : tile);    /* 256 is 0 */
    reply->data[3] = done ? 1 : 0;
    reply->data[4] = (uint8_t)count;
    reply->len = (uint8_t)(TILES_HEAD + count * ENTRY);
    cam->chunked = true;
    if (done) {
        keep(cam);                  /* with these the console holds it whole */
    } else {
        cam->partial = true;
    }
}

/* A pixel of the frame held, 0 to 3, by where it is in the 128-pixel square. */
static unsigned shade(const uint8_t *frame, unsigned x, unsigned y)
{
    const uint8_t *tile = frame + (size_t)((y / 8) * SIDE + x / 8) * TILE;
    unsigned bit = 7 - x % 8;
    return ((tile[y % 8] >> bit) & 1u) | (((tile[8 + y % 8] >> bit) & 1u) << 1);
}

/* The frame held at a quarter of the size, into small: each pixel the mean
 * of the four it stands for, rounded, a half up. */
static void shrink(nesp_cam_t *cam)
{
    const uint8_t *frame = cam->frames[cam->held];
    memset(cam->small, 0, SMALL);
    for (unsigned y = 0; y < SMALL_SIDE * 8; y++) {
        for (unsigned x = 0; x < SMALL_SIDE * 8; x++) {
            unsigned sum = shade(frame, 2 * x, 2 * y) + shade(frame, 2 * x + 1, 2 * y)
                           + shade(frame, 2 * x, 2 * y + 1) + shade(frame, 2 * x + 1, 2 * y + 1);
            unsigned value = (sum + 2) / 4;
            uint8_t *tile = cam->small + (size_t)((y / 8) * SMALL_SIDE + x / 8) * TILE;
            uint8_t bit = (uint8_t)(0x80u >> (x % 8));
            if (value & 1u) {
                tile[y % 8] |= bit;
            }
            if (value & 2u) {
                tile[8 + y % 8] |= bit;
            }
        }
    }
    cam->small_made = true;
    cam->shrunk++;
}

static void small(nesp_cam_t *cam, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 2 || request->data[1] >= SMALL_CHUNKS || reply->max < HEAD + CHUNK) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->once = true;
    if (cam->watching) {
        cam->asked = true;
    }
    reply->data[0] = state(cam);
    reply->data[1] = held_serial(cam);
    reply->len = 2;
    if (!cam->holding || request->data[0] != cam->serial) {
        return;                     /* not the frame held: the console starts over */
    }
    if (!cam->small_made) {
        shrink(cam);
    }
    uint8_t index = request->data[1];
    reply->data[2] = index;
    memcpy(reply->data + HEAD, cam->small + (size_t)index * CHUNK, CHUNK);
    reply->len = HEAD + CHUNK;
    cam->chunked = true;
    if (index == SMALL_CHUNKS - 1) {
        cam->finished = true;       /* done with it, though not holding it whole */
    }
}

/* WANT, FROM, COUNT, LIMIT, TAG: COUNT tiles of the frame held from FROM,
 * as a burst on the fast way, and a reply that says which. A WANT that is
 * not the frame held is given that one from its first tile; a FROM at or
 * past LIMIT of the frame held has nothing to send (the console has it
 * whole, and is waiting for the next). Like WATCH, it begins the watching. */
static void turbo(nesp_cam_t *cam, const nesp_request_t *request, nesp_reply_t *reply)
{
    if (request->len != 5 || request->data[2] == 0) {
        reply->status = NESP_ST_BAD_ARG;
        return;
    }
    reply->once = true;
    if (!cam->watching) {
        begin_watching(cam);
    }
    cam->asked = true;
    reply->data[0] = state(cam);
    reply->data[1] = held_serial(cam);
    reply->len = 2;
    if (cam->hal->fast_send == NULL) {
        reply->data[0] = NESP_CAM_NO_PORT;
        return;
    }
    if (!cam->holding) {
        return;
    }
    uint8_t want = request->data[0];
    unsigned from = request->data[1];
    unsigned count = request->data[2];
    unsigned limit = request->data[3] == 0 ? TILES : request->data[3];
    uint8_t tag = request->data[4];
    if (want != cam->serial) {
        from = 0;                   /* the frame held, from its start */
    } else if (from >= limit) {
        keep(cam);                  /* the console holds it whole */
        return;
    }
    if (count > limit - from) {
        count = limit - from;
    }
    const uint8_t tail[2] = {tag, (uint8_t)~tag};
    const uint8_t *now = cam->frames[cam->held] + (size_t)from * TILE;
    if (!cam->hal->fast_send(cam->hal->ctx, now, (size_t)count * TILE, true)
        || !cam->hal->fast_send(cam->hal->ctx, tail, sizeof(tail), false)) {
        reply->data[0] = NESP_CAM_NO_PORT;
        return;
    }
    reply->data[2] = (uint8_t)from;
    reply->data[3] = (uint8_t)count;
    reply->len = TURBO_HEAD;
    cam->chunked = true;
    if (from + count >= limit) {
        keep(cam);                  /* with these the console holds it whole */
    }
}

static void handle(void *ctx, const nesp_request_t *request, nesp_reply_t *reply)
{
    nesp_cam_t *cam = ctx;
    cam->turbo = request->op == NESP_CAM_TURBO;     /* the pace is the last op's */
    switch (request->op) {
    case NESP_CAM_WATCH:
        watch(cam, request, reply);
        break;
    case NESP_CAM_CHUNK:
        chunk(cam, request, reply);
        break;
    case NESP_CAM_TILES:
        tiles(cam, request, reply);
        break;
    case NESP_CAM_SMALL:
        small(cam, request, reply);
        break;
    case NESP_CAM_TURBO:
        turbo(cam, request, reply);
        break;
    default:
        reply->status = NESP_ST_NO_OP;
        break;
    }
}

/* A hello has asked for it, or another ROM is on the console. The frames'
 * memory is kept; the serial goes on from where it was, so a frame is
 * never taken for one from before, and the base is forgotten. */
static void reset(void *ctx)
{
    drop(ctx);
}

void nesp_cam_service(nesp_service_t *service, nesp_cam_t *cam, const nesp_hal_t *hal)
{
    memset(cam, 0, sizeof(*cam));
    cam->hal = hal;
    cam->sink.ctx = cam;
    cam->sink.body = body;
    cam->sink.done = done;
    memset(service, 0, sizeof(*service));
    service->id = NESP_SVC_CAM;
    service->name = "CAM";
    service->activate = reset;
    service->deactivate = reset;
    service->handle = handle;
    service->tick = tick;
    service->ctx = cam;
}
