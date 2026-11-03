/* A PNG as it arrives into rows (nesp_png.h). This is the one file that
 * holds miniz's inflate (third_party/miniz).
 *
 * A PNG is eight bytes of signature and then chunks: a length, four bytes
 * high first; a type of four letters; that many bytes; and a checksum. The
 * first is IHDR, what the image is. PLTE is the palette, IDAT the image's
 * bytes, deflated as one zlib stream however many IDATs it is cut into,
 * and IEND the end. The rest are passed over.
 *
 * Each row of the image's bytes is a filter's number and then the row,
 * filtered: each byte less what is before it (1), or above it (2), or the
 * two's mean (3), or Paeth's guess from the three (4).
 */
#include <string.h>

#include "nesp_png.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#include "miniz_tinfl.c"
#pragma GCC diagnostic pop

/* Which part of the file is coming. */
#define SIGNATURE 0
#define HEAD 1                      /* a chunk's length and type */
#define BODY 2
#define CHECK 3                     /* its checksum */
#define END 4                       /* after IEND: nothing more is read */

#define TYPE(a, b, c, d) ((uint32_t)(a) << 24 | (uint32_t)(b) << 16 | (uint32_t)(c) << 8 | (uint32_t)(d))
#define IHDR TYPE('I', 'H', 'D', 'R')
#define PLTE TYPE('P', 'L', 'T', 'E')
#define IDAT TYPE('I', 'D', 'A', 'T')
#define IEND TYPE('I', 'E', 'N', 'D')

static const uint8_t signature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};

static uint32_t be32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 | (uint32_t)bytes[2] << 8
           | (uint32_t)bytes[3];
}

void nesp_png_init(nesp_png_t *png, nesp_png_row_t out, void *ctx)
{
    tinfl_init(&png->inflate);
    png->window_at = 0;
    png->stage = SIGNATURE;
    png->left = sizeof(signature);
    png->head_len = 0;
    png->type = 0;
    png->size = 0;
    png->broken = false;
    png->inflated = false;
    png->width = 0;
    png->height = 0;
    png->channels = 0;
    png->depth = 0;
    png->colours = 0;
    png->y = 0;
    png->row_at = 0;
    png->filter = 0;
    png->row = 0;
    png->out = out;
    png->ctx = ctx;
}

bool nesp_png_whole(const nesp_png_t *png)
{
    return !png->broken && png->height != 0 && png->y == png->height;
}

void nesp_png_colour(const nesp_png_t *png, const uint8_t *pixels, uint32_t x, uint8_t rgb[3])
{
    if (png->channels == 1) {
        uint8_t index;
        if (png->depth == 8) {
            index = pixels[x];
        } else {                    /* packed, the leftmost in the high bits */
            uint32_t bit = x * png->depth;
            index = (uint8_t)((pixels[bit / 8] >> (8 - png->depth - bit % 8)) & ((1u << png->depth) - 1));
        }
        if (index < png->colours) {
            memcpy(rgb, png->palette[index], 3);
        } else {
            memset(rgb, 0, 3);
        }
    } else {
        memcpy(rgb, pixels + (size_t)x * png->channels, 3);
    }
}

/* What IHDR says: whether it is an image this reads. */
static bool header(nesp_png_t *png)
{
    const uint8_t *head = png->head;
    png->width = be32(head);
    png->height = be32(head + 4);
    uint8_t depth = head[8], colour = head[9];
    png->channels = colour == 3 ? 1 : colour == 2 ? 3 : colour == 6 ? 4 : 0;
    png->depth = depth;
    bool packed = colour == 3 && (depth == 1 || depth == 2 || depth == 4);
    if (png->width == 0 || png->width > NESP_PNG_WIDTH_MAX || png->height == 0
        || png->height > 0x7FFFFFFF || (depth != 8 && !packed) || png->channels == 0
        || head[10] != 0 || head[11] != 0 || head[12] != 0) {
        return false;               /* not 8 bits (or fewer, for a palette), a grey,
                                     * interlaced, or of no known kind */
    }
    memset(png->rows, 0, sizeof(png->rows));    /* the row above the first is zeros */
    return true;
}

/* A row's bytes, less its filter's. */
static size_t row_length(const nesp_png_t *png)
{
    return ((size_t)png->width * png->channels * png->depth + 7) / 8;
}

static uint8_t paeth(uint8_t left, uint8_t above, uint8_t corner)
{
    int guess = (int)left + (int)above - (int)corner;
    int to_left = guess > left ? guess - left : left - guess;
    int to_above = guess > above ? guess - above : above - guess;
    int to_corner = guess > corner ? guess - corner : corner - guess;
    if (to_left <= to_above && to_left <= to_corner) {
        return left;
    }
    return to_above <= to_corner ? above : corner;
}

/* The row that has just come whole, unfiltered, and handed on. */
static void row_done(nesp_png_t *png)
{
    uint8_t *row = png->rows[png->row];
    const uint8_t *above = png->rows[png->row ^ 1];
    size_t length = row_length(png);
    size_t step = png->depth == 8 ? png->channels : 1;     /* a packed row's: a byte */
    for (size_t at = 0; at < length; at++) {
        uint8_t left = at >= step ? row[at - step] : 0;
        uint8_t corner = at >= step ? above[at - step] : 0;
        switch (png->filter) {
        case 1: row[at] = (uint8_t)(row[at] + left); break;
        case 2: row[at] = (uint8_t)(row[at] + above[at]); break;
        case 3: row[at] = (uint8_t)(row[at] + (uint8_t)(((unsigned)left + above[at]) / 2)); break;
        case 4: row[at] = (uint8_t)(row[at] + paeth(left, above[at], corner)); break;
        default: break;
        }
    }
    png->out(png->ctx, png->y, row);
    png->row ^= 1;
    png->y++;
    png->row_at = 0;
}

/* The image's bytes, inflated, as they come: into rows. What comes after
 * the last row is passed over. */
static bool unpack(nesp_png_t *png, const uint8_t *data, size_t len)
{
    size_t length = row_length(png);
    while (len > 0 && png->y < png->height) {
        if (png->row_at == 0) {
            png->filter = *data++;
            len--;
            if (png->filter > 4) {
                return false;
            }
            png->row_at = 1;
            continue;
        }
        size_t room = length - (png->row_at - 1);
        size_t taken = len < room ? len : room;
        memcpy(png->rows[png->row] + png->row_at - 1, data, taken);
        png->row_at += taken;
        data += taken;
        len -= taken;
        if (png->row_at - 1 == length) {
            row_done(png);
        }
    }
    return true;
}

/* An IDAT's bytes: inflated into the window, and from there into rows. */
static bool inflate(nesp_png_t *png, const uint8_t *data, size_t len)
{
    while (!png->inflated) {
        size_t taken = len;
        size_t given = sizeof(png->window) - png->window_at;
        tinfl_status status = tinfl_decompress(&png->inflate, data, &taken, png->window,
                                               png->window + png->window_at, &given,
                                               TINFL_FLAG_PARSE_ZLIB_HEADER
                                                   | TINFL_FLAG_HAS_MORE_INPUT);
        data += taken;
        len -= taken;
        if (given > 0 && !unpack(png, png->window + png->window_at, given)) {
            return false;
        }
        png->window_at = (png->window_at + given) & (sizeof(png->window) - 1);
        if (status < TINFL_STATUS_DONE) {
            return false;
        }
        if (status == TINFL_STATUS_DONE) {
            png->inflated = true;
        } else if (status == TINFL_STATUS_NEEDS_MORE_INPUT || (taken == 0 && given == 0)) {
            break;                  /* until the next of the file */
        }
    }
    return true;
}

/* As much of a chunk's body as has come. */
static bool body(nesp_png_t *png, const uint8_t *data, size_t len)
{
    uint32_t at = png->size - png->left;   /* how far into the body this is */
    switch (png->type) {
    case IHDR:
        memcpy(png->head + at, data, len);
        if (at + len == png->size) {
            return header(png);
        }
        return true;
    case PLTE:
        for (size_t index = 0; index < len; index++) {
            png->palette[(at + index) / 3][(at + index) % 3] = data[index];
        }
        png->colours = (uint16_t)((at + len) / 3);
        return true;
    case IDAT:
        return inflate(png, data, len);
    default:
        return true;                /* passed over */
    }
}

/* A chunk's length and type have come: whether it is one that may be
 * here. */
static bool chunk(nesp_png_t *png)
{
    png->size = be32(png->head);
    png->type = be32(png->head + 4);
    if (png->size > 0x7FFFFFFF) {
        return false;
    }
    bool headed = png->width != 0;
    switch (png->type) {
    case IHDR:
        return !headed && png->size == 13;
    case PLTE:
        return headed && png->size % 3 == 0 && png->size <= sizeof(png->palette)
               && png->colours == 0;
    case IDAT:
        return headed && (png->channels != 1 || png->colours != 0);
    default:
        return headed;              /* IHDR is first */
    }
}

bool nesp_png_feed(nesp_png_t *png, const uint8_t *data, size_t len)
{
    while (len > 0 && !png->broken && png->stage != END) {
        size_t taken = len < png->left ? len : png->left;
        switch (png->stage) {
        case SIGNATURE:
            if (memcmp(data, signature + sizeof(signature) - png->left, taken) != 0) {
                png->broken = true;
            }
            break;
        case HEAD:
            memcpy(png->head + png->head_len, data, taken);
            png->head_len = (uint8_t)(png->head_len + taken);
            break;
        case BODY:
            png->broken = !body(png, data, taken);
            break;
        default:
            break;                  /* a checksum */
        }
        data += taken;
        len -= taken;
        png->left -= (uint32_t)taken;
        if (png->left > 0 || png->broken) {
            continue;
        }
        switch (png->stage) {
        case HEAD:
            png->broken = !chunk(png);
            png->stage = BODY;
            png->left = png->size;
            if (png->left == 0) {
                png->stage = CHECK;     /* a body of nothing */
                png->left = 4;
            }
            break;
        case BODY:
            png->stage = CHECK;
            png->left = 4;
            break;
        case CHECK:
            png->stage = png->type == IEND ? END : HEAD;
            png->left = 8;
            png->head_len = 0;
            break;
        default:                    /* the signature */
            png->stage = HEAD;
            png->left = 8;
            png->head_len = 0;
            break;
        }
    }
    return !png->broken;
}
