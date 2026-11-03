/* A PNG as it arrives, a piece at a time, into rows of pixels (png.c).
 *
 * The image is never held whole, nor is the file: what has come is
 * inflated (miniz's tinfl, third_party/miniz) into a window of 32 KB, and
 * each row is handed on as soon as it is whole. The row before it is kept,
 * which the filters need, and nothing else of the image.
 *
 * Read are 8-bit images of a palette, of RGB and of RGBA, not interlaced,
 * up to NESP_PNG_WIDTH_MAX pixels across. Anything else is an image this
 * cannot read, and is said to be broken. The checksums of the chunks are
 * not checked; zlib's of the image's bytes is.
 */
#ifndef NESP_PNG_H
#define NESP_PNG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "miniz.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NESP_PNG_WIDTH_MAX 256
#define NESP_PNG_ROW_MAX (NESP_PNG_WIDTH_MAX * 4)

/* A row of the image, the first at y 0, as the file has its pixels: a
 * palette's index, or three or four bytes. nesp_png_colour reads one. */
typedef void (*nesp_png_row_t)(void *ctx, uint32_t y, const uint8_t *pixels);

typedef struct {
    tinfl_decompressor inflate;
    uint8_t window[TINFL_LZ_DICT_SIZE];
    size_t window_at;               /* where in the window the inflating writes next */
    uint8_t stage;                  /* which part of the file is coming (png.c) */
    uint32_t left;                  /* bytes still to come of that part */
    uint8_t head[13];               /* the part being gathered: a chunk's length and type, */
    uint8_t head_len;               /* or IHDR */
    uint32_t type;                  /* the chunk's */
    uint32_t size;                  /* and its length */
    bool broken;                    /* not an image this reads, or not a PNG */
    bool inflated;                  /* the image's bytes have ended */
    uint32_t width;
    uint32_t height;
    uint8_t channels;               /* 1 a palette's index, 3 RGB, 4 RGBA */
    uint8_t depth;                  /* bits a sample: 8, or 1, 2 or 4 for a palette's */
    uint16_t colours;               /* in the palette */
    uint8_t palette[256][3];
    uint32_t y;                     /* the row coming */
    size_t row_at;                  /* its bytes so far, its filter's among them */
    uint8_t filter;
    uint8_t rows[2][NESP_PNG_ROW_MAX];
    uint8_t row;                    /* which of the two is the one coming */
    nesp_png_row_t out;
    void *ctx;
} nesp_png_t;

/* A new image; `out` is told each row of it. */
void nesp_png_init(nesp_png_t *png, nesp_png_row_t out, void *ctx);

/* More of the file. False: it is broken, and nothing more is read. */
bool nesp_png_feed(nesp_png_t *png, const uint8_t *data, size_t len);

/* Every row of the image has been handed on. */
bool nesp_png_whole(const nesp_png_t *png);

/* The colour of pixel x of a row as it was handed on. A palette's index
 * that the palette has not is black. */
void nesp_png_colour(const nesp_png_t *png, const uint8_t *pixels, uint32_t x, uint8_t rgb[3]);

#ifdef __cplusplus
}
#endif

#endif
