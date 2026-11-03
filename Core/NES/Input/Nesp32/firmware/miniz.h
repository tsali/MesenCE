/* nesp32's miniz.h: miniz's inflate and nothing else.
 *
 * miniz_common.h, miniz_tinfl.h and miniz_tinfl.c are miniz 3.0.2's
 * (https://github.com/richgel999/miniz, tag 3.0.2), unchanged, under its
 * LICENSE. This file and miniz_export.h stand for miniz's own miniz.h and
 * the miniz_export.h its CMake writes: they set what miniz.h would for a
 * 32-bit processor with no file system, no clock and no malloc, and say
 * nothing of deflate or zip. png.c (firmware/components/nesp_services)
 * is the one file that holds the code.
 */
#pragma once

/* The code is png.c's, which includes nesp_png.h and then miniz_tinfl.c.
 * Compiled on its own, as a build that compiles every .c it finds does
 * (MesenCE's), miniz_tinfl.c is nothing, so there are not two of it. */
#ifndef NESP_PNG_H
#define MINIZ_NO_INFLATE_APIS
#endif

#define MINIZ_NO_STDIO
#define MINIZ_NO_TIME
#define MINIZ_NO_MALLOC
#define MINIZ_NO_DEFLATE_APIS
#define MINIZ_NO_ARCHIVE_APIS
#define MINIZ_NO_ZLIB_APIS
#define MINIZ_USE_UNALIGNED_LOADS_AND_STORES 0
#define MINIZ_LITTLE_ENDIAN 0
#define MINIZ_HAS_64BIT_REGISTERS 0

#include "miniz_common.h"
#include "miniz_tinfl.h"
