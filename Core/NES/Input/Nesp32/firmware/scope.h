/* What RADAR (radar.c) and HARBOR (harbor.c) share: nes-radar's scope and
 * its records, and the reading of one object of a reply. Not for anything
 * else: the services' own interface is nesp_services.h. */
#ifndef NESP_SCOPE_H
#define NESP_SCOPE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nesp_services.h"

#define TARGETS NESP_RADAR_TARGETS_MAX
#define MOTION NESP_RADAR_MOTION_LEN
#define NAMES NESP_RADAR_NAMES_LEN

/* The scope is 144 points across, its middle at (72, 72) and its edge 71
 * points from it. */
#define MIDDLE 72
#define EDGE 71.0
#define LAST 143
#define PI 3.14159265358979323846
#define EARTH_NM (6371.0 * 0.539957)

/* A motion record's first byte, with its slot. */
#define NO_TRACK 0x08
#define NO_ALTITUDE 0x10
#define NO_SPEED 0x20
#define NO_RATE 0x80                /* in place of a rate of climb */

/* The value of one of an object's names: its characters, less the quotes
 * of a string. */
typedef struct {
    const char *at;
    size_t len;
    bool string;
} scope_value_t;

/* What an object has by this name, at its top. False if it has none, or
 * null. */
bool scope_member(const char *item, size_t len, const char *name, scope_value_t *out);
/* A number, or a string that is one. */
bool scope_number(const char *item, size_t len, const char *name, double *out);
/* A value as a name on the scope: capitals, digits, spaces and hyphens, cut
 * to the width and padded with spaces. */
void scope_put_name(const scope_value_t *value, uint8_t *out, size_t width);
double scope_radians(double degrees);
long scope_rounded(double value, long least, long most);
/* A place put on the scope among the nearest (radar.c). */
nesp_radar_target_t *scope_spot(nesp_radar_t *radar, double latitude, double longitude,
                                double range_nm);

/* HARBOR's own, which radar.c calls on its behalf (harbor.c). */
void harbor_header(nesp_radar_t *radar, const char *item, size_t len);
void harbor_vessel(nesp_radar_t *radar, const char *item, size_t len);
void harbor_zip(nesp_radar_t *radar, const char *item, size_t len);
bool harbor_key(nesp_radar_t *radar);
void harbor_url(const nesp_radar_t *radar, char *url, size_t size);
/* Makes radar.c's service, as RADAR or as HARBOR (radar.c). */
void scope_service(nesp_service_t *service, nesp_radar_t *radar, const nesp_hal_t *hal,
                   bool ships);

#endif
