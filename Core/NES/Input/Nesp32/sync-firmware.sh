#!/bin/sh
# Copy the NESP32 firmware's portable part (its link and services, the same
# code the real board runs) from a checkout of github.com/Armitage64/nesp32.
#   Core/NES/Input/Nesp32/sync-firmware.sh /path/to/nesp32
# The nesp32 checkout must have run `make proto` first (it generates nesp_proto.h).
set -e
SRC=${1:?usage: sync-firmware.sh /path/to/nesp32}
DST=$(dirname "$0")/firmware
mkdir -p "$DST"
rm -f "$DST"/*.c "$DST"/*.h
cp "$SRC"/firmware/components/nesp_core/nesp_core.c "$DST"/
cp "$SRC"/firmware/components/nesp_core/include/*.h "$DST"/
cp "$SRC"/firmware/components/nesp_services/*.c "$DST"/
cp "$SRC"/firmware/components/nesp_services/*.h "$DST"/ 2>/dev/null || true
cp "$SRC"/firmware/components/nesp_services/include/*.h "$DST"/
cp "$SRC"/build/nesp_proto.h "$DST"/
cp "$SRC"/third_party/minimp3/minimp3.h "$DST"/
cp "$SRC"/third_party/minimp3/LICENSE "$DST"/LICENSE.minimp3
cp "$SRC"/LICENSE "$DST"/LICENSE.nesp32
( cd "$SRC" && git rev-parse HEAD ) > "$DST"/NESP32_COMMIT
echo "synced from $(cat "$DST"/NESP32_COMMIT)"
