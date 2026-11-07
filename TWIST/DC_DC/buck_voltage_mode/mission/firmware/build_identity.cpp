/*
 * Build identity exposed over ThingSet (group Build in data_objects.yaml).
 *
 * tools/build_firmware.sh writes build_identity.h next to this file in the
 * build copy, from tools/build_identity.py. A build made any other way has
 * no such header and reports "unknown", so it never passes for a known build.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "data_objects.h"

#if __has_include("build_identity.h")
#include "build_identity.h"
#else
#define BUILD_COMMIT "unknown"
#define BUILD_DIRTY true
#define BUILD_SOURCE_HASH "unknown"
#endif

char build_commit[41] = BUILD_COMMIT;
bool build_dirty = BUILD_DIRTY;
char build_source_hash[17] = BUILD_SOURCE_HASH;
