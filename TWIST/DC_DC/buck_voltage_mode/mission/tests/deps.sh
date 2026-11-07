#!/usr/bin/env bash
# Shared by the test scripts: locate or fetch the pinned control_library.
#
# CONTROL_LIBRARY_DIR overrides the location (a checkout of
# owntech-foundation/control_library). Otherwise the pinned commit is
# cloned once into ${OWNSTUDIO_CACHE:-$HOME/.cache/ownstudio}.
CONTROL_LIBRARY_URL=https://github.com/owntech-foundation/control_library.git
CONTROL_LIBRARY_COMMIT=1ef80979d3181035c7dcc7a4f1c6218f297db7ca

control_library_dir() {
    if [ -n "${CONTROL_LIBRARY_DIR:-}" ]; then
        echo "$CONTROL_LIBRARY_DIR"; return
    fi
    local dir="${OWNSTUDIO_CACHE:-$HOME/.cache/ownstudio}/control_library-${CONTROL_LIBRARY_COMMIT:0:7}"
    if [ ! -d "$dir/src" ]; then
        git clone -q "$CONTROL_LIBRARY_URL" "$dir" >&2
        git -C "$dir" checkout -q "$CONTROL_LIBRARY_COMMIT" >&2
    fi
    echo "$dir"
}
