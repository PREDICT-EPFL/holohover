#!/bin/bash
# Builds and installs the third-party C++ libraries needed by the holohover packages.
set -euo pipefail

# Each library is installed only if its *_VERSION (tag or commit) is set and non-empty.
QUILL_VERSION=${QUILL_VERSION:-}
CASADI_VERSION=${CASADI_VERSION:-}
PIQP_VERSION=${PIQP_VERSION:-}

# clone_at <url> <ref> <dir>: shallow checkout of a tag or commit
clone_at() {
    git init -q "$3"
    git -C "$3" remote add origin "$1"
    git -C "$3" fetch -q --depth 1 origin "$2"
    git -C "$3" checkout -q FETCH_HEAD
}

# build_install <dir> [cmake args...]
build_install() {
    local dir=$1; shift
    cmake -S "$dir" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release "$@"
    cmake --build "$dir/build" -j"$(nproc)"
    cmake --install "$dir/build"
    rm -rf "$dir"
}

cd /tmp

if [ -n "$QUILL_VERSION" ]; then
    echo "Installing QUILL ${QUILL_VERSION}..."
    clone_at https://github.com/odygrd/quill.git "$QUILL_VERSION" quill
    build_install quill
else
    echo "Skipping QUILL (QUILL_VERSION not set)"
fi

if [ -n "$CASADI_VERSION" ]; then
    echo "Installing CASADI ${CASADI_VERSION}..."
    clone_at https://github.com/casadi/casadi.git "$CASADI_VERSION" casadi
    build_install casadi
else
    echo "Skipping CASADI (CASADI_VERSION not set)"
fi

if [ -n "$PIQP_VERSION" ]; then
    echo "Installing PIQP ${PIQP_VERSION}..."
    clone_at https://github.com/PREDICT-EPFL/piqp.git "$PIQP_VERSION" piqp
    build_install piqp -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF
else
    echo "Skipping PIQP (PIQP_VERSION not set)"
fi

ldconfig
