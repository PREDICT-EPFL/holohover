#!/bin/bash
# Builds and installs the third-party C++ libraries needed by the holohover packages.
set -euo pipefail

QUILL_VERSION=${QUILL_VERSION:-v4.4.1}
CASADI_VERSION=${CASADI_VERSION:-582be5d9d0e8a9854951e9a11e2b033fa924e1ee}
PIQP_VERSION=${PIQP_VERSION:-a9e2cd91bb378223723cecf4ac8a628ef252fe04}

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

echo "Installing QUILL ${QUILL_VERSION}..."
clone_at https://github.com/odygrd/quill.git "$QUILL_VERSION" quill
build_install quill

echo "Installing CASADI ${CASADI_VERSION}..."
clone_at https://github.com/casadi/casadi.git "$CASADI_VERSION" casadi
build_install casadi

echo "Installing PIQP ${PIQP_VERSION}..."
clone_at https://github.com/PREDICT-EPFL/piqp.git "$PIQP_VERSION" piqp
build_install piqp -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF

ldconfig
