#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "$0 - Wrong number of parameters"
    echo "Usage:"
    echo "   $0 [amd64/x86]"
    exit -1
fi

ARCH=$1

for subdir in */; do
  if [ -d "$subdir" ] && [ "$(basename "$subdir")" != "matlab" ]; then
    echo "Entering directory: $subdir"
    cd $subdir
    if [ ! -f locFuns-$ARCH.so ]; then
      gcc -fPIC -shared -O3 locFuns.c -o locFuns-$ARCH.so
    fi
    cd -
  fi
done
