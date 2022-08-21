#!/bin/bash

set -e

docker run --rm -v $(pwd)/../:/src -u $(id -u):$(id -g) emscripten/emsdk \
    emcc -I /src/main/compression /src/host/wasm.cpp --no-entry \
    -sDISABLE_EXCEPTION_CATCHING=2 -sDISABLE_EXCEPTION_THROWING=0 \
    -sINITIAL_MEMORY=64kB -s TOTAL_STACK=16kB -sALLOW_MEMORY_GROWTH -sMAXIMUM_MEMORY=256MB \
    -fno-exceptions -Oz  -o /src/host/decoder.wasm


base64 decoder.wasm  | sed 's/^\(.*\)$/"\1",/g' > decoder.wasm.base64