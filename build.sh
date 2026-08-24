#!/bin/bash
# de8ae5a
set -e
git config --global --add safe.directory /__w/Telemetrix4RpiPico/Telemetrix4RpiPico # fix for cmake not reading git stuff
mkdir build2/
ls
cd build2
# Sometimes the debug build reports more errors than Release
cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w ..
make -j
cd ..
sha256sum * || true
sha256sum build/* || true
