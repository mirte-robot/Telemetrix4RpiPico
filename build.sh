#!/bin/bash
# de8ae5a
set -ex
git config --global --add safe.directory /__w/Telemetrix4RpiPico/Telemetrix4RpiPico # fix for cmake not reading git stuff
mkdir -p build2/
ls
cd build2
# Sometimes the debug build reports more errors than Release
cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w ..
make -j
cd ..
sha256sum * || true
sha256sum build/* || true

mkdir -p build_pico2/
cd build_pico2/
cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico2 ..
make -j3
cd ..
cat build2/Telemetrix4RpiPico.uf2 build_pico2/Telemetrix4RpiPico.uf2 > build_pico2/Telemetrix4RpiPico_merged.uf2