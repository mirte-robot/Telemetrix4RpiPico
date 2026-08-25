#!/bin/bash

set -ex
./build_variants.sh

# use w variants to merge so they'll have the led pin fixes
cat build_pico_w/Telemetrix4RpiPico.uf2 build_pico2_w/Telemetrix4RpiPico.uf2 > build_pico2_w/Telemetrix4RpiPico_merged.uf2