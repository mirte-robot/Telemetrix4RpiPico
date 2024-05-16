#!/bin/bash

set -xe

mkdir build || true
cd build
make
picotool info Telemetrix4RpiPico.uf2
sftp mirtefiles@arend-jan.com:/files/telemetrix/release/ <<< $'put Telemetrix4RpiPico.uf2'