#!/bin/bash

set -xe

mkdir build || true
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
sha256sum Telemetrix4RpiPico.uf2 >Telemetrix4RpiPico.uf2.sha256sum
sha256sum Telemetrix4RpiPico.bin >Telemetrix4RpiPico.bin.sha256sum
picotool info Telemetrix4RpiPico.uf2
sftp mirtefiles@arend-jan.com:/files/telemetrix/release/ <<END
put Telemetrix4RpiPico.uf2
put Telemetrix4RpiPico.uf2.sha256sum
put Telemetrix4RpiPico.bin.sha256sum
END
