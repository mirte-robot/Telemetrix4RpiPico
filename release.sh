#!/bin/bash
set -xe
mkdir build_rel || true
cd build_rel
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j
sha256sum * || true
sha256sum Telemetrix4RpiPico.bin > Telemetrix4RpiPico.bin.sha256sum
sha256sum Telemetrix4RpiPico.uf2 > Telemetrix4RpiPico.uf2.sha256sum

sftp mirtefiles@mirte.arend-jan.com <<END
cd files/telemetrix/release
rm *
put Telemetrix4RpiPico.bin.sha256sum
put Telemetrix4RpiPico.uf2
put Telemetrix4RpiPico.uf2.sha256sum
bye
END