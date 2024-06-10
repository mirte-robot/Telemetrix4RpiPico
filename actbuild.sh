#!/bin/bash

set -xe 
REBUILD=true
rm -rf /tmp/artifacts || true
mkdir /tmp/artifacts || true
$REBUILD && act --artifact-server-path /tmp/artifacts -j build-deploy || true
mkdir artifacts || true

$REBUILD && cp -r /tmp/artifacts/* ./artifacts
# somehow the files get an extra __ at the end
find ./artifacts -name "*.gz__" -exec sh -c 'cp "$1" "${1%.gz__}.gz"' _ {} \;
find ./artifacts -name "*.gz" -exec sh -c 'gunzip -f "$1"' _ {} \;
find ./artifacts -name "*.uf2" -exec sh -c 'sha256sum "$1"' _ {} \;

IP=192.168.1.215
scp artifacts/1/dist/Telemetrix4RpiPico.uf2 mirte@$IP:/home/mirte/mirte_ws/ && ssh mirte@$IP -t "cd ~/mirte_ws/ && sudo picotool load -f Telemetrix4RpiPico.uf2"