#!/bin/bash

set -ex

function build_variant() {
  local build_dir=$1
  local board=$2
  if [ -d "$build_dir" ]; then
    rm -rf "$build_dir"
  fi
  mkdir -p "$build_dir"
  cd "$build_dir"
  cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD="$board" ..
  make -j3
  cd ..
}

variants=("pico" "pico_w" "pico2" "pico2_w")

for variant in "${variants[@]}"; do
  build_variant "build_$variant" "$variant"
done