#!/bin/bash
set -e
mkdir build2/
ls
cd build2
# Sometimes the debug build reports more errors than Release
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
cd ..
rm -rf build2
mkdir build2
cd build2
# Build for the artifacts:
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sha256sum * || true