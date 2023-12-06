#!/bin/bash
# cd /project/src/
ls
ls ..
mkdir build2/
ls
cd build2
# Sometimes the debug build reports more errors than Release
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

# Build for the artifacts:
cmake -DCMAKE_BUILD_TYPE=Release ..
make
