#!/bin/bash
cd /project/src/
ls
ls ..
mkdir build2/
ls
cd build2
cmake -DCMAKE_BUILD_TYPE=Release ..
make