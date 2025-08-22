#!/bin/bash

mkdir build
cd build
cmake ..
make
./strapdown ../tests/simple_strapdown_reference.csv 