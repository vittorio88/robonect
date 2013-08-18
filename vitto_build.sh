#!/bin/sh

#### SOURCE ME. DO NOT ./ !!!!

mkdir build
cd build
cmake ../src -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}  -G"Eclipse CDT4 - Unix Makefiles"
make install
cd ..

source ${CMAKE_INSTALL_PREFIX}/setup.bash
