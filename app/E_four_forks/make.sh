#!/bin/sh

if [ -d './build' ];then 
    echo "rm -r ./build"
    rm -r build
else
    echo "mkdir build"
fi

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make -j12

make install
