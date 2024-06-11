#!/bin/sh

if [ -d './build' ];then 
    echo "rm -r ./build"
    rm -r build
else
    echo "mkdir build"
fi

mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
cp *.a* ../../../../../simulation_world/webots/libraries/
cd ..
cp -r foxglove-vn ../../include
rm ./foxglove-vn/*.cc ./foxglove-vn/*.cs ./foxglove-vn/*.h
