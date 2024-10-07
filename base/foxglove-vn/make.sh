#!/bin/sh

if [ -d './build' ];then 
    echo "rm -r ./build"
    rm -r build
else
    echo "mkdir build"
fi

mkdir build 
cd build
python3 ../build.py
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j12

echo "cp libraries"
cp *.a* ../../../../../sim_module_pkg/libraries/

echo "cp include"
cd ..
cp -r foxglove-vn ../../include
rm ./foxglove-vn/*.cc ./foxglove-vn/*.cs ./foxglove-vn/*.h
