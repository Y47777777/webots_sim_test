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
cp *.so* ../../../../../simulation_world/webots/libraries/

cd ..
current_folder_name=$(basename "$(pwd)")
mkdir -p ../../include/"$current_folder_name"
cp *.h ../../include/"$current_folder_name"/