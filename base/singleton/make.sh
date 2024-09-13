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
make -j12

echo "cp libraries"
#cp *.so* ../../../../../simulation_world/webots/libraries/
cp *.a* ../../../../../simulation_world/webots/libraries/


echo "cp include"
cd ..
current_folder_name=$(basename "$(pwd)")
cp -r include/"$current_folder_name" ../../include
#mkdir -p ../../include/"$current_folder_name"
#cp include/Parser/*.hpp ../../include/"$current_folder_name"/
