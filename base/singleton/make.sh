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
#cp *.so* ../../../../../sim_mode_pkg/libraries/
cp *.a* ../../../../../sim_mode_pkg/libraries/


echo "cp include"
cd ..
current_folder_name=$(basename "$(pwd)")
cp -r include/"$current_folder_name" ../../include
#mkdir -p ../../include/"$current_folder_name"
#cp include/Parser/*.hpp ../../include/"$current_folder_name"/
