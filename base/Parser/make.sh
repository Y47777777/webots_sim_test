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
cp *.so* ../../../../../sim_module_pkg/libraries/
cp *.a* ../../../../../sim_module_pkg/libraries/


echo "cp include"
cd ..
current_folder_name=$(basename "$(pwd)")
mkdir -p ../../include/"$current_folder_name"
cp include/Parser/*.hpp ../../include/"$current_folder_name"/
