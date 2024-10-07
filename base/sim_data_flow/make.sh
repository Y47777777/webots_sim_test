#!/bin/sh

if [ -d './build' ];then 
    echo "rm -r ./build"
    rm -r build
else
    echo "mkdir build"
fi

# mkdir build
# cd ../
# protoc --proto_path=./ --proto_path=./sim_data_flow --proto_path=./foxglove-vn/foxglove-vn/  --cpp_out=./sim_data_flow/build ./sim_data_flow/*.proto

# cd sim_data_flow
# cp -r  build/sim_data_flow ../../include/

mkdir build 
python3 "build.py"

cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make 

echo "cp libraries"
cp *.a* ../../../../../sim_mode_pkg/libraries/

echo "cp include"
rm -rf ../../../include/sim_data_flow/
mkdir -p ../../../include/sim_data_flow/

cp *.h ../../../include/sim_data_flow/
cp *.cc ../../../include/sim_data_flow/

