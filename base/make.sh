#!/bin/sh
cd logvn
./make.sh
cd ..

cd foxglove-vn
./make.sh
cd ..

cd actionservice
./make.sh
cd ..

cd sim_data_flow
./make.sh
cd ..

cd keyboardform
./make.sh
cd ..
