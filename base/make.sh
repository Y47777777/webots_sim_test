#!/bin/sh
# 不能用循坏，要确定顺序

SIMULATION_LIB_DIR=../../../sim_mode_pkg/libraries
if [ ! -d ${SIMULATION_LIB_DIR} ]; then
	echo "create SIMULATION_LIB_DIR=${SIMULATION_LIB_DIR}"
	mkdir -p ${SIMULATION_LIB_DIR}
fi
cd logvn
./make.sh
cd ..

cd foxglove-vn
./make.sh
cd ..

cd geometry
./make.sh
cd ..

#cd actionservice
#./make.sh
#cd ..

cd sim_data_flow
./make.sh
cd ..

cd keyboardform
./make.sh
cd ..

cd Parser
./make.sh
cd ..

cd singleton
./make.sh
cd ..
