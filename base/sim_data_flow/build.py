import os

os.chdir("../")
os.system("ls")
# cmd="protoc --proto_path=./ --proto_path=./sim_data_flow --proto_path=./foxglove-vn/foxglove-vn/  --cpp_out=./sim_data_flow/build ./sim_data_flow/*.proto"
cmd="protoc --proto_path=./sim_data_flow --proto_path=./foxglove-vn/foxglove-vn --cpp_out=./sim_data_flow/build ./sim_data_flow/*.proto"
os.system(cmd)




