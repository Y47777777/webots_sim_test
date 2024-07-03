import os

os.chdir("../foxglove-vn")
cmd="protoc -I=. --cpp_out=. ./*.proto"
os.system(cmd)
cmd="protoc -I=. --csharp_out=. ./*.proto"
os.system(cmd)



