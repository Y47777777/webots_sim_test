#!/bin/bash

# 获取当前目录下所有子目录的列表
directories=$(find . -maxdepth 1 -type d)

# 遍历每个子目录
for dir in $directories; do
    # 跳过当前目录
    if [[ "$dir" == "." ]]; then
        continue
    fi

    if [[ -d "$dir" && -f "$dir/make.sh" ]]; then
        echo "Entering directory: $dir"
        (cd "$dir" && ./make.sh)
        echo "Exiting directory: $dir"
    fi
done
