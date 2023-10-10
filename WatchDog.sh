#!/bin/bash

# 程序名称
program_name=autoaim
# 程序所在路径
program_path=/home/promise/RM_cpp/cmake-build-debug/


times=0
mkdir -p /home/promise/RM_cpp/cmake-build-debug
cd $program_path || exit
cmake .. -G Ninja
ninja

while true
do
    count=$(pgrep -fc $program_name)
    if [ "$count" -ge 1 ];then
        echo "$program_name 正在运行!"
        echo "-----------------------"
        sleep 2
    else
        echo "$program_name 未运行!"
        echo "正在运行$program_name..."
        gnome-terminal -- bash -c "cd $program_path;./$program_name;exec bash;"
        echo "$program_name 已启动!"
        echo "-----------------------"
        ((times=times+1))
        sleep 2
    fi
    echo "$program_name 已启动次数: $times"
done