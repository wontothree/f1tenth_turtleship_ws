#!/bin/bash

# $1 변수에 아무 값이 없으면 MAP_NAME 변수를 사용한다.
if [ -z "$1" ]; then
    map_name=$MAP_NAME
else
    map_name=$1
fi

echo "MAP_NAME: $map_name"
roslaunch stack_master base_system.launch map_name:=$map_name racecar_version:=NUC2