#!/bin/bash

# MAP_NAME 변수가 설정되어 있지 않으면 종료
if [ -z "$MAP_NAME" ]; then
    echo "MAP_NAME is not set"
    exit 0
fi

echo "MAP_NAME: $MAP_NAME"

current_path=$(pwd)

workspace_path=/home/turtleship/turtleship_ws

# global_waypoints.csv 복사
cp $workspace_path/src/race_stack/stack_master/maps/$MAP_NAME/global_waypoints.csv \
    $workspace_path/src/proj-svg_mppi/src/reference_waypoint_loader/data && \
# 이동
cd $workspace_path/src/proj-svg_mppi/src/reference_waypoint_loader/data && \
# 파일 이름 변경
mv global_waypoints.csv reference_waypoint.csv && \
echo "global_waypoint.csv is set to reference_waypoint.csv"

cd $current_path