echo 'Starting all the racing stuff...'

#Start teleop
/home/forzapblcar/catkin_ws/src/f1tenth_system/scripts/f110_start_teleop.sh &


#Sleep to guarantee nodes started
sleep 5 


#Start SLAM
/home/forzapblcar/catkin_ws/src/f1tenth_system/scripts/f110_start_slam.sh &

echo 'Done with racing...'
