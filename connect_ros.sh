#!/bin/bash
echo "🔗 快速连接ROS机器并运行模拟器"
ssh root@192.168.203.30 << 'ENDSSH'
source /home/ren/catkin_ws/devel/setup.bash
mkdir -p /tmp/public
cd /tmp
python3 ros_mission_simulator.py
ENDSSH
