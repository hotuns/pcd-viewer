启动ros的流程：
连接
ssh root@192.168.203.30
gZg!p95L

加载
source /home/ren/catkin_ws/devel/setup.bash

启动ros
roslaunch ego_planner run_in_sim.launch

启动ws
roslaunch rosbridge_server rosbridge_websocket.launch

