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




roscd ego_planner/launch
roscd rosbridge_server/launch

---

## 部署并使用 mission_msgs

1. **上传消息包**
   ```bash
   # 在本地
   cd /home/hotuns/pcd-viewer
   scp -r mission_msgs root@192.168.203.30:/home/ren/catkin_ws/src/

   # 上传脚本
   scp ./mission_msgs_simulator.py root@192.168.203.30:/home/ren/catkin_ws/src/pcd-viewer/
   ```
   如需打包可以 `tar czf mission_msgs.tar.gz mission_msgs`，上传后在远端 `catkin_ws/src` 解压。

2. **编译工作空间**
   ```bash
   ssh root@192.168.203.30
   cd /home/ren/catkin_ws
   catkin_make
   ```
   编译成功后会生成对应的 `lib` 与 `devel/include`。

3. **加载环境**
   ```bash
   source /home/ren/catkin_ws/devel/setup.bash
   ```
   确保新终端都执行一次，`rosmsg list | grep mission_msgs` 可验证生效。

4. **在 ROS 中使用**
   - 发布 MissionList：
     ```bash
     rostopic pub /mission/list mission_msgs/MissionList '{id: 0, HomePos: {header:{stamp:{secs:0,nsecs:0},frame_id:"map"}, pose:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}}, PosNum:1, PosList:[{x:0,y:0,z:1,pass_type:false,task_type:"0",info:"demo"}]}'
     ```
   - 其他消息（MissionStatus、HangarChargeStatus、Control 等）用法类似，按照 Missionlogic 的话题发布/订阅即可。

5. **更新包**
   - 有更新时重复上传并 `catkin_make`，或直接在服务器中 `git pull` 后重编译。



## py模拟脚本
```
scp ./mission_msgs_simulator.py root@192.168.203.30:/home/ren/catkin_ws/src/pcd-viewer/
cd /home/ren/catkin_ws/src/pcd-viewer
source /home/ren/catkin_ws/devel/setup.bash
chmod +x mission_msgs_simulator.py      # 首次运行需要
python3 mission_msgs_simulator.py

```
