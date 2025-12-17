# ROS任务模拟测试指南

## 🎯 测试目标
验证PCD Viewer的完整任务执行流程，包括：
- 无人机3D模型位置实时更新
- 航点到达状态变化  
- 任务完成消息处理

## 📋 测试环境准备

### 1. ROS机器端 (192.168.203.30)

```bash
# 终端1: SSH连接
ssh root@192.168.203.30
# 密码: gZg!p95L

# 加载ROS环境
source /home/ren/catkin_ws/devel/setup.bash

# 启动ego_planner仿真
roslaunch ego_planner run_in_sim.launch
```

```bash
# 终端2: 新的SSH会话
ssh root@192.168.203.30

# 加载ROS环境
source /home/ren/catkin_ws/devel/setup.bash

# 启动WebSocket桥接服务
roslaunch rosbridge_server rosbridge_websocket.launch
```

```bash
# 终端3: 运行任务模拟器
ssh root@192.168.203.30

# 加载ROS环境
source /home/ren/catkin_ws/devel/setup.bash

# 切换到模拟器目录
cd /tmp

# 运行模拟器(需要先部署脚本文件)
python3 ros_mission_simulator.py
```

### 2. 本地端 (PCD Viewer)

```bash
# 启动PCD Viewer开发服务器
cd /home/hotuns/pcd-viewer
pnpm dev
```

## 🚀 完整测试流程

### 步骤1: 部署模拟器脚本
```bash
# 在PCD Viewer项目目录下运行
./deploy_ros_simulator.sh
```

### 步骤2: 启动ROS服务
按照上面的ROS机器端准备步骤，依次启动：
1. ego_planner仿真环境
2. rosbridge_websocket服务
3. mission_simulator节点

### 步骤3: 测试PCD Viewer连接
1. 打开浏览器访问 `http://localhost:3000`
2. 在启动页选择"演示飞行任务"或创建新任务
3. 进入主界面后，点击"连接ROS"
4. 输入WebSocket地址：`ws://192.168.203.30:9999`
5. 确认连接成功（状态显示为已连接）

### 步骤4: 开始任务测试
1. 确保任务状态为"ready"（就绪）
2. 点击"开始任务"按钮
3. 观察任务状态变为"running"（执行中）
4. 在ROS机器端启动模拟器开始发布数据

## 🔍 预期测试结果

### 实时位置更新
- **现象**: 3D场景中的无人机模型实时移动
- **路径**: (0,0,1) → (5,0,1) → (5,5,1) → (0,5,1) → (0,0,1)
- **速度**: 约1m/s，每个航点间停留1秒

### 航点状态变化
- **初始状态**: 所有航点为"pending"（灰色）
- **当前航点**: "active"（橙色+发光效果）
- **到达航点**: "completed"（绿色）
- **状态顺序**: pending → active → completed

### 任务状态流转
- **开始**: running（执行中）
- **结束**: completed（完成）
- **左侧面板**: 显示任务完成信息

### ROS消息验证
模拟器会发布以下消息：
```json
# 航点到达消息 (/mission/waypoint_reached)
{
  "waypoint_index": 0,
  "timestamp": 1730000000,
  "position": {"x": 0.0, "y": 0.0, "z": 1.0}
}

# 任务完成消息 (/mission/complete)
{
  "mission_id": "test_mission_001",
  "status": "completed", 
  "timestamp": 1730000000,
  "reason": "All waypoints reached successfully"
}
```

## 🛠️ 故障排除

### 连接问题
- 检查rosbridge_websocket是否在端口9999正常运行
- 确认防火墙/网络可达性
- 验证ROS_MASTER_URI设置

### 数据问题
- 使用`rostopic list`查看可用话题
- 使用`rostopic echo /odom_visualization/pose`验证位置数据
- 检查模拟器日志输出

### 界面问题
- 检查浏览器开发者工具的WebSocket连接状态
- 确认任务状态正确初始化
- 验证航点数据加载

## 📊 性能指标

- **位置更新频率**: 10Hz
- **任务执行时间**: 约20-25秒
- **航点数量**: 5个
- **总距离**: 约15米
- **WebSocket延迟**: <50ms（局域网）

## 🎉 测试成功标志

✅ 无人机模型平滑移动沿编辑航线  
✅ 航点状态实时更新（灰→橙→绿）  
✅ 任务状态正确流转（running→completed）  
✅ 左侧面板显示完成信息  
✅ ROS连接状态稳定  
✅ 无WebSocket连接错误  

完成以上验证即表示PCD Viewer的ROS集成功能正常工作！
