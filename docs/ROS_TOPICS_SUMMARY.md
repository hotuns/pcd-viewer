# ROS 话题订阅梳理与需求

## 当前已实现的订阅

### 1. 无人机位置话题 ✅ (ROS已提供)
- **话题名称**: `/odom_visualization/pose`
- **消息类型**: `geometry_msgs/PoseStamped`
- **功能**: 获取无人机实时位置和姿态
- **数据结构**:
  ```json
  {
    "pose": {
      "position": { "x": float, "y": float, "z": float },
      "orientation": { "x": float, "y": float, "z": float, "w": float }
    }
  }
  ```
- **触发时机**: 点击"开始任务"后订阅
- **状态**: ✅ 已实现并测试通过

### 2. 点云地图话题 ✅ (ROS已提供)
- **话题名称**: `/grid_map/occupancy_inflate`
- **消息类型**: `sensor_msgs/PointCloud2`
- **功能**: 获取无人机扫描的实时点云数据
- **数据结构**:
  ```json
  {
    "header": {
      "stamp": { "sec": int, "nsec": int },
      "frame_id": string
    },
    "height": int,
    "width": int,
    "fields": [{ "name": string, "offset": int, "datatype": int, "count": int }],
    "is_bigendian": bool,
    "point_step": int,
    "row_step": int,
    "data": string (base64编码的二进制数据)
  }
  ```
- **触发时机**: 点击"开始任务"后订阅（且"显示点云地图"开关打开）
- **状态**: ✅ 已实现，支持base64解码和3D渲染

---

## 需要 ROS 补充的话题（实现完整任务流程）

### 3. 航点到达通知话题 ❌ (需要ROS提供)
- **话题名称**: `/mission/waypoint_reached`
- **消息类型**: `std_msgs/String`
- **功能**: 当无人机到达一个航点时通知前端
- **建议数据结构** (JSON字符串):
  ```json
  {
    "waypoint_index": int,        // 到达的航点索引 (0-based)
    "position": {
      "x": float,
      "y": float, 
      "z": float
    },
    "timestamp": int              // Unix时间戳（秒）
  }
  ```
- **使用场景**: 
  - 前端更新航点状态（pending → completed）
  - 显示任务进度
  - 改变航线颜色（已完成部分显示绿色）
- **状态**: ⚠️ 代码已实现订阅，但ROS端需要发布

### 4. 任务完成/失败通知话题 ❌ (需要ROS提供)
- **话题名称**: `/mission/complete`
- **消息类型**: `std_msgs/String`
- **功能**: 任务完成或失败时通知前端
- **建议数据结构** (JSON字符串):
  ```json
  {
    "status": "completed" | "failed",  // 任务状态
    "reason": string,                  // 完成/失败原因（可选）
    "timestamp": int                   // Unix时间戳（秒）
  }
  ```
- **使用场景**:
  - 前端弹出任务完成/失败提示
  - 更新任务状态
  - 记录执行日志
- **状态**: ⚠️ 代码已实现订阅，但ROS端需要发布

---

## ROS 端需要实现的功能

### 最小可行方案（MVP）

如果只使用当前ROS提供的两个话题（位置 + 点云），前端可以：
- ✅ 实时显示无人机位置
- ✅ 渲染实时点云地图
- ✅ 显示编辑航线
- ✅ 相机跟随功能
- ❌ **无法知道航点到达状态**（需要前端自己计算距离判断）
- ❌ **无法知道任务完成**（需要前端判断是否到达终点）

### 推荐方案

**方案A：ROS发布航点到达和任务完成话题**
- 优点：前端逻辑简单，状态准确
- 需要ROS端添加：
  1. 航点到达判断逻辑
  2. 发布 `/mission/waypoint_reached` 话题
  3. 发布 `/mission/complete` 话题

**方案B：前端自己计算（临时方案）**
- 优点：不需要修改ROS端
- 缺点：
  - 需要前端持续计算距离
  - 可能不够准确
  - 增加前端计算负担
- 实现思路：
  ```typescript
  // 在位置更新时判断
  const distanceToWaypoint = Math.hypot(
    dronePos.x - waypoint.x,
    dronePos.y - waypoint.y,
    dronePos.z - waypoint.z
  );
  
  if (distanceToWaypoint < THRESHOLD) {
    // 标记航点为已到达
  }
  ```

---

## 可选扩展话题（增强功能）

### 5. 任务状态话题（可选）
- **话题名称**: `/mission/status`
- **消息类型**: `std_msgs/String`
- **功能**: 实时报告任务执行状态
- **数据结构**:
  ```json
  {
    "state": "idle" | "takeoff" | "executing" | "landing" | "paused",
    "current_waypoint": int,
    "progress": float,  // 0.0 - 1.0
    "timestamp": int
  }
  ```

### 6. 电池状态话题（可选）
- **话题名称**: `/battery/status`
- **消息类型**: `sensor_msgs/BatteryState`
- **功能**: 显示无人机电池电量

### 7. 速度信息话题（可选）
- **话题名称**: `/velocity`
- **消息类型**: `geometry_msgs/TwistStamped`
- **功能**: 显示无人机当前速度

---

## 总结

### 当前状态
- ✅ **已有**: 位置、点云
- ⚠️ **缺失**: 航点到达通知、任务完成通知

### 建议
1. **优先级P0**: ROS端实现航点到达判断和通知
   - 发布 `/mission/waypoint_reached` 话题
   - 发布 `/mission/complete` 话题

2. **优先级P1**: 如果ROS端短期无法实现，前端可以临时采用距离判断方案

3. **优先级P2**: 后续可考虑添加电池、速度等扩展信息

---

## 前端交互与消息流转流程图

使用 Markdown 画出的流程图，左侧是前端操作，右侧标注对应的 ROS 消息收发：

```text
┌───────────────┐
│ 1. 连接 ROS   │
└───┬───────────┘
    │ WebSocket
    ▼
┌────────────────────┐
│ 2. 上传航线        │ -- mission_msgs/MissionList --> /mission/list
└────┬───────────────┘
     │ MissionStatus=mission_upload
     ▼
┌──────────────────────────────┐
│ 3. 起飞并执行                │
│   • TaskOpt(opt=1) ---------> /mission/task_opt
│   • Control(cmd=1/3) -------> /mission/control
└──────┬───────────────────────┘
       │ MissionStatus=takeoff/executing
       ▼
┌──────────────────────────────┐
│ 4. 执行中监控                │
│   ← MissionStatus (阶段&航点)
│   ← WaypointPosition (反馈)
│   ← HangarChargeStatus/BatteryState
│   ← PoseStamped (无人机位姿)
└──────┬───────────────────────┘
       │（可视化/日志/自动触发返航）
       ▼
┌──────────────────────────────┐
│ 5. 返航 / 降落 / 停止        │
│   • TaskOpt(opt=3) ---------> /mission/task_opt
│   • Control(cmd=4/2/5) -----> /mission/control
└──────┬───────────────────────┘
       │ MissionStatus=returning/landing/charging
       ▼
┌──────────────────────────────┐
│ 6. 充电完成，机库 ready      │
│   ← HangarChargeStatus(status=2)
└──────────────────────────────┘
```
