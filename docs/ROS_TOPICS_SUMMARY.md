# 前端交互与消息流转流程图

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
