# Missionlogic ROS 流程与消息梳理

> 供与后端（Missionlogic）对照：描述前端在“配置 → 规划 → 上传/执行 → 返航/迫降 → 续航”各阶段触发的 ROS 接口与消息体要求。代码实现参考 `src/hooks/useMissionRuntime.ts`。

---

## 1. 任务阶段与触发动作

| 阶段 | 入口条件 | 前端动作 | 关键 ROS 消息 |
|------|----------|----------|---------------|
| 配置（setup） | 任务 `status ∈ {draft, configured}`，迫降点如未配置会自动写入 `(0,0,0,yaw=0)` | 上传点云/航线、设置迫降点 | 暂无 |
| 规划（planning） | “进入规划”按钮 -> 设置 `status=planning`，用户在 3D 中拖拽航点 | 保存航线后点击“完成规划” -> `status=ready` | 暂无 |
| 执行准备 | 用户点击“上传任务” | `/mission/list`、`/mission/task_opt`(START)、`/mission/control`(EXECUTE) |
| 运行（runtime） | Missionlogic 反馈 `mission/status` | 订阅进度、机库/电池/位姿、航点反馈、点云 | `/mission/status`、`/hangar/charge_status`、`/battery/status`、`/odom_visualization/pose`、`/mission/waypoint_feedback`、`/grid_map/occupancy_inflate` |
| 返航/迫降 | 用户点击快捷按钮 | 生成单点航线 → `/mission/list` + START + EXECUTE | 同上传任务 |
| 续航剩余 | `pendingWaypoints` 非空，点击“继续任务” | 再次 `/mission/list`，包含剩余航点 | 同上传任务 |

---

## 2. 服务 / 话题清单

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/mission/command` | `mission_msgs/MissionCommand` | Service（call） | 备选，直接调用 MissionCommand（目前主要功能由下列话题承担） |
| `/mission/list` | `mission_msgs/MissionList` | Topic（publish） | 上传航线（完整/返航/迫降/续航） |
| `/mission/control` | `mission_msgs/Control` | Topic（publish） | TAKEOFF / LAND / EXECUTE / RETURN_HOME / ARM_OFF |
| `/mission/task_opt` | `mission_msgs/TaskOpt` | Topic（publish） | START / PAUSE / STOP；与 `/mission/control` 配合使用 |
| `/mission/status` | `mission_msgs/MissionStatus` | Topic（subscribe） | Missionlogic 运行阶段、航点完成情况 |
| `/mission/waypoint_feedback` | `mission_msgs/WaypointPosition` | Topic（subscribe） | 单个航点通过反馈 |
| `/hangar/charge_status` | `mission_msgs/HangarChargeStatus` | Topic（subscribe） | 机库充电/门控信息 |
| `/battery/status` | `sensor_msgs/BatteryState` | Topic（subscribe） | 电池电量 |
| `/odom_visualization/pose` | `nav_msgs/Odometry` | Topic（subscribe） | 无人机位姿 |
| `/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | Topic（subscribe） | 点云栅格 |
| `/nest/status` | `nest_msgs/NestStatus` | Topic（subscribe） | 机库内部温湿度 & 手动/自动模式 |
| `/nest/battery` | `nest_msgs/NestBattery` | Topic（subscribe） | 机库供电、充电电压/电流、单体电压、温度 |
| `/nest/motor_status` | `nest_msgs/NestMotorStatus` | Topic（subscribe） | X/Y 轴电机报警与到位状态 |
| `/nest/auto_mode` | `nest_msgs/NestAutoMode` | Topic（subscribe） | 自动离/进巢模式 |
| `/nest/uav_on` | `nest_msgs/UavOn` | Topic（subscribe） | 无人机当前是否在巢 |

---

## 3. 关键消息载荷

### 3.1 `/mission/list` (`mission_msgs/MissionList`)

```jsonc
{
  "id": int32,            // mission.id 数字化后 clamp
  "HomePos": {
    "header": { "stamp": { "secs": uint32, "nsecs": uint32 }, "frame_id": "map" },
    "pose": {
      "position": { "x": number, "y": number, "z": number },
      "orientation": { "x": number, "y": number, "z": number, "w": number } // 由 yaw 转四元数
    }
  },
  "PosNum": uint32,
  "PosList": [
    {
      "x": number,
      "y": number,
      "z": number,
      "pass_type": false,
      "task_type": "0" | "1" | ... | "5",
      "info": string
    }
  ]
}
```

> **注意**: 前端在上传主任务或续航时，会自动将迫降点追加到 `PosList` 尾部；单独的“返航”仍会发送仅含 HomePos 的 MissionList，而“迫降”不再发送 MissionList。

### 3.2 `/mission/control` (`mission_msgs/Control`)

```jsonc
{ "cmd": 1|2|3|4|5, "drone_id": number }  // 默认 drone_id=1
```

| cmd | 说明 |
|-----|------|
| 1 | TAKEOFF |
| 2 | LAND |
| 3 | EXECUTE |
| 4 | RETURN_HOME |
| 5 | ARM_OFF |

### 3.3 `/mission/task_opt` (`mission_msgs/TaskOpt`)

```jsonc
{ "opt": 1|2|3, "id": int32 }
```

| opt | 说明 |
|-----|------|
| 1 | START（开始或续航） |
| 2 | PAUSE |
| 3 | STOP |

### 3.4 `/mission/status` (`mission_msgs/MissionStatus`)

```jsonc
{
  "status": 0~7,
  "PosNum": number,
  "PosList": [
    {
      "x": number, "y": number, "z": number,
      "pass_type": bool,
      "task_type": string|number,
      "info": string
    }
  ]
}
```

- `status` → MissionPhase 映射：0=hangar_ready, 1=mission_upload, 2=takeoff_ready, 3=takeoff, 4=executing, 5=returning, 6=landing, 7=charging, 其他=idle。
- `pass_type=true` 的航点被认为已完成，用于计算进度与 `lastWaypoint`。

### 3.5 其余订阅

- `/mission/waypoint_feedback`: `{ x, y, z, pass_type, task_type, info }`
- `/hangar/charge_status`: `{ charge_status, battery_percentage, charge_power, charge_duration, error_message }`
- `/battery/status`: `sensor_msgs/BatteryState` 原始结构（percentage 为 0~1 浮点）。
- `/odom_visualization/pose`: `nav_msgs/Odometry.pose.pose`。
- `/grid_map/occupancy_inflate`: `sensor_msgs/PointCloud2`。

---

## 4. 典型消息时序

1. **上传/启动主任务**
   1. `/mission/list` —— `PosList` = 编辑后的航线。
   2. `/mission/task_opt` —— `{ opt:1, id }`。
   3. `/mission/control` —— `{ cmd:3 (EXECUTE), drone_id }`。
2. **续航剩余航点**
   1. `/mission/list` —— `PosList` = `pendingWaypoints`。
   2. `/mission/task_opt` opt=1。
   3. `/mission/control` cmd=3。
3. **返航**
   1. `/mission/list` —— 单点（HomePos, task_type=0, info="返航航线"）。
   2. `/mission/task_opt` opt=1。
   3. `/mission/control` cmd=3。
4. **迫降**
   1. `/mission/task_opt` —— opt=3 (STOP)。
   2. `/mission/control` —— cmd=2 (LAND)，立即原地降落。

---

## 5. 自动返航阈值

实时订阅 `/battery/status` 后，只要 `voltage <= 21V` 就会自动触发 RETURN_HOME。电压恢复到 22V 以上才会解除该保护。若电压数据缺失，会回退到历史的“百分比 ≤ 25%”逻辑。

---

## 6. task_type 枚举

| 值 | 动作 |
|----|------|
| 0 | 无动作（默认） |
| 1 | 云台向前拍摄 |
| 2 | 云台向左拍摄 |
| 3 | 云台向右拍摄 |
| 4 | RFID 数据采集 |
| 5 | 迫降 |

所有 `PosList` / 反馈消息中使用 `"0"~"5"` 字符串；后端若返回数字，前端会调用 `normalizeTaskType` 兼容。

---

如需扩展新的控制指令或消息字段，请同步更新此文档及 `useMissionRuntime.ts` 中的常量定义。***
