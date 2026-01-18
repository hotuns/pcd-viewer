# Missionlogic ROS 消息对照

> 依据 `src/hooks/useMissionRuntime.ts` 当前实现，汇总 Web 端在任务全流程中调用的 ROS 服务与话题，便于后端（Missionlogic）对照实现。

---

## 1. 服务（Services）

| 名称 | 类型 | 方向 | 载荷 | 说明 |
|------|------|------|------|------|
| `/mission/command` | `mission_msgs/MissionCommand` | 客户端调用 | `request: { command: number }`<br>`response: { status?: string }` | 用于触发 Missionlogic 内部状态机指令；目前仅在需要直接调用 MissionCommand（备选方案）时使用。 |

> `command` 数值即 Missionlogic 约定的枚举（TAKEOFF/LAND/EXECUTE…）。当前前端更常用 `/mission/control` + `/mission/task_opt` 组合，`MissionCommand` 作为兼容入口。  

---

## 2. 客户端发布的话题

### 2.1 `/mission/list` — `mission_msgs/MissionList`

- **触发场景**: 上传整条航线、续航剩余航点、上传返航/迫降单点任务。
- **消息结构（简化）**:

```jsonc
{
  "id": int32,             // Mission ID（数值，前端会 clamp 到 Int32）
  "HomePos": {
    "header": { "stamp": { "secs": uint32, "nsecs": uint32 }, "frame_id": "map" },
    "pose": {
      "position": { "x": number, "y": number, "z": number },
      "orientation": { "x": number, "y": number, "z": number, "w": number } // 由 yaw 生成的四元数
    }
  },
  "PosNum": uint32,        // 航点数量
  "PosList": [
    {
      "x": number,
      "y": number,
      "z": number,
      "pass_type": false,  // 上传时恒为 false
      "task_type": string, // 0~5 的字符串，参见 task_type 枚举
      "info": string       // 默认 wp-{index}
    }
  ]
}
```

### 2.2 `/mission/control` — `mission_msgs/Control`

- **触发场景**: TAKEOFF / LAND / EXECUTE / RETURN_HOME / ARM_OFF。
- **消息体**:

```jsonc
{ "cmd": number, "drone_id": number }
```

| cmd | 语义 |
|-----|------|
| 1 | TAKEOFF |
| 2 | LAND |
| 3 | EXECUTE |
| 4 | RETURN_HOME |
| 5 | ARM_OFF |

`drone_id` 默认 `1`（可在 `useMissionRuntime` options.droneId 修改）。

### 2.3 `/mission/task_opt` — `mission_msgs/TaskOpt`

- **触发场景**: 启动 / 暂停 / 停止 Missionlogic 任务。
- **消息体**:

```jsonc
{ "opt": number, "id": int32 }
```

| opt | 语义 |
|-----|------|
| 1 | START（开始任务或续航） |
| 2 | PAUSE |
| 3 | STOP |

`id` 为 Mission ID（取任务 id 数字部分并 clamp int32）。

---

## 3. 客户端订阅的话题

### 3.1 `/mission/status` — `mission_msgs/MissionStatus`

```jsonc
{
  "status": number,          // 0~7 → hangar_ready / mission_upload / ... / charging
  "PosNum": number,
  "PosList": [
    {
      "x": number,
      "y": number,
      "z": number,
      "pass_type": bool,     // true=已通过
      "task_type": string|number,
      "info": string
    }
  ]
}
```

- 前端根据 `status` 映射 MissionPhase。
- `PosList.pass_type` 用于刷新完成/剩余航点、`lastWaypoint`、`pendingWaypoints`。

### 3.2 `/mission/waypoint_feedback` — `mission_msgs/WaypointPosition`

```jsonc
{ "x": number, "y": number, "z": number, "pass_type": bool, "task_type": string, "info": string }
```

- 每通过一个航点推送一次，用于时间线事件 & “最近航点”。

### 3.3 `/hangar/charge_status` — `mission_msgs/HangarChargeStatus`

```jsonc
{
  "charge_status": number,         // 0~4 不同机库状态
  "battery_percentage": number,
  "charge_power": number,
  "charge_duration": number,
  "error_message": string
}
```

- `charge_status` 影响 UI 显示及阶段（充电/错误/ready）。

### 3.4 `/battery/status` — `sensor_msgs/BatteryState`

`voltage`, `percentage`（0~1 浮点，前端换算百分比）, `charge`, `capacity`, `power_supply_status`。

### 3.5 `/odom_visualization/pose` — `nav_msgs/Odometry`

```jsonc
{
  "pose": {
    "pose": {
      "position": { "x": number, "y": number, "z": number },
      "orientation": { "x": number, "y": number, "z": number, "w": number }
    }
  }
}
```

- 用于更新 3D 视图中的无人机位置，并相对于 HomePos 做平移。

### 3.6 `/grid_map/occupancy_inflate` — `sensor_msgs/PointCloud2`

- 解析 `PointCloud2`（`data`, `point_step`, `row_step`, `width`, `height`）并转换为可视化点云。

### 3.7 机库（Nest）相关话题

| Topic | Msg | 说明 |
|-------|-----|------|
| `/nest/status` | `nest_msgs/NestStatus` | 机库内部温度、湿度、手动/自动模式（`Opt_mode`，1=手动，0=自动） |
| `/nest/auto_mode` | `nest_msgs/NestAutoMode` | 自动离巢/进巢状态（`Auto_mode`，1=离巢，0=进巢） |
| `/nest/battery` | `nest_msgs/NestBattery` | 机库供电/电池：SOC、充电电压/电流、单体电压、温度、供电/均衡/触点状态等 |
| `/nest/motor_status` | `nest_msgs/NestMotorStatus` | X/Y 轴电机报警（0 正常/1 异常）以及到位状态（0 开到位/1 合到位/2 未到位） |
| `/nest/uav_on` | `nest_msgs/UavOn` | 无人机在巢状态（0 在巢、1 离巢、2 异常在巢） |

这些话题仅订阅（只读），UI 在“机库控制”标签页中展示温湿度、供电、电池温度、电机报警以及无人机在巢状态。

---

## 4. task_type 定义

| 数值 | 说明 |
|------|------|
| 0 | 无动作（默认） |
| 1 | 云台向前拍摄 |
| 2 | 云台向左拍摄 |
| 3 | 云台向右拍摄 |
| 4 | RFID 采集 |
| 5 | 迫降 |

在 MissionList/Waypoint 相关消息中使用字符串形式（`"0"~"5"`），MissionStatus/feedback 若返回数字也会被 `normalizeTaskType` 兼容。

---

## 5. 典型流程摘要

1. **上传主任务**: `/mission/list`（自动附加迫降点） → `/mission/task_opt` opt=1 → `/mission/control` cmd=3（EXECUTE）。
2. **续航剩余**: `/mission/list`（`pendingWaypoints` + 迫降点） → `/mission/task_opt` opt=1 → `/mission/control` cmd=3。
3. **返航**: 上传仅含 HomePos 的 `/mission/list`，随后 `/mission/task_opt` opt=1 + `/mission/control` cmd=3。
4. **迫降**: 直接 `/mission/task_opt` opt=3（STOP）+ `/mission/control` cmd=2（LAND），立即原地降落。

此文档可直接提供给后端/同事确保 ROS 接口字段保持一致。***
