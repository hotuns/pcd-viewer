# Mission Runtime Orchestrator

## 目的
MissionController + MissionRuntimePanel + hooks 组成的运行编排层，负责 ROS 连接、Missionlogic 状态反馈与操作按钮。

## 模块概述
- **职责:** 维护当前任务/航线/HomePos，全局 ROS 连接状态，触发 Missionlogic 命令并渲染运行面板。
- **状态:** 🚧开发中
- **最后更新:** 2025-12-24

## 规范

### 需求: Mission 运行握手
**模块:** Mission Runtime Orchestrator  
确保 MissionDetail 页渲染 MissionController 时能正确载入任务并连接 ROS。

#### 场景: Mission 加载
- `MissionDetailClient` 通过 `/api/missions/:id` 获取数据、转换日期后注入 `initialMission`。
- MissionController 收到后设置本地 state，并在 mission 变化时同步 `homePose`。

### 需求: 配置阶段进入规划
**模块:** Mission Runtime Orchestrator  
提供从“配置阶段”进入“规划阶段”的显式入口，避免任务停留在草稿/已配置状态。

- UI: TaskConfigPanel/Homes 表单下方新增 `ArrowRight` 图标按钮（Tooltip 展示当前限制）。
- 条件：`scene`、`trajectory`、HomePos、迫降点全部存在时按钮可点，调用 `handleMissionUpdate({...status: "planning"})`。
- 失败场景提示：Tooltip 文案按优先级提示缺少的配置（场景、航线、HomePos、迫降点）。

### 需求: 默认 HomePos / 迫降点
**模块:** Mission Runtime Orchestrator  
MissionController 载入任务时若缺少 HomePos 或迫降点，需自动写入 `(0,0,0)`、`yaw=0`、`frameId=map` 的默认值并立即持久化。

- 逻辑：`useEffect` 检测 `selectedMission.home/emergency`，缺失则克隆 `DEFAULT_HOME` 写入 mission。
- Side effect: 更新本地 `homePose/emergencyPose` state，确保 TrajectoryEditor Anchor 能立刻可用。
- 目的：避免“配置阶段”卡在必填提示，让未配置任务也具备初始锚点，可直接调整后再覆盖真实坐标。

#### 场景: ROS 连接
- `useRosConnection` 默认地址 `ws://192.168.203.30:9999`，同时持久化到 localStorage。
- 成功连接后 `rosConnected=true`，由 `MissionRuntimePanel` 以 Badge 提示；失败则在 UI 上提示 `connectionError`。

### 需求: Missionlogic 指令流程
**模块:** Mission Runtime Orchestrator  
`useMissionRuntime` 封装 Missionlogic 的话题/服务调用与订阅。

#### 场景: 上传/执行
- `onUploadMission`：通过 `missionCommandService` / `missionListTopic` 推送 MissionList。
- `onExecuteMission` + `onTakeoff`：先发布 TAKEOFF，再发布 EXECUTE；busyAction 期间按钮置灰。
- `onReturnHome`/`onLand`：调用 Control Topic cmd=RETURN_HOME/LAND，并在成功后清空 busyAction。

#### 场景: 状态/事件更新
- MissionStatus Topic 映射至 MissionPhase（0~7）。
- Waypoint feedback 更新 `lastWaypoint` & `progress`。
- Hangar/Battery Topic 映射至面板电量/充电信息；低电时触发自动返航（`autoReturnTriggered`）。

### 需求: 航线/点云同步
**模块:** Mission Runtime Orchestrator  
PCDCanvas/TrajectoryEditor 需要共享航线点。

#### 场景: 导入航线
- Mission `trajectory` 支持 URL/File；加载逻辑按来源 fetch/text → JSON.parse → `plannedPoints`。
- 拖拽/编辑后调用 `onPlannedPointsChange` 更新 state 与 `setPlannedPoints`。
- 在保存时将 File 转成 DataURL 存入 Mission（当前数据库仅支持 URL）。
- 自 2025-12-24 起，`plannedPoints` 采用 `PlannedPoint` 结构，包含 `w/t/task_type/info`，上传 MissionList 时会保留任务类型（缺省回退为 `"0"`）。

## API接口
- `useMissionDatabase.updateMission`：MissionController 在更改 home/scene/trajectory 时更新 SQLite。
- MissionRuntimePanel 中 `missionReady` 依赖 mission home + plannedPoints 是否齐全。

## 数据模型
- `PlannedPoint`: 航线编辑器使用的点结构（x/y/z/t/task_type/info）
- `MissionHomePosition`：frameId + position + yaw（rad）。
- `MissionRuntimeEvent`：`{id,timestamp,level,message,details?}`，在 UI 中以时间倒序显示。

## 新增运行控制
- 任务控制面板提供“上传返航航线”“上传迫降航线”按钮：分别使用 HomePos（task_type=0）与迫降点（task_type=5）生成单点 MissionList 并立即执行。
- 面板顶部新增 Tab，可在“任务控制 / 机库控制”间切换。机库面板集中展示充电百分比、功率、时长以及门控、ARM OFF 操作。

## 依赖
- Hooks: `useMissionDatabase`, `useRosConnection`, `useMissionRuntime`
- UI 子组件: `TaskInfo`, `TaskConfigPanel`, `MissionRuntimePanel`, `MissionHomeForm`, `TrajectoryEditor`
- 3D: `PCDCanvas`, `PCDCanvasHandle`（视角控制）
- ROSLIB 话题/服务：MissionCommand、MissionList、MissionStatus、Control、TaskOpt、HangarChargeStatus、BatteryState、Pose、WaypointFeedback、PointCloud

## 变更历史
- 2025-12-24 首次记录模块职责，标记 ROS 操作流程。*** End Patch***
