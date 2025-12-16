## UAV Mission Manager v0.3

一个围绕《Missionlogic.md》状态机重构的无人机任务地面软件，基于 Next.js + three.js (@react-three/fiber, @react-three/drei) 构建。当前聚焦「机库 ready → 任务上传 → 起飞 → 执行 → 返航 → 降落/充电」闭环，使用 roslibjs 与 `mission_msgs` 中的消息示例交互。

### 功能概览

- 任务配置
	- 上传/链接点云场景（.pcd/.ply），即时加载到 3D 视图
	- 使用全新的 `TrajectoryEditor` 编辑 MissionList 轨迹，支持 JSON 文件往返
	- 在 `HomePos` 表单中维护机库/返航点 (PoseStamped)
- Missionlogic 运行面板
	- 通过 `MissionCommand` 服务控制机库开关
	- 一键上传 MissionList、发送 TAKEOFF/LAND/EXECUTE/RETURN_HOME/ARM_OFF 控制 (`mission_msgs/Control`)
	- 订阅 `mission_msgs/MissionStatus`、`mission_msgs/HangarChargeStatus`、`sensor_msgs/BatteryState`，可视化阶段、航点、充电状态
	- 电量低于阈值自动触发回家任务，符合 Missionlogic.md 描述
- ROS 实时遥测
	- 订阅 `/odom_visualization/pose`，三维视图展示无人机模型；可开启视角跟随
	- 轨迹点云仅展示规划航线，不再绘制实时航迹/网格地图，减少与 Missionlogic 不一致的扩展功能

> `mission_msgs/` 目录内附带了 MissionList、WaypointPosition、HangarChargeStatus 等示例消息定义，可直接在 ROS 工作空间中编译使用。

### 当前完成情况（2025-12-10）

- ✅ 将 Missionlogic.md 中的状态机流程映射到前端运行面板：包含机库开/关、任务上传、起飞/执行/返航/降落/停桨等指令按钮，以及电量低自动返航逻辑。
- ✅ 新增 `useMissionRuntime` Hook，统一连接 `mission_msgs` 中的 MissionCommand/MissionList/MissionStatus/HangarChargeStatus/BatteryState 等 ROS 接口，实时展示航点进度与机库充电状态。
- ✅ 左侧界面重构：TaskInfo 显示 Mission 状态 + Runtime 阶段，MissionRuntimePanel 负责 Missionlogic 操作，MissionHomeForm + TrajectoryEditor 用于配置 HomePos 与航线。
- ✅ `mission_msgs` 增补 HangarChargeStatus.msg，CMakeLists 已包含，可直接在 ROS 工作空间编译。
- ⚠️ 数据持久化层暂未存储 HomePos、运行事件等扩展字段，刷新页面会丢失这些信息；后续需扩展 `/api/missions` 与 SQLite 表结构。

### 运行

```bash
npm run dev
```

打开浏览器访问 http://localhost:3000

### 使用

1) 创建任务
- 在侧栏创建任务并选择：
	- 场景（点云 .pcd）：上传文件或填写 URL
	- 航线轨迹：上传 JSON/CSV 或填写 URL（当前不绘制，仅保存）

2) ROS 实时位置
- 在侧栏填写 ROSBridge 地址（如 ws://101.132.193.179:7001），点击连接
- 点击「开始任务」后，将通过 roslibjs 订阅 /mavros/local_position/odom 并在侧栏显示实时坐标与速度

### 文件结构（关键）

- `src/components/pcd/PCDCanvas.tsx`：3D 场景（点云加载 + 视角适配 + 基本显示）
- `src/components/pcd/MissionController.tsx`：任务创建/选择、ROS 连接与实时位置订阅、基本显示设置
- `src/components/mission/MissionManager.tsx`：任务增删改与资源（场景/航线）配置
- `src/app/page.tsx`：首页入口

### 已知限制（v0.2）

- 当前不绘制航迹，仅显示实时位置数据
- PCD 二进制格式兼容性依赖 three.js 的 PCDLoader，不同导出器可能存在差异（建议优先 ASCII）
- ROS 话题与消息类型需按实际环境调整（默认 /mavros/local_position/odom, nav_msgs/msg/Odometry）

### 文档

所有补充说明文档已集中放在 [docs/](./docs/README.md) 目录，可按主题查阅：

- 架构/交互：`docs/INTERACTION_FLOW.md`、`docs/LEFT_PANEL_REFACTOR.md`、`docs/REFACTORING_SUMMARY.md`
- 数据/存储：`docs/DATABASE_GUIDE.md`
- ROS 集成：`docs/ROS_TOPICS_SUMMARY.md`、`docs/ROS_TEST_GUIDE.md`、`docs/test_ros_connection.md`、`docs/ros.md`
- 渲染相关：`docs/VOXEL_RENDERING_GUIDE.md`

新增文档时请直接在 `docs/` 下创建并更新索引，保持根目录整洁。
