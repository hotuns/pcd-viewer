# Changelog

本文件记录项目所有重要变更。
格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/),
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [Unreleased]

### 新增
- TrajectoryEditor/PCDCanvas/MissionRuntime 支持为航点配置 `task_type`，JSON 导入/导出与 MissionList 上传均保留该字段。
- 规划阶段引入 HomePos/迫降点约束、航点 yaw（w）输入，以及可调的航线宽度/航点大小；运行面板新增返航/迫降快捷航线与机库控制 Tab。
- MissionController 配置阶段增加“进入规划”图标按钮：满足场景/航线/HomePos/迫降点条件后可一键将任务状态切换为 `planning`。
- MissionController 自动为缺失的 HomePos 与迫降点写入 `(0,0,0,yaw=0)` 默认值，保证初次进入配置阶段即可获得锚点。

## [0.1.0] - 2025-12-24

### 新增
- 基于 Next.js 15 App Router 的任务启动页与 MissionDetail 页面，支持创建/筛选任务并跳转到 3D 主界面。
- 全新的 PCD 视图 + 航线编辑体验：整合 `PCDCanvas`、`TrajectoryEditor`、`MissionController`，完成点云加载、航点拖拽与 Missionlogic 面板联动。
- SQLite `missions/waypoints/execution_logs` 数据层与 `/api/missions` REST API，串联前端 `useMissionDatabase` Hook 与 ROS 运行面板。

### 变更
- Missionlogic 运行面板统一通过 `useMissionRuntime` Hook 与 ROS 话题/服务交互，实现 TAKEOFF/LAND/RETURN 等指令与低电自动返航。

### 修复
- 初始化版本尚无针对性修复记录。

### 移除
- 初始化版本尚无移除记录。
