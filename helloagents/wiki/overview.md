# PCD Viewer Mission Manager

> 本文件包含项目级别的核心信息。详细的模块文档见 `modules/` 目录。

---

## 1. 项目概述

### 目标与背景
围绕 Missionlogic 状态机重构的无人机任务地面软件，集中解决“机库 Ready → 任务配置 → 上传 → 起飞/执行/返航/降落/充电”闭环，并提供可视化航线编辑与 ROS 遥测监控。

### 范围
- **范围内:** 任务管理（列表/启动页）、MissionDetail 场景编辑、ROS 运行面板、点云渲染与航点编辑、SQLite 持久化、REST API。
- **范围外:** 服务端调度器/任务规划算法、真实飞控适配层、跨设备同步、权限系统。

### 干系人
- **负责人:** hotuns（全栈），ROS/飞控团队提供 Missionlogic 协议/消息。

---

## 2. 模块索引

| 模块名称 | 职责 | 状态 | 文档 |
|---------|------|------|------|
| Startup Launcher | 任务启动页、统计、筛选与跳转 | ✅稳定 | [modules/startup-launcher.md](modules/startup-launcher.md) |
| Mission Runtime Orchestrator | MissionController + useMissionRuntime 驱动 ROS 指令/遥测 | 🚧开发中 | [modules/mission-runtime.md](modules/mission-runtime.md) |
| PCD Canvas & Editor | 点云加载、航点可视化/拖拽、无人机模型跟随 | 🚧开发中 | [modules/pcd-canvas.md](modules/pcd-canvas.md) |
| Mission Persistence & API | SQLite DAO、Next.js `/api/missions`、客户端 Hook | ✅稳定 | [modules/mission-api.md](modules/mission-api.md) |

---

## 3. 快速链接
- [技术约定](../project.md)
- [架构设计](arch.md)
- [API 手册](api.md)
- [数据模型](data.md)
- [变更历史](../history/index.md)

## 4. 运行特性更新

- TrajectoryEditor + PCDCanvas 支持在航线规划阶段为每个航点配置 `task_type`，并在 JSON 导入/导出与 MissionList 上传过程中保持同步，保障不同任务动作（起降、前视/侧视拍摄、RFID）不会丢失。
- 规划阶段强制填写 HomePos 与迫降点，航线首尾自动锁定到 HomePos，支持编辑 yaw(w) 以及可调的航线宽度/航点大小。
- 若任务缺少 HomePos/迫降点，MissionController 会自动写入 `(0,0,0)` 与 `yaw=0` 的默认坐标，避免初次配置时出现空值。
- MissionRuntimePanel 增加“返航航线”“迫降航线”快捷按钮并提供机库控制 Tab，集中展示充电/门控信息。
- MissionController 的配置阶段新增“进入规划”图标按钮：仅当场景点云、航线 JSON、HomePos、迫降点全部配置完成时才可点击，以使命状态切换为 `planning` 并展开航点编辑。
