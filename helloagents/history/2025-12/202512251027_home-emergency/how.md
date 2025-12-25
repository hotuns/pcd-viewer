# 怎么改（How）

## 方案概览
1. **航点模型升级**
   - 扩展 `PlannedPoint` / `Waypoint` 数据结构新增 `w`（yaw, rad）与将 `task_type` 统一为 number。
   - TrajectoryEditor + JSON 导入导出 + PCDCanvas 拖拽都需要读写 `w` 与数字类型。
2. **规划约束**
   - MissionController 若未配置 HomePos 与迫降点（新增字段）则禁用规划。
   - 自动将航线数组维护为 `[HomePos, ...用户自定义..., HomePos]`，新增/删除点仅影响中间段。
3. **运行指令**
   - useMissionRuntime 提供 `returnHome()` / `emergencyLand()`，内部调用 `sendMissionList` 仅上传 HomePos，迫降版本 task_type=5。
4. **显示可调**
   - 在 UI（MissionController or TrajectoryEditor 控件）加入 slider 控制 PCDCanvas `lineWidth` 与 `plannedPointSize`。
5. **机库信息布局**
   - 重构 MissionController 左侧控制区，顶部增加 Tabs（如“任务”“机库”），机库 tab 显示 Hangar 状态、按钮（开/关门、充电状态）。

## 模块拆分
| 模块 | 变更 | 说明 |
|------|------|------|
| 类型层 (`src/types/mission.ts`) | `PlannedPoint` 增 `w`、task_type:number；`Waypoint` 视需要补充 yaw | 避免重复定义 |
| TrajectoryEditor | 表单新增 `w` 输入 + task_type 选择（数字枚举），操作仅影响中间段 | 需 UI 组件（Select/NumberInput） |
| MissionController | 新 state：迫降点；首尾锁定 HomePos；提供 lineWidth/pointSize 控件；新增返航/迫降按钮；机库面板 Tab | 需要较多 UI 调整 |
| PCDCanvas | 接收 `plannedPathLineWidth` / `plannedPointScale` props；渲染时使用 yaw（可绘制方向箭头） | 重点是 lineWidth/pointSize 可配置 |
| useMissionRuntime | 新方法 `returnHome()`、`emergencyLand()`；`sendMissionList` 接受 yaw/task_type 数字 | 迫降点使用 home 坐标 + task_type=5 |
| MissionRuntimePanel | 新按钮触发返航/迫降；机库信息分 Tab/区域显示 | 与 MissionController 联动 |
| 文档 | 更新 HelloAgents wiki + Missionlogic + 实时扫描点云文档 | 保持 SSOT |

## 风险与缓解
- **航线首尾锁定**：用户删除 HomePos 造成错乱 → 引入 helper 函数统一增删逻辑。
- **枚举兼容**：旧 JSON 仍是字符串 → 解析时保持兼容（能解析数字/字符串并转换）。
- **UI 调整**：Tab 切换影响现有布局 → 采用最小改动策略（在 MissionController 左栏加 Tab 容器）。
