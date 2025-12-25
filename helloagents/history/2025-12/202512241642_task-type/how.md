# 怎么改（How）

## 方案概览
1. **类型扩展**：在 `src/types/mission.ts` 添加 `PlannedPoint` 类型，`useMissionRuntime`、`MissionController`、`PCDCanvas` 等引用同一结构（含 `task_type`）。
2. **TrajectoryEditor UI**：
   - JSON 解析/导入/导出/手动录入均读写 `task_type`。
   - 列表行新增「任务类型」输入框；手动添加时也可设定默认值。
   - 外部 3D 拖拽回写时保留既有 `task_type`。
3. **数据链路同步**：
   - `MissionController` 持有的 `plannedPoints`、`handleTrajectoryPointsChange`、`handleCanvasPointsChange` 传递 `task_type`。
   - `PCDCanvas` 内部拖拽更新保持对象其余字段。
   - `useMissionRuntime` 上传 MissionList 时直接使用 `task_type`，仍对缺省值回落为 `"0"`。
4. **知识库更新**：在 `helloagents/wiki/overview.md` 增加任务类型可编辑能力描述。

## 模块影响
| 模块 | 影响 | 处理 |
|------|------|------|
| TrajectoryEditor | UI 与状态需要携带 `task_type` | 扩展 `Waypoint` 类型、UI 加输入框 |
| MissionController | `plannedPoints` 类型、解析保存链路 | 使用新类型并在 3D/编辑器之间合并字段 |
| PCDCanvas | 拖拽更新只修改坐标 | `...pt` 保留其余属性 |
| useMissionRuntime | `plannedPoints` 入参需要 task_type | 类型更新并沿用 `sendMissionList` 默认逻辑 |
| 知识库 | 说明新功能 | 更新 overview |

## 风险与缓解
- **跨组件类型漂移**：若某处未更新为新类型可能导致 TS 错误 → 统一导入 `PlannedPoint` 避免匿名类型。
- **旧 JSON 未含 task_type**：UI 默认空字符串，不影响上传；上传阶段依旧回退 `"0"`。
- **拖拽创建新点**：当前仅编辑器创建，3D 只调整坐标，保持 `task_type` 即可。
