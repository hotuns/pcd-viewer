# 为什么要改（Why）

## 背景
- Missionlogic 任务执行依赖 `task_type` 区分前视/左视/RFID/起降动作，但目前航线编辑器仅支持 XYZ。
- 运营人员需要在规划阶段配置每个航点的任务类型，否则上传 MissionList 时统一回落到默认值。

## 目标
1. 在 TrajectoryEditor/航线拖拽链路中让 `task_type` 成为一等参数，编辑时即可设置。
2. 维持 JSON 导入/导出与 ROS 上传路径的字段完整性，避免信息丢失。
3. 更新知识库，说明 task_type 编辑能力及其数据流。
