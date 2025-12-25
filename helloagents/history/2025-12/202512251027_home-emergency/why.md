# 为什么要改（Why）

## 背景
1. 现场运控提出：任务规划必须严格围绕 HomePos/迫降点，避免遗漏返航路径。
2. 航点信息需包含偏航角（w）与任务类型枚举，以匹配 Missionlogic 协议中 0~5 的定义。
3. 运行面板缺少一键返航/迫降能力，也无法调节航线显示与机库信息面板，影响调度效率。

## 目标
- 在 MissionController/TrajectoryEditor/PCDCanvas/useMissionRuntime 中统一新的航点数据结构（xyz+w+task_type 枚举）。
- 强制 HomePos + 迫降点输入后才能规划，并自动把航线首尾锁定为 HomePos。
- 新增返航/迫降按钮（上传仅包含 HomePos 的航线，迫降 task_type=5），并提供航线宽度/航点大小调节及机库控制信息布局。
