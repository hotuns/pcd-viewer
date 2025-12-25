# 数据模型

## 概述
任务/航点/执行日志全部持久化在本地 `data/missions.db`（SQLite），由 `better-sqlite3` 单进程访问。以下结构与 `src/lib/db.ts`、`docs/DATABASE_GUIDE.md` 保持一致。

---

## 数据表

### missions

**描述:** 存放任务基础信息与场景/航线引用。

| 字段名 | 类型 | 约束 | 说明 |
|--------|------|------|------|
| id | TEXT | PRIMARY KEY | 任务 ID（字符串或时间戳） |
| name | TEXT | NOT NULL | 任务名称 |
| status | TEXT | NOT NULL | 参见 `MissionStatus` 枚举 |
| scene_type | TEXT |  | scene 来源（url/file） |
| scene_url | TEXT |  | 当 type=url 时存放 URL/DataURL |
| trajectory_type | TEXT |  | 航线来源 |
| trajectory_url | TEXT |  | 航线 URL/DataURL |
| current_waypoint_index | INTEGER |  | 当前执行中的航点索引 |
| created_at | INTEGER | NOT NULL | Unix ms |
| started_at | INTEGER |  | Unix ms |
| completed_at | INTEGER |  | Unix ms |

**索引:** `idx_missions_status`, `idx_missions_created_at`

### waypoints

**描述:** 记录任务的航点队列与状态。

| 字段名 | 类型 | 约束 | 说明 |
|--------|------|------|------|
| id | TEXT | PRIMARY KEY | 航点 ID（通常 uuid） |
| mission_id | TEXT | FOREIGN KEY | 关联 missions.id（ON DELETE CASCADE） |
| index_num | INTEGER | NOT NULL | 航点顺序，从 0 开始 |
| x/y/z | REAL | NOT NULL | 位置（米） |
| status | TEXT | NOT NULL | pending/active/completed/skipped |
| completed_at | INTEGER |  | Unix ms |

**索引:** `idx_waypoints_mission_id`

### execution_logs

**描述:** 运行时日志，记录 Missionlogic 阶段事件、航点回执等。

| 字段名 | 类型 | 约束 | 说明 |
|--------|------|------|------|
| id | INTEGER | PRIMARY KEY AUTOINCREMENT | 日志主键 |
| mission_id | TEXT | FOREIGN KEY | 任务 ID |
| timestamp | INTEGER | NOT NULL | Unix ms |
| event | TEXT | NOT NULL | 事件关键字，如 `waypoint_reached` |
| details | TEXT |  | JSON 文本（可选） |

**索引:** `idx_execution_logs_mission_id`

---

## 数据模型

### Mission (TypeScript)

| 字段 | 类型 | 说明 |
|------|------|------|
| id | string | 唯一 ID |
| name | string | 任务名 |
| scene | Source | 点云场景（File/URL） |
| trajectory | Source | 航线文件 |
| home | MissionHomePosition | 机库/返航点 |
| emergency | MissionHomePosition | 迫降点（用于紧急航线） |
| status | MissionStatus | 任务状态机 |
| waypoints | Waypoint[] | 航点数组 |
| currentWaypointIndex | number | 当前航点 |
| createdAt/startedAt/completedAt | Date | 生命周期 |
| executionLog | {timestamp,event,details?}[] | 运行日志 |

### Waypoint (TypeScript)

| 字段 | 类型 | 说明 |
|------|------|------|
| id | string | 唯一标识 |
| index | number | 顺序 |
| x/y/z | number | 位置 |
| yaw | number | 偏航角（弧度） |
| taskType | TaskType | 任务类型（数字枚举 0~5） |
| status | WaypointStatus | 状态 |
| completedAt | Date | 完成时间 |

---

## 已知约束与注意事项
- **文件存储:** 数据库目前仅持久化 URL/DataURL；真实 `File` 需额外落地到文件系统或对象存储。
- **并发:** better-sqlite3 单线程，若未来上云需改造成 API 服务层 + 外部 DB。
- **时间戳:** 所有 `Date` 序列化后写入毫秒数，客户端需用 `new Date()` 还原。
