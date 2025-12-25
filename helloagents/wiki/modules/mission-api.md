# Mission Persistence & API

## 目的
统一管理任务数据的 REST 接口与 SQLite DAO，供前端 hook、MissionController、StartupPage 使用。

## 模块概述
- **职责:** `/api/missions` 与 `/api/missions/[id]` 路由、`missionDao.ts`、`db.ts` 初始化及 `useMissionDatabase` Hook。
- **状态:** ✅稳定
- **最后更新:** 2025-12-24

## 规范

### 需求: 数据库初始化
**模块:** Mission Persistence  
确保 Next.js Server 侧在首次 import 时创建 `data/missions.db`。

#### 场景: initDatabase
- `src/lib/db.ts` 在 Node 端执行 `initDatabase()`，创建 missions/waypoints/execution_logs 表与索引。
- 缺失 data 目录时自动 `fs.mkdirSync`（recursive）。

### 需求: REST API（集合）
**模块:** Mission Persistence  
`/api/missions` 处理 POST（创建）与 GET（列表/统计）。

#### 场景: GET 列表
- 支持 `?stats=true` 分支，调用 `getMissionStats()`。
- 默认返回 `rowToMission` 映射后的任务数组（无航点/日志）。

#### 场景: POST 创建
- 仅允许 scene/trajectory 的 URL 来源；File 类型需在前端转换成 DataURL。
- 服务器补足 `createdAt=new Date()` 并写库，返回 `{success:true, missionId}`。

### 需求: REST API（单项）
**模块:** Mission Persistence  
`/api/missions/[id]` 提供 GET/PUT/DELETE/POST。

#### 场景: GET
- 返回 Mission + waypoints + executionLog；若不存在返回 404。

#### 场景: PUT
- 允许部分字段，时间字段可以 string / number / Date；`normalizeMissionPayload` 负责转换。
- 若附带 `waypoints` 将先删后插。

#### 场景: DELETE
- 删除 missions 行并依赖外键清空关联的航点/日志。

#### 场景: POST (导入)
- 直接传完整 Mission，并调用 `createMission`；用于批量导入或模拟器。

## API接口
- `useMissionDatabase` Hook 暴露 `saveMission/updateMission/deleteMission/getMission/getAllMissions/getMissionStats`。
- 内部使用 `fetch`，默认 JSON Content-Type，抛错时写入 `error` state。

## 数据模型
- 详见 `wiki/data.md`；DAO `missionToRow/rowToMission/waypointToRow/rowToWaypoint` 保证字段转换。

## 依赖
- better-sqlite3、Node fs/path
- Next.js `NextRequest/NextResponse`
- 类型：`Mission`、`Waypoint`、`Source`

## 变更历史
- 2025-12-24 整理模块文档，覆盖 v0.1.0 的 API 功能。
