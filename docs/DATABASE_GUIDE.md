# SQLite 数据库集成使用指南

## 概述

项目已集成 SQLite 数据库来持久化存储任务信息，使用 `better-sqlite3` 库。

## 数据库结构

### 表设计

#### 1. missions（任务表）
```sql
CREATE TABLE missions (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  status TEXT NOT NULL,
  scene_type TEXT,
  scene_url TEXT,
  trajectory_type TEXT,
  trajectory_url TEXT,
  current_waypoint_index INTEGER,
  created_at INTEGER NOT NULL,
  started_at INTEGER,
  completed_at INTEGER
)
```

#### 2. waypoints（航点表）
```sql
CREATE TABLE waypoints (
  id TEXT PRIMARY KEY,
  mission_id TEXT NOT NULL,
  index_num INTEGER NOT NULL,
  x REAL NOT NULL,
  y REAL NOT NULL,
  z REAL NOT NULL,
  status TEXT NOT NULL,
  completed_at INTEGER,
  FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
)
```

#### 3. execution_logs（执行日志表）
```sql
CREATE TABLE execution_logs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  mission_id TEXT NOT NULL,
  timestamp INTEGER NOT NULL,
  event TEXT NOT NULL,
  details TEXT,
  FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
)
```

## 使用方法

### 在组件中使用

```typescript
import { useMissionDatabase } from '@/hooks/useMissionDatabase';

function MyComponent() {
  const { 
    saveMission, 
    updateMission, 
    deleteMission,
    getMission,
    getAllMissions,
    getMissionStats,
    loading,
    error 
  } = useMissionDatabase();

  // 保存任务
  const handleSave = async () => {
    const success = await saveMission(missionData);
    if (success) {
      console.log('Mission saved!');
    }
  };

  // 更新任务
  const handleUpdate = async (id: string) => {
    const success = await updateMission(id, {
      status: 'completed',
      completedAt: new Date(),
    });
  };

  // 获取所有任务
  const loadMissions = async () => {
    const missions = await getAllMissions();
    console.log('All missions:', missions);
  };

  // 获取统计
  const loadStats = async () => {
    const stats = await getMissionStats();
    console.log('Mission stats:', stats);
  };
}
```

### API 路由

#### GET /api/missions
获取所有任务列表

```bash
curl http://localhost:3000/api/missions
```

#### GET /api/missions?stats=true
获取任务统计

```bash
curl http://localhost:3000/api/missions?stats=true
```

#### GET /api/missions/:id
获取特定任务详情

```bash
curl http://localhost:3000/api/missions/mission-123
```

#### POST /api/missions
创建新任务

```bash
curl -X POST http://localhost:3000/api/missions \
  -H "Content-Type: application/json" \
  -d '{"id":"mission-1","name":"Test Mission","status":"draft",...}'
```

#### PUT /api/missions/:id
更新任务

```bash
curl -X PUT http://localhost:3000/api/missions/mission-123 \
  -H "Content-Type: application/json" \
  -d '{"status":"completed"}'
```

#### DELETE /api/missions/:id
删除任务

```bash
curl -X DELETE http://localhost:3000/api/missions/mission-123
```

## 服务器端直接使用

```typescript
import { 
  createMission, 
  updateMission, 
  deleteMission,
  getMissionById,
  getAllMissions,
  getMissionStats,
  addExecutionLog 
} from '@/lib/missionDao';

// 创建任务
createMission(mission);

// 获取任务
const mission = getMissionById('mission-123');

// 更新任务
updateMission('mission-123', { status: 'completed' });

// 添加执行日志
addExecutionLog('mission-123', 'waypoint_reached', { 
  waypointIndex: 5 
});

// 获取统计
const stats = getMissionStats();
```

## 数据库文件位置

- 数据库文件：`./data/missions.db`
- 自动创建：首次运行时自动创建数据库和表
- WAL 文件：`missions.db-shm` 和 `missions.db-wal`（SQLite 写前日志）

## 注意事项

### 1. File 对象限制
由于数据库无法直接存储 `File` 对象，当前实现：
- ✅ 支持存储 URL 类型的场景和轨迹
- ❌ 不支持直接存储 File 类型（需要额外处理，如上传到文件系统）

### 2. 事务处理
批量操作（如插入多个航点）使用数据库事务，确保数据一致性。

### 3. 性能优化
- 已创建索引：`status`、`created_at`、`mission_id`
- 启用外键约束：级联删除关联数据

### 4. 时区处理
所有时间戳存储为 Unix 毫秒时间戳（UTC），客户端转换为 Date 对象。

## 扩展建议

### 存储 File 对象
如果需要存储上传的 PCD/PLY 文件：

```typescript
// 1. 将文件保存到文件系统
import fs from 'fs/promises';
import path from 'path';

const uploadDir = path.join(process.cwd(), 'uploads');
const filePath = path.join(uploadDir, `${missionId}-scene.pcd`);
await fs.writeFile(filePath, fileBuffer);

// 2. 在数据库中存储文件路径
updateMission(missionId, {
  sceneUrl: `/uploads/${missionId}-scene.pcd`
});
```

### 添加全文搜索
```sql
CREATE VIRTUAL TABLE missions_fts USING fts5(
  id, 
  name, 
  content='missions'
);
```

### 数据备份
```bash
# 备份数据库
cp data/missions.db data/missions_backup_$(date +%Y%m%d).db

# 或使用 SQLite 命令
sqlite3 data/missions.db ".backup data/missions_backup.db"
```

## 故障排查

### 数据库锁定
如果遇到 "database is locked" 错误：
```typescript
db.pragma('busy_timeout = 5000'); // 5秒超时
```

### 查看数据库内容
```bash
# 安装 SQLite 命令行工具
sqlite3 data/missions.db

# 查看所有表
.tables

# 查看表结构
.schema missions

# 查询数据
SELECT * FROM missions;
```

## 性能监控

```typescript
// 启用查询日志
db.pragma('journal_mode = WAL');
db.pragma('synchronous = NORMAL');

// 查看数据库统计
console.log(db.pragma('database_list'));
console.log(db.pragma('table_info(missions)'));
```
