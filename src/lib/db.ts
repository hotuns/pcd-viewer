import Database from 'better-sqlite3';
import path from 'path';
import fs from 'fs';

// 数据库文件路径
const DB_DIR = path.join(process.cwd(), 'data');
const DB_PATH = path.join(DB_DIR, 'missions.db');

// 确保数据目录存在
if (!fs.existsSync(DB_DIR)) {
  fs.mkdirSync(DB_DIR, { recursive: true });
}

// 创建数据库连接
const db = new Database(DB_PATH);

// 启用外键约束
db.pragma('foreign_keys = ON');

// 初始化数据库表
export function initDatabase() {
  // 任务表
  db.exec(`
    CREATE TABLE IF NOT EXISTS missions (
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
  `);

  // 航点表
  db.exec(`
    CREATE TABLE IF NOT EXISTS waypoints (
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
  `);

  // 执行日志表
  db.exec(`
    CREATE TABLE IF NOT EXISTS execution_logs (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      mission_id TEXT NOT NULL,
      timestamp INTEGER NOT NULL,
      event TEXT NOT NULL,
      details TEXT,
      FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
    )
  `);

  // 创建索引
  db.exec(`
    CREATE INDEX IF NOT EXISTS idx_missions_status ON missions(status);
    CREATE INDEX IF NOT EXISTS idx_missions_created_at ON missions(created_at);
    CREATE INDEX IF NOT EXISTS idx_waypoints_mission_id ON waypoints(mission_id);
    CREATE INDEX IF NOT EXISTS idx_execution_logs_mission_id ON execution_logs(mission_id);
  `);

  console.log('Database initialized at:', DB_PATH);
}

// 初始化数据库（只在服务器端执行）
if (typeof window === 'undefined') {
  initDatabase();
}

export default db;
