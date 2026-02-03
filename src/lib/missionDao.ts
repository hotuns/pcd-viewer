import db from './db';
import type { Mission, Waypoint, Source } from '@/types/mission';

/**
 * 任务数据访问对象
 */

// 数据库行类型
interface MissionRow {
  id: string;
  name: string;
  status: string;
  scene_type: string | null;
  scene_url: string | null;
  trajectory_type: string | null;
  trajectory_url: string | null;
  current_waypoint_index: number | null;
  created_at: number;
  started_at: number | null;
  completed_at: number | null;
}

interface WaypointRow {
  id: string;
  mission_id: string;
  index_num: number;
  x: number;
  y: number;
  z: number;
  status: string;
  completed_at: number | null;
}

interface ExecutionLogRow {
  id: number;
  mission_id: string;
  timestamp: number;
  event: string;
  details: string | null;
}

interface StatsRow {
  status: string;
  count: number;
}

// 将 Mission 对象转换为数据库行
function missionToRow(mission: Mission) {
  return {
    id: mission.id,
    name: mission.name,
    status: mission.status,
    scene_type: mission.scene?.type || null,
    scene_url: mission.scene?.type === 'url' ? mission.scene.url : null,
    trajectory_type: mission.trajectory?.type || null,
    trajectory_url: mission.trajectory?.type === 'url' ? mission.trajectory.url : null,
    current_waypoint_index: mission.currentWaypointIndex ?? null,
    created_at: mission.createdAt.getTime(),
    started_at: mission.startedAt?.getTime() ?? null,
    completed_at: mission.completedAt?.getTime() ?? null,
  };
}

// 将数据库行转换为 Mission 对象（不包含航点和日志）
const normalizeFileUrl = (url: string | null) => {
  if (!url) return null;
  if (url.startsWith('/api/uploads/')) return url;
  if (url.startsWith('/uploads/')) return `/api${url}`;
  return url;
};

function rowToMission(row: MissionRow): Omit<Mission, 'waypoints' | 'executionLog'> & {
  sceneUrl?: string;
  trajectoryUrl?: string;
} {
  const normalizedSceneUrl = normalizeFileUrl(row.scene_url);
  const normalizedTrajectoryUrl = normalizeFileUrl(row.trajectory_url);
  const sceneSource: Source | undefined = normalizedSceneUrl ? { type: 'url', url: normalizedSceneUrl } : undefined;
  const trajectorySource: Source | undefined = normalizedTrajectoryUrl ? { type: 'url', url: normalizedTrajectoryUrl } : undefined;
  return {
    id: row.id,
    name: row.name,
    status: row.status as Mission['status'],
    scene: sceneSource,
    sceneUrl: normalizedSceneUrl ?? undefined,
    trajectory: trajectorySource,
    trajectoryUrl: normalizedTrajectoryUrl ?? undefined,
    currentWaypointIndex: row.current_waypoint_index ?? undefined,
    createdAt: new Date(row.created_at),
    startedAt: row.started_at ? new Date(row.started_at) : undefined,
    completedAt: row.completed_at ? new Date(row.completed_at) : undefined,
  };
}

// 将 Waypoint 对象转换为数据库行
function waypointToRow(waypoint: Waypoint, missionId: string) {
  return {
    id: waypoint.id,
    mission_id: missionId,
    index_num: waypoint.index,
    x: waypoint.x,
    y: waypoint.y,
    z: waypoint.z,
    status: waypoint.status,
    completed_at: waypoint.completedAt?.getTime() ?? null,
  };
}

// 将数据库行转换为 Waypoint 对象
function rowToWaypoint(row: WaypointRow): Waypoint {
  return {
    id: row.id,
    index: row.index_num,
    x: row.x,
    y: row.y,
    z: row.z,
    status: row.status as Waypoint['status'],
    completedAt: row.completed_at ? new Date(row.completed_at) : undefined,
  };
}

/**
 * 创建新任务
 */
export function createMission(mission: Mission): void {
  const insertMission = db.prepare(`
    INSERT INTO missions (
      id, name, status, scene_type, scene_url, 
      trajectory_type, trajectory_url, current_waypoint_index,
      created_at, started_at, completed_at
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  `);

  const row = missionToRow(mission);
  insertMission.run(
    row.id, row.name, row.status, row.scene_type, row.scene_url,
    row.trajectory_type, row.trajectory_url, row.current_waypoint_index,
    row.created_at, row.started_at, row.completed_at
  );

  // 插入航点
  if (mission.waypoints && mission.waypoints.length > 0) {
    const insertWaypoint = db.prepare(`
      INSERT INTO waypoints (id, mission_id, index_num, x, y, z, status, completed_at)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    `);

    const insertMany = db.transaction((waypoints: Waypoint[]) => {
      for (const wp of waypoints) {
        const wpRow = waypointToRow(wp, mission.id);
        insertWaypoint.run(
          wpRow.id, wpRow.mission_id, wpRow.index_num,
          wpRow.x, wpRow.y, wpRow.z, wpRow.status, wpRow.completed_at
        );
      }
    });

    insertMany(mission.waypoints);
  }

  // 插入执行日志
  if (mission.executionLog && mission.executionLog.length > 0) {
    const insertLog = db.prepare(`
      INSERT INTO execution_logs (mission_id, timestamp, event, details)
      VALUES (?, ?, ?, ?)
    `);

    const insertLogs = db.transaction((logs: Mission['executionLog']) => {
      for (const log of logs!) {
        insertLog.run(
          mission.id,
          log.timestamp.getTime(),
          log.event,
          log.details ? JSON.stringify(log.details) : null
        );
      }
    });

    insertLogs(mission.executionLog);
  }
}

/**
 * 获取任务（不包含 File 对象的 scene/trajectory）
 */
export function getMissionById(id: string) {
  const mission = db.prepare('SELECT * FROM missions WHERE id = ?').get(id) as MissionRow | undefined;
  if (!mission) return null;

  const waypoints = db.prepare('SELECT * FROM waypoints WHERE mission_id = ? ORDER BY index_num').all(id) as WaypointRow[];
  const logs = db.prepare('SELECT * FROM execution_logs WHERE mission_id = ? ORDER BY timestamp').all(id) as ExecutionLogRow[];

  return {
    ...rowToMission(mission),
    waypoints: waypoints.map(rowToWaypoint),
    executionLog: logs.map((log) => ({
      timestamp: new Date(log.timestamp),
      event: log.event,
      details: log.details ? JSON.parse(log.details) as Record<string, unknown> : undefined,
    })),
  };
}

/**
 * 获取所有任务列表（简化版本）
 */
export function getAllMissions() {
  const missions = db.prepare(`
    SELECT * FROM missions 
    ORDER BY created_at DESC
  `).all() as MissionRow[];

  return missions.map(rowToMission);
}

/**
 * 更新任务
 */
export function updateMission(id: string, updates: Partial<Mission>): void {
  const fields: string[] = [];
  const values: (string | number | null)[] = [];

  if (updates.name !== undefined) {
    fields.push('name = ?');
    values.push(updates.name);
  }
  if (updates.status !== undefined) {
    fields.push('status = ?');
    values.push(updates.status);
  }
  if (updates.scene !== undefined) {
    fields.push('scene_type = ?');
    values.push(updates.scene?.type ?? null);
    fields.push('scene_url = ?');
    values.push(updates.scene?.type === 'url' ? updates.scene.url : null);
  }
  if (updates.trajectory !== undefined) {
    fields.push('trajectory_type = ?');
    values.push(updates.trajectory?.type ?? null);
    fields.push('trajectory_url = ?');
    values.push(updates.trajectory?.type === 'url' ? updates.trajectory.url : null);
  }
  if (updates.currentWaypointIndex !== undefined) {
    fields.push('current_waypoint_index = ?');
    values.push(updates.currentWaypointIndex);
  }
  if (updates.startedAt !== undefined) {
    fields.push('started_at = ?');
    values.push(updates.startedAt?.getTime() ?? null);
  }
  if (updates.completedAt !== undefined) {
    fields.push('completed_at = ?');
    values.push(updates.completedAt?.getTime() ?? null);
  }

  if (fields.length > 0) {
    values.push(id);
    const sql = `UPDATE missions SET ${fields.join(', ')} WHERE id = ?`;
    db.prepare(sql).run(...values);
  }

  // 更新航点（如果提供）
  if (updates.waypoints) {
    // 删除旧航点
    db.prepare('DELETE FROM waypoints WHERE mission_id = ?').run(id);

    // 插入新航点
    const insertWaypoint = db.prepare(`
      INSERT INTO waypoints (id, mission_id, index_num, x, y, z, status, completed_at)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    `);

    const insertMany = db.transaction((waypoints: Waypoint[]) => {
      for (const wp of waypoints) {
        const wpRow = waypointToRow(wp, id);
        insertWaypoint.run(
          wpRow.id, wpRow.mission_id, wpRow.index_num,
          wpRow.x, wpRow.y, wpRow.z, wpRow.status, wpRow.completed_at
        );
      }
    });

    insertMany(updates.waypoints);
  }
}

/**
 * 删除任务
 */
export function deleteMission(id: string): void {
  db.prepare('DELETE FROM missions WHERE id = ?').run(id);
}

/**
 * 添加执行日志
 */
export function addExecutionLog(
  missionId: string,
  event: string,
  details?: Record<string, unknown>
): void {
  db.prepare(`
    INSERT INTO execution_logs (mission_id, timestamp, event, details)
    VALUES (?, ?, ?, ?)
  `).run(
    missionId,
    Date.now(),
    event,
    details ? JSON.stringify(details) : null
  );
}

/**
 * 获取任务统计
 */
export function getMissionStats(): Record<string, number> {
  const stats = db.prepare(`
    SELECT 
      status,
      COUNT(*) as count
    FROM missions
    GROUP BY status
  `).all() as StatsRow[];

  return stats.reduce((acc, row) => {
    acc[row.status] = row.count;
    return acc;
  }, {} as Record<string, number>);
}
