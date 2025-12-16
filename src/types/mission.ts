/**
 * 任务配置类型定义
 */

export type FileSource = {
  type: "file";
  file: File;
};

export type URLSource = {
  type: "url";
  url: string;
};

export type Source = FileSource | URLSource;

// 任务状态枚举
export type MissionStatus = 
  | 'draft'      // 草稿：刚创建，需要配置场景和航线
  | 'configured' // 已配置：场景和航线都已设置
  | 'planning'   // 规划中：正在编辑航线
  | 'ready'      // 就绪：航线确认，等待执行
  | 'running'    // 执行中：任务正在进行
  | 'paused'     // 暂停：无人机暂停或充电
  | 'completed'  // 完成：任务执行完毕
  | 'failed';    // 失败：任务执行失败

// Missionlogic 运行阶段
export type MissionPhase =
  | 'idle'             // 未连接或未准备
  | 'hangar_ready'     // 机库 ready，可执行上传
  | 'mission_upload'   // 正在上传任务
  | 'takeoff_ready'    // 起飞前检查完成
  | 'takeoff'          // 起飞中
  | 'executing'        // 任务执行中
  | 'returning'        // 返航中
  | 'landing'          // 降落中
  | 'charging'         // 充电中
  | 'error';           // 异常

export interface MissionHomePosition {
  frameId: string;
  position: {
    x: number;
    y: number;
    z: number;
  };
  yaw: number;
}

export interface HangarChargeTelemetry {
  status: number;
  percentage: number;
  power: number;
  duration: number;
  error?: string;
  updatedAt: Date;
}

export interface BatteryTelemetry {
  percentage?: number;
  voltage?: number;
  remaining?: number;
  updatedAt?: Date;
}

export interface MissionRuntimeEvent {
  id: string;
  timestamp: Date;
  level: 'info' | 'warning' | 'error';
  message: string;
  details?: Record<string, unknown>;
}

export interface MissionRuntimeSnapshot {
  phase: MissionPhase;
  progress?: {
    completed: number;
    total: number;
  };
  lastWaypoint?: {
    index: number;
    info?: string;
    position?: { x: number; y: number; z: number };
  };
}

// 航点状态枚举
export type WaypointStatus = 
  | 'pending'    // 待执行
  | 'active'     // 当前目标点
  | 'completed'  // 已完成
  | 'skipped';   // 已跳过

// 航点扩展信息
export interface Waypoint {
  x: number;
  y: number;
  z: number;
  id: string;
  status: WaypointStatus;
  completedAt?: Date;
  index: number;
}

export interface Mission {
  id: string;
  name: string;
  scene?: Source;  // 点云场景
  trajectory?: Source;  // 航线轨迹
  home?: MissionHomePosition;
  status: MissionStatus;
  waypoints?: Waypoint[];  // 航点状态信息
  currentWaypointIndex?: number;  // 当前目标航点索引
  createdAt: Date;
  startedAt?: Date;
  completedAt?: Date;
  executionLog?: Array<{
    timestamp: Date;
    event: string;
    details?: Record<string, unknown>;
  }>;
}

export interface DronePosition {
  x: number;
  y: number;
  z: number;
  orientation?: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
  velocity?: {
    x: number;
    y: number;
    z: number;
  };
}

// ROS消息类型
export interface WaypointReachedMessage {
  waypoint_index: number;
  timestamp: number;
  position: {
    x: number;
    y: number;
    z: number;
  };
}

export interface MissionCompleteMessage {
  mission_id: string;
  status: 'completed' | 'failed' | 'returned';
  timestamp: number;
  reason?: string;
}

// 实时点云帧数据
export interface PointCloudFrame {
  id: string;
  timestamp: Date;
  points: Array<{
    x: number;
    y: number;
    z: number;
    intensity?: number;
    r?: number;
    g?: number;
    b?: number;
  }>;
  frameId: string;  // ROS frame_id
}

// 网格地图数据 (实际来自 PointCloud2)
export interface GridMapData {
  timestamp: Date;
  width: number;
  height: number;
  resolution: number;  // 米/像素 (对于 PointCloud2 可能需要估算)
  origin: {
    x: number;
    y: number;
    z: number;
  };
  data: number[];  // 对于 OccupancyGrid 是占用概率；对于 PointCloud2 是原始二进制数据
  frameId?: string;  // ROS frame_id
  pointCount?: number;  // PointCloud2 的点数
}
