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

export interface Mission {
  id: string;
  name: string;
  scene?: Source;  // 点云场景
  trajectory?: Source;  // 航线轨迹
  createdAt: Date;
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
