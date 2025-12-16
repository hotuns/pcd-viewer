import * as THREE from "three";

/**
 * 体素数据结构
 */
export interface VoxelData {
  position: THREE.Vector3;
  color?: THREE.Color;
  intensity?: number;
  pointIndices: number[]; // 该体素内包含的原始点索引
}

/**
 * 体素化点云 - 将点云空间划分为体素网格
 * @param geometry 点云几何体
 * @param voxelSize 体素大小（米）
 * @returns 体素数据数组
 */
export function voxelizePointCloud(
  geometry: THREE.BufferGeometry,
  voxelSize: number = 0.05
): VoxelData[] {
  const positions = geometry.getAttribute('position');
  const colors = geometry.getAttribute('color');
  const intensity = geometry.getAttribute('intensity');
  
  if (!positions) return [];

  // 使用 Map 来存储唯一的体素
  const voxelMap = new Map<string, VoxelData>();

  for (let i = 0; i < positions.count; i++) {
    const x = positions.getX(i);
    const y = positions.getY(i);
    const z = positions.getZ(i);

    // 计算点所在的体素索引
    const voxelX = Math.floor(x / voxelSize);
    const voxelY = Math.floor(y / voxelSize);
    const voxelZ = Math.floor(z / voxelSize);

    // 创建唯一键
    const key = `${voxelX},${voxelY},${voxelZ}`;

    // 如果这个体素还没有被记录，创建新体素
    if (!voxelMap.has(key)) {
      const centerX = (voxelX + 0.5) * voxelSize;
      const centerY = (voxelY + 0.5) * voxelSize;
      const centerZ = (voxelZ + 0.5) * voxelSize;
      
      voxelMap.set(key, {
        position: new THREE.Vector3(centerX, centerY, centerZ),
        pointIndices: [i],
      });
    } else {
      // 添加点索引到现有体素
      voxelMap.get(key)!.pointIndices.push(i);
    }
  }

  // 计算每个体素的平均颜色和强度
  const voxels = Array.from(voxelMap.values());
  
  for (const voxel of voxels) {
    if (colors) {
      // 计算该体素内所有点的平均颜色
      let r = 0, g = 0, b = 0;
      for (const idx of voxel.pointIndices) {
        r += colors.getX(idx);
        g += colors.getY(idx);
        b += colors.getZ(idx);
      }
      const count = voxel.pointIndices.length;
      voxel.color = new THREE.Color(r / count, g / count, b / count);
    }
    
    if (intensity) {
      // 计算该体素内所有点的平均强度
      let sum = 0;
      for (const idx of voxel.pointIndices) {
        sum += intensity.getX(idx);
      }
      voxel.intensity = sum / voxel.pointIndices.length;
    }
  }

  return voxels;
}

/**
 * 从体素中心点创建实例化网格数据
 * @param voxels 体素数据数组
 * @returns 位置矩阵数组
 */
export function createVoxelInstances(voxels: VoxelData[]): THREE.Matrix4[] {
  return voxels.map(v => {
    const matrix = new THREE.Matrix4();
    matrix.setPosition(v.position);
    return matrix;
  });
}

/**
 * 获取体素颜色
 * @param voxels 体素数据数组
 * @param colorMode 着色模式
 * @param geometry 原始几何体（用于获取原始颜色/强度属性）
 * @returns 颜色数组
 */
export function getVoxelColors(
  voxels: VoxelData[],
  colorMode: "none" | "rgb" | "intensity" | "height"
): THREE.Color[] {
  if (voxels.length === 0) return [];

  console.log(`Getting voxel colors in mode: ${colorMode}, voxel count: ${voxels.length}`);

  // 无着色模式
  if (colorMode === "none") {
    console.log('Using none mode - returning gray colors');
    return voxels.map(() => new THREE.Color(0.8, 0.8, 0.8));
  }

  // RGB 模式：使用体素的平均颜色
  if (colorMode === "rgb") {
    console.log('Using RGB mode');
    console.log(`Voxels with colors: ${voxels.filter(v => v.color).length}/${voxels.length}`);
    return voxels.map(v => v.color || new THREE.Color(0.8, 0.8, 0.8));
  }

  // 强度模式：基于平均强度
  if (colorMode === "intensity") {
    console.log('Using intensity mode');
    const hasIntensity = voxels.some(v => v.intensity !== undefined);
    console.log(`Voxels with intensity: ${voxels.filter(v => v.intensity !== undefined).length}/${voxels.length}`);
    
    if (!hasIntensity) {
      console.warn('No intensity data found, falling back to gray');
      return voxels.map(() => new THREE.Color(0.5, 0.5, 0.5));
    }
    
    // 找到最小和最大强度
    let minIntensity = Infinity;
    let maxIntensity = -Infinity;
    
    for (const v of voxels) {
      if (v.intensity !== undefined) {
        if (v.intensity < minIntensity) minIntensity = v.intensity;
        if (v.intensity > maxIntensity) maxIntensity = v.intensity;
      }
    }
    
    const span = maxIntensity - minIntensity || 1;
    
    return voxels.map(v => {
      if (v.intensity === undefined) return new THREE.Color(0.5, 0.5, 0.5);
      const normalized = (v.intensity - minIntensity) / span;
      return mapScalarToColor(normalized);
    });
  }

  // 高度模式：基于 Z 坐标
  if (colorMode === "height") {
    console.log('Using height mode');
    // 找到最小和最大高度
    let minZ = Infinity;
    let maxZ = -Infinity;
    for (const v of voxels) {
      if (v.position.z < minZ) minZ = v.position.z;
      if (v.position.z > maxZ) maxZ = v.position.z;
    }

    const spanZ = maxZ - minZ || 1;
    console.log(`Height range: ${minZ.toFixed(2)} to ${maxZ.toFixed(2)}, span: ${spanZ.toFixed(2)}`);

    return voxels.map(v => {
      const normalized = (v.position.z - minZ) / spanZ;
      return mapScalarToColor(normalized);
    });
  }

  // 默认灰色
  console.log('Falling back to default gray');
  return voxels.map(() => new THREE.Color(0.8, 0.8, 0.8));
}

/**
 * 颜色渐变函数：蓝-青-绿-黄-红
 */
function mapScalarToColor(v01: number): THREE.Color {
  const v = Math.min(1, Math.max(0, v01));
  let r = 0, g = 0, b = 0;
  
  if (v < 0.25) {
    const t = v / 0.25;
    r = 0; g = t; b = 1;
  } else if (v < 0.5) {
    const t = (v - 0.25) / 0.25;
    r = 0; g = 1; b = 1 - t;
  } else if (v < 0.75) {
    const t = (v - 0.5) / 0.25;
    r = t; g = 1; b = 0;
  } else {
    const t = (v - 0.75) / 0.25;
    r = 1; g = 1 - t; b = 0;
  }
  
  return new THREE.Color(r, g, b);
}
