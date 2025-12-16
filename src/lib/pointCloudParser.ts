/**
 * 解析 ROS PointCloud2 消息中的点云数据
 * @param data Base64 编码的二进制点云数据
 * @param pointStep 每个点的字节步长
 * @param rowStep 每行的字节步长
 * @param width 点云宽度
 * @param height 点云高度
 * @returns Float32Array 格式的点云数据 [x1, y1, z1, x2, y2, z2, ...]
 */
export function parsePointCloud2Data(
  data: string,
  pointStep: number,
  rowStep: number,
  width: number,
  height: number
): Float32Array {
  // Base64 解码
  const binaryString = atob(data);
  const len = binaryString.length;
  const bytes = new Uint8Array(len);
  
  for (let i = 0; i < len; i++) {
    bytes[i] = binaryString.charCodeAt(i);
  }

  // 解析点云数据
  const dataView = new DataView(bytes.buffer);
  const totalPoints = width * height;
  const points: number[] = [];

  for (let i = 0; i < totalPoints; i++) {
    const offset = i * pointStep;
    
    // 假设 XYZ 是前 12 个字节（3 个 float32）
    try {
      const x = dataView.getFloat32(offset, true); // little-endian
      const y = dataView.getFloat32(offset + 4, true);
      const z = dataView.getFloat32(offset + 8, true);

      // 过滤无效点（NaN 或 Infinity）
      if (isFinite(x) && isFinite(y) && isFinite(z)) {
        points.push(x, y, z);
      }
    } catch {
      // 忽略解析错误的点
      continue;
    }
  }

  return new Float32Array(points);
}

/**
 * 从 ROS PointCloud2 消息创建 GridMapData
 */
export function createGridMapDataFromPointCloud2(msg: {
  header: { frame_id: string };
  data: string;
  point_step: number;
  row_step: number;
  width: number;
  height: number;
}) {
  const parsedData = parsePointCloud2Data(
    msg.data,
    msg.point_step,
    msg.row_step,
    msg.width,
    msg.height
  );

  const pointCount = parsedData.length / 3;
  console.log(`Parsed ${pointCount} valid points from PointCloud2`);

  return {
    frameId: msg.header.frame_id,
    data: parsedData,
    pointCount,
  };
}
