# 组件拆分说明

## 拆分概览

为了提高代码可维护性,将原本集中在几个大文件中的功能进行了适当拆分:

## 新建文件

### 1. Hooks

#### `src/hooks/useRosConnection.ts`
- **功能**: ROS WebSocket 连接管理
- **导出**: `useRosConnection` hook
- **状态管理**: 
  - `rosUrl` - ROS服务器地址
  - `rosConnected` - 连接状态
  - `rosRef` - ROSLIB.Ros 实例引用
- **方法**:
  - `connectROS()` - 建立连接
  - `disconnectROS()` - 断开连接

### 2. 工具函数

#### `src/lib/pointCloudParser.ts`
- **功能**: ROS PointCloud2 消息解析
- **导出函数**:
  - `parsePointCloud2Data()` - 解析 Base64 编码的点云二进制数据
  - `createGridMapDataFromPointCloud2()` - 从 PointCloud2 创建 GridMapData

### 3. 3D 渲染组件

#### `src/components/pcd/DroneModel.tsx`
- **功能**: 无人机 GLTF 3D 模型渲染
- **Props**: `position`, `orientation`
- **特性**: 材质优化、阴影支持、预加载

#### `src/components/pcd/PointCloudMap.tsx`
- **功能**: 点云地图 Three.js 渲染
- **Props**: `gridMapData`
- **特性**: BufferGeometry 优化、透明绿色点云显示

#### `src/components/pcd/CameraFollower.tsx`
- **功能**: 相机跟随无人机
- **Props**: `dronePosition`, `followDrone`, `controlsRef`
- **特性**: 第三人称视角、平滑插值(lerp)

## 更新的文件

### `src/components/pcd/PCDCanvas.tsx`
- **变化**: 
  - ✅ 删除了内部定义的 `DroneModel`, `PointCloudMap`, `CameraFollower` 组件
  - ✅ 改为导入独立组件
  - ✅ 减少约 100 行代码

### `src/components/pcd/MissionController.tsx`
- **变化**:
  - ✅ 删除了 ROS 连接管理逻辑 (约 50 行)
  - ✅ 使用 `useRosConnection` hook
  - ✅ 使用 `createGridMapDataFromPointCloud2` 工具函数
  - ✅ 删除了手动 Base64 解码和点云解析逻辑 (约 80 行)
  - ✅ 添加了 `handleDisconnect` 包装函数统一管理订阅清理
  - ✅ 总共减少约 130 行代码

## 优势

1. **职责分离**: 每个文件职责更清晰
   - Hooks 管理状态和连接
   - 工具函数处理数据解析
   - 组件专注于渲染

2. **可复用性**: 
   - `useRosConnection` 可用于其他需要 ROS 连接的组件
   - `parsePointCloud2Data` 可在不同场景复用
   - 3D 组件可独立测试和优化

3. **可测试性**: 
   - 工具函数更容易单元测试
   - 组件可以独立测试
   - Mock 更简单

4. **代码量**: 
   - `PCDCanvas.tsx`: 从 882 行 → 约 780 行
   - `MissionController.tsx`: 从 1315 行 → 约 1180 行
   - 总体代码更清晰,虽然文件数增加但单文件更易维护

## 保持原有功能

✅ 所有功能保持不变:
- ROS 连接和断开
- 点云地图解析和渲染
- 无人机模型显示
- 相机跟随功能
- 任务控制逻辑

## 未来可继续优化

如果需要进一步拆分,可以考虑:
- 将 `MissionController.tsx` 的任务状态管理提取为 hook
- 将航线编辑逻辑提取为独立组件
- 将 UI 控制面板提取为独立组件

但目前的拆分已经达到了"适当"的程度,避免了过度工程化。
