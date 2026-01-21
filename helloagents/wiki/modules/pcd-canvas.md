# PCD Canvas & Editor

## 目的
封装点云渲染、航线可视化/编辑、无人机模型展示（`PCDCanvas.tsx` + 子组件）。

## 模块概述
- **职责:** 加载 PCD/PLY 文件、渲染 plannedPath、支持 TransformControls 拖拽航点、显示实时无人机/PointCloud2 栈。
- **状态:** 🚧开发中
- **最后更新:** 2025-12-24

### 坐标轴约定
- Three.js 仍使用默认右手坐标（红 X→右、绿 Y→上、蓝 Z→朝观察者）。
- 通过 `AxisLabels(mode="ros")` 在轴标签中提示与 ROS ENU 机体系的对应关系：红轴=ROS -Y（右/左）、绿轴=ROS Z（上/下）、蓝轴=ROS -X（后/前）。
- `RosAxes` 仅渲染 Three.js 默认方向的参考线；所有坐标变换继续由 `frameTransforms.ts` 负责。

## 规范

### 需求: 点云加载
**模块:** PCD Canvas  
支持静态场景与实时 PointCloud2 叠加。

#### 场景: 静态场景
- `source` 可为 URL/File；使用 PCDLoader/PLYLoader 完成加载，期间调用 `onLoadingChange(true/false)`。
- 加载完成后触发 `onLoadedAction({bbox,count})`，计算 bounding box 供视图适配。
- 读取 ROS 坐标后立即通过 `convertBodyPositionToViewer` 转换为 Three.js 视图坐标，确保与 `/tools/orientation` 一致；法线同步旋转。
- Mission 页面额外提供“加载点云”按钮，可临时覆盖任务场景文件；再次点击“使用任务点云”即可恢复。

#### 场景: 实时点云
- `livePointClouds`（Float32Array[]）最新帧排在数组前端；最多保留若干帧并设置不同 opacity。
- 卸载组件时需 dispose BufferGeometry。

### 需求: 航点渲染与编辑
**模块:** PCD Canvas  
plannedPathPoints 在 3D 视图中表现为 Line + Sphere。

#### 场景: 仅查看
- 受控属性 `plannedPathVisible`、`plannedPointSize` 控制显示。
- 新增 `plannedPathLineWidth` 控制线条粗细，默认 2，可由 MissionController 实时调整。
- 不可编辑时禁用 TransformControls。

#### 场景: 编辑模式
- `plannedPathEditable=true` 时启用 TransformControls，拖拽更新点位并回调 `onPlannedPointsChange`。
- `selectedPointIndex` 控制选中高亮；`onSelectPoint` 响应点击事件。
- `PCDCanvasHandle` 暴露 `fitToView / zoomToCenter / orientToPlane`，供 MissionController 的按钮使用。

### 需求: 无人机跟随
**模块:** PCD Canvas  
实时显示无人机模型（GLTF）和航点状态。

#### 场景: Follow 模式
- 设置 `followDrone=true` 时相机目标自动跟随 DronePosition。
- 无人机 orientation 由四元数 (x,y,z,w) 提供。

#### 场景: 航点状态染色
- 根据 `waypoints` + `currentWaypointIndex` 渲染不同颜色：pending/active/completed/skipped。

## API接口
- 内部不直接访问 HTTP；仅透传 `onPlannedPointsChange`、`onSelectPoint` 等回调给父组件。

## 数据模型
- `plannedPathPoints`: `PlannedPoint[]`（x/y/z/w 以及 t、task_type、info 字段，定义于 `src/types/mission.ts`）
- `livePointClouds`: `Float32Array[]`（每三个 float 代表坐标）
- `DronePosition`: 参照 `src/types/mission.ts`

## 依赖
- three.js, @react-three/fiber, @react-three/drei (OrbitControls/Grid/TransformControls)
- 自研组件：`DroneModel`, `CameraFollower`, `VoxelizedPointCloud`
- 外部 loader：`three/examples/jsm/loaders/PCDLoader`, `PLYLoader`

## 变更历史
- 2025-12-24 创建模块文档，标注视图控制与编辑约束。
- Mission 页面右上角提供 XY/XZ/YZ/45° 视角预设按钮（复用 `/tools/orientation` 逻辑），可快速切换顶视/侧视/等角视角。
