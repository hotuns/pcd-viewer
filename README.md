## UAV Mission Manager v0.2

一个基于 Next.js + three.js (@react-three/fiber, @react-three/drei) 的无人机任务管理小工具，支持创建任务并加载场景点云，同时通过 roslibjs 订阅无人机实时位置，采用深色专业主题。

### 功能概览

- 任务管理
	- 创建任务；为任务配置场景（点云 .pcd）与航线轨迹（JSON/CSV或URL）
	- 场景支持本地文件或 URL；加载后自动视角适配（fit to view）
- ROS 实时位置
	- 通过 roslibjs 连接 ROSBridge（如 ws://host:port）
	- 订阅 /mavros/local_position/odom，显示实时位置与速度信息
	- 仅显示实时数据，不在三维视图中绘制航迹

注：为对齐新目标，已移除轨迹显示、模拟回放、着色模式与性能面板等扩展功能，仅保留任务与实时位置。

### 运行

```bash
npm run dev
```

打开浏览器访问 http://localhost:3000

### 使用

1) 创建任务
- 在侧栏创建任务并选择：
	- 场景（点云 .pcd）：上传文件或填写 URL
	- 航线轨迹：上传 JSON/CSV 或填写 URL（当前不绘制，仅保存）

2) ROS 实时位置
- 在侧栏填写 ROSBridge 地址（如 ws://101.132.193.179:7001），点击连接
- 点击「开始任务」后，将通过 roslibjs 订阅 /mavros/local_position/odom 并在侧栏显示实时坐标与速度

### 文件结构（关键）

- `src/components/pcd/PCDCanvas.tsx`：3D 场景（点云加载 + 视角适配 + 基本显示）
- `src/components/pcd/MissionController.tsx`：任务创建/选择、ROS 连接与实时位置订阅、基本显示设置
- `src/components/mission/MissionManager.tsx`：任务增删改与资源（场景/航线）配置
- `src/app/page.tsx`：首页入口

### 已知限制（v0.2）

- 当前不绘制航迹，仅显示实时位置数据
- PCD 二进制格式兼容性依赖 three.js 的 PCDLoader，不同导出器可能存在差异（建议优先 ASCII）
- ROS 话题与消息类型需按实际环境调整（默认 /mavros/local_position/odom, nav_msgs/msg/Odometry）


