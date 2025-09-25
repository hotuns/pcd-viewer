## PCD Viewer v0.1

一个基于 Next.js + three.js (@react-three/fiber, @react-three/drei) 的点云（PCD）查看与轨迹演示小工具，深色专业主题。

### 功能概览

- 加载点云
	- 本地上传 .pcd 文件或输入 URL 加载
	- 自动视角适配（fit to view）、点大小调节、网格/坐标轴开关
	- 点云着色模式：原始色、按高度、按强度、按分类
- 轨迹展示（本地坐标系）
	- 规划轨迹：加载 JSON/CSV 的 XYZ 路径并以折线展示
	- 实时轨迹：可通过前端模拟“沿规划路径飞行”，并绘制尾迹
	- 无人机可视化：机体标记、可选航空姿态（转弯滚转效果）、速度箭头、面包屑尾迹（高效 Instances）
- 性能辅助
	- 可选 Stats HUD + Renderer 信息面板

注：为保障性能，已移除标绘/测量等重型交互。

### 运行

```bash
npm run dev
```

打开浏览器访问 http://localhost:3000

### 使用

1) 点云
- 通过「选择文件」或输入 URL 加载 .pcd 文件。
- 在侧边栏可切换着色模式（原始/高度/强度/分类）、调整点大小、开关网格与坐标轴。

2) 轨迹
- 规划轨迹：在侧边栏加载 JSON 或 CSV（字段：x,y,z[,t]）。示例见 `public/example-planned-path.json`。
- 实时轨迹：默认使用前端模拟，沿规划路径重放，便于对齐对比；也保留 WebSocket 接入的扩展位（未默认启用）。
- 无人机标记：支持航空姿态（转弯滚转）、速度箭头与面包屑尾迹。

### 文件结构（关键）

- `src/components/pcd/PCDCanvas.tsx`：3D 场景（点云、着色、轨迹、UAV 可视化、性能 HUD）
- `src/components/pcd/PCDViewer.tsx`：UI 与状态（加载、模式切换、轨迹管理、前端模拟）
- `src/app/api/telemetry/ws/route.ts`：Edge Runtime WebSocket 路由（保留可选）
- `public/example-planned-path.json`：示例规划轨迹

### 已知限制（v0.1）

- 以性能为先：无标绘/测量等重交互工具
- 强度/高度着色为“当前数据自适应归一化”，跨数据集对比需谨慎；可后续支持固定阈值
- WebSocket 实时数据管道留有接口，但默认关闭；当前以纯前端定时器模拟为主
- PCD 二进制格式兼容性依赖 three.js 的 PCDLoader，不同导出器可能存在差异（建议优先 ASCII）

### 路线图（后续）

- UI 开关与参数：航空姿态、速度箭头、面包屑数量/长度、模拟速度与循环模式
- 可选固定色阶（强度/高度）与 LUT 配置
- 更多性能选项（DPR、LOD、点大小随距调整等）

