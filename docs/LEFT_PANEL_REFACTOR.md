# 左侧交互区重构说明

## 🎯 重构目标

将混乱的左侧控制面板重新组织为清晰的、按任务状态区分的区域，并正确管理任务状态变更和数据库同步。

## 📋 新组件结构

### 1. **TaskInfo.tsx** - 任务信息组件
- 位置：顶部固定区域
- 内容：任务名称、状态徽章、配置摘要
- 特点：紧凑显示，一目了然

### 2. **TaskConfigPanel.tsx** - 任务配置面板
- 根据任务状态动态变化
- 包含文件上传和URL输入
- 自动状态转换逻辑
- **数据库同步**：每次更新都同步到 SQLite

## 🔄 状态转换流程

```
draft (草稿)
  ↓ [上传场景 + 航线]
configured (已配置)
  ↓ [开始规划]
planning (规划中)
  ↓ [确认航线]
ready (就绪)
  ↓ [开始任务]
running (执行中)
  ↓ [任务结束]
completed/failed (完成/失败)
```

## 🎨 左侧面板布局

```
┌─────────────────────────┐
│  📍 任务信息 (固定顶部)  │
├─────────────────────────┤
│                         │
│  📋 可滚动内容区:       │
│                         │
│  1. 任务配置            │
│     - draft/configured: │
│       文件上传          │
│     - planning:         │
│       编辑提示          │
│     - ready:            │
│       就绪提示          │
│                         │
│  2. 执行控制 (ready时)  │
│     - 开始/停止按钮     │
│                         │
│  3. 执行状态 (running)  │
│     - 进度条            │
│     - 无人机位置        │
│     - ROS 消息状态      │
│                         │
│  4. 场景显示设置        │
│     - 网格/坐标轴       │
│     - 点云大小          │
│     - 体素化            │
│     - 着色模式          │
│                         │
│  5. 航线显示设置        │
│     - 显示/隐藏         │
│     - 航点大小          │
│                         │
│  6. 实时点云地图        │
│     (仅 running 时显示) │
│                         │
│  7. 场景信息            │
│                         │
├─────────────────────────┤
│  🎯 视图适配 (固定底部)  │
└─────────────────────────┘
```

## ✅ 已完成的改进

1. **清晰的区域划分**
   - 顶部：任务概况
   - 中部：根据状态变化的控制区
   - 底部：通用操作

2. **状态驱动的UI**
   - draft/configured: 显示文件配置
   - planning: 显示编辑提示
   - ready: 显示执行按钮
   - running: 显示实时状态

3. **数据库集成**
   - `handleMissionUpdate` 自动同步到 SQLite
   - 每次状态变更都持久化
   - 不再依赖 localStorage

4. **简化的组件**
   - `TaskInfo`: 紧凑的任务信息显示
   - `TaskConfigPanel`: 智能的配置界面

## 🔧 使用方法

在 MissionController.tsx 中:

```tsx
import { TaskInfo } from "./TaskInfo";
import { TaskConfigPanel } from "./TaskConfigPanel";

// 在 return 中使用
<aside className="...">
  {/* 顶部固定 */}
  <div className="p-4 border-b">
    <TaskInfo selectedMission={selectedMission} />
  </div>

  {/* 可滚动中间区 */}
  <div className="flex-1 overflow-y-auto p-4">
    <TaskConfigPanel 
      selectedMission={selectedMission}
      onMissionUpdate={handleMissionUpdate}
    />
    {/* ... 其他控制区 ... */}
  </div>

  {/* 底部固定 */}
  <div className="p-4 border-t">
    <Button>视图适配</Button>
  </div>
</aside>
```

## 📊 状态管理最佳实践

1. **单一数据源**：`selectedMission` 是唯一的真实来源
2. **自动同步**：`handleMissionUpdate` 负责数据库更新
3. **UI响应**：根据 `selectedMission.status` 渲染不同UI
4. **清理逻辑**：状态转换时自动设置编辑模式等

## 🚀 下一步优化建议

1. 将 PCDCanvas 的显示设置提取为独立组件
2. 将 ROS 连接控制提取为独立组件
3. 添加任务执行日志显示组件
4. 添加任务历史/统计视图
