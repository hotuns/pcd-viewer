# Startup Launcher

## 目的
提供任务启动页（StartupPage），集中创建/筛选任务、查看统计并跳转 MissionDetail。

## 模块概述
- **职责:** 读取 SQLite 中的任务列表及状态统计，支持创建/删除、筛选/排序、卡片视图与列表视图切换，并在选择后调用路由跳转。
- **状态:** ✅稳定
- **最后更新:** 2025-12-24

## 规范

### 需求: 任务创建与跳转
**模块:** Startup Launcher  
在启动页输入任务名并创建，或在卡片列表中选择已有任务。

#### 场景: 创建新任务
- 空名称禁止提交，提示“请输入任务名称”。
- 成功保存后刷新任务列表与统计，自动清空输入框。

#### 场景: 打开已有任务
- 选中卡片并点击“打开任务”，调用 `onMissionSelect` 将 Mission 传给 `/mission/[id]` 路由。
- 任务卡片需展示场景/航线配置状态徽章与人类可读时间。

### 需求: 状态过滤与视图切换
**模块:** Startup Launcher  
支持 `状态筛选` + `卡片/列表` 两种视图。

#### 场景: 状态筛选
- 默认值 `all`，切换后仅显示目标状态任务；统计面板仍展示全量。
- 重新加载列表（`getAllMissions`）后保持筛选值。

#### 场景: 视图切换
- Card 模式显示图文卡片、List 模式显示行式表格，两者共用数据源。
- 需记住用户选择（`viewMode` state）；若要长期记忆可扩展 localStorage。

## API接口

### GET /api/missions
- StartupPage 初始化调用，返回任务数组供排序/渲染。

### GET /api/missions?stats=true
- 提供状态数量统计，映射至 `StatsRow`。

### POST /api/missions
- 保存新任务；失败时弹出 alert。

### DELETE /api/missions/:id
- 删除任务及其航点；删除当前选中任务后应清空 `selectedMissionId`。

## 数据模型
- 使用 `MissionRow` 作为 UI 层数据（字符串时间），在导航前转换成 `Mission`（Date 对象）。
- stats 结构：`{ total, draft, configured, ... }`。

## 依赖
- `useMissionDatabase` Hook（封装 fetch 调用）
- `lucide-react` 图标、shadcn/ui `Card/Button/Input/Badge`
- `next/navigation` 用于 `router.push`

## 变更历史
- 2025-12-24 初始化文档，覆盖 v0.1.0 功能。*** End Patch*** End Patch
