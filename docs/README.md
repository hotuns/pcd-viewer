# 文档索引

所有补充说明文档已经搬迁至 `docs/` 目录，方便集中维护。下面按照主题分类列出当前可用的指南：

## 架构与界面
- [INTERACTION_FLOW.md](./INTERACTION_FLOW.md)：启动页与任务执行流程的完整交互说明。
- [LEFT_PANEL_REFACTOR.md](./LEFT_PANEL_REFACTOR.md)：左侧控制区重构目标、组件划分与状态变更策略。
- [REFACTORING_SUMMARY.md](./REFACTORING_SUMMARY.md)：近期组件拆分与新增 Hook 的概览。

## 数据与存储
- [DATABASE_GUIDE.md](./DATABASE_GUIDE.md)：SQLite 表结构、迁移脚本以及 `better-sqlite3` 的使用方法。

## ROS 集成
- [ROS_TOPICS_SUMMARY.md](./ROS_TOPICS_SUMMARY.md)：当前订阅的 ROS 话题及后续需求。
- [ROS_TEST_GUIDE.md](./ROS_TEST_GUIDE.md)：端到端任务模拟与测试步骤。
- [test_ros_connection.md](./test_ros_connection.md)：如何在真实 ROS 环境中验证连接与话题数据。
- [ros.md](./ros.md)：常用 ROS 启动命令与环境准备速查。

## 渲染与可视化
- [VOXEL_RENDERING_GUIDE.md](./VOXEL_RENDERING_GUIDE.md)：体素化点云渲染的设计思路与调优参数。

如需新增文档，请直接在 `docs/` 中创建文件，并在本索引中登记，保持结构清晰。
