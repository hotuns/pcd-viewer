# 项目技术约定

---

## 技术栈
- **核心:** TypeScript 5 + Next.js 15 (App Router) + React 19
- **可视化:** three.js 0.180 + @react-three/fiber/@react-three/drei
- **实时通信:** roslib.js WebSocket 到 ROSBridge
- **数据库:** better-sqlite3（本地 `data/missions.db`）
- **样式/UI:** Tailwind CSS 4 + shadcn/ui 组件

---

## 开发约定
- **代码规范:** 使用 ESLint (`eslint.config.mjs`) + TypeScript 严格类型，组件默认 `use client`，hooks 置于 `src/hooks/`。
- **命名约定:** React 组件 PascalCase，hooks camelCase 以 `use` 前缀；数据库文件 snake_case 与 SQL 保持一致。
- **目录划分:** `src/app` 存放路由，`src/components` 拆分 Startup/Mission/PCD 模块，`src/lib` 存放 db/ROS/解析工具，`src/types` 作为跨层类型定义。

---

## 错误与日志
- **策略:** Mission/ROS 相关异常写入浏览器控制台，`useMissionRuntime` 通过 `events` 队列反馈给 UI；API 层所有 errors 记录在 server 控制台。
- **日志:** SQLite DAO 仅在 `initDatabase` 打印初始化路径；若需要持久化日志，可扩展 `execution_logs` 表或追加 `history/` 记录。

---

## 测试与流程
- **测试:** 当前以手动流程为主——使用 `docs/ROS_TEST_GUIDE.md` 连接仿真 ROSBridge，回归启动页→MissionDetail→ROS 运行。
- **提交:** 采用 Conventional Commits，先运行 `npm run lint` 确认通过；若修改数据库结构需同步更新 `docs/DATABASE_GUIDE.md` 与 `wiki/data.md`。
