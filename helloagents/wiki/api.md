# API 手册

## 概述
所有 API 由 Next.js App Router 提供，默认以 JSON 交互，无需额外认证。客户端通过 `fetch` 或 `useMissionDatabase` Hook 间接访问。

## 认证方式
当前仅限本地开发环境，未实现鉴权；部署到公网前需引入 Token 或网关校验。

---

## 接口列表

### Mission 资源

#### GET /api/missions
**描述:** 获取所有任务（按创建时间倒序）。

**请求参数:** 无

**响应:**
```json
{
  "missions": [
    {
      "id": "1734950000000",
      "name": "demo",
      "status": "draft",
      "sceneUrl": "https://...",
      "trajectoryUrl": "https://...",
      "createdAt": "2025-12-24T07:40:00.000Z"
    }
  ]
}
```

#### GET /api/missions?stats=true
**描述:** 返回每个状态的任务数量，供启动页统计卡片使用。

**响应字段:**
| 字段 | 类型 | 说明 |
|------|------|------|
| stats | object | 例如 `{ "draft": 2, "running": 1 }` |

#### POST /api/missions
**描述:** 新建任务，服务器会补全 `id/createdAt` 等字段。

**请求参数:**
| 参数名 | 类型 | 必填 | 说明 |
|--------|------|------|------|
| id | string | 否 | 不传则由服务器使用时间戳 |
| name | string | 否 | 默认 `未命名任务` |
| status | string | 否 | 默认 `draft` |
| scene | {type:"url",url:string} | 否 | 仅支持 URL 来源 |
| trajectory | {type:"url",url:string} | 否 | 仅支持 URL 来源 |
| waypoints | Waypoint[] | 否 | 可选航点表 |

**响应:**
```json
{
  "success": true,
  "missionId": "1734950000000",
  "message": "Mission created successfully"
}
```

#### GET /api/missions/:id
**描述:** 返回指定任务的完整信息（含航点和执行日志）。

**响应片段:**
```json
{
  "mission": {
    "id": "1734950000000",
    "status": "ready",
    "waypoints": [{ "index": 0, "x": 1, "status": "pending" }],
    "executionLog": [
      { "timestamp": "2025-12-24T08:00:00.000Z", "event": "waypoint_reached" }
    ]
  }
}
```

#### PUT /api/missions/:id
**描述:** 按需更新任务，支持 status/home/scene/trajectory/waypoints 等字段。

**请求体:** `Partial<Mission>`（时间字段可接受 ISO 字符串或毫秒时间戳）

**响应:** `{ "success": true }`

#### DELETE /api/missions/:id
**描述:** 删除任务并级联删除航点/日志。

**响应:** `{ "success": true }`

#### POST /api/missions/:id
**描述:** 针对指定 ID 完整写入任务（用于导入/迁移）。

**请求体:** `Mission` 对象；服务器会调用 `createMission` 直接落库。

---

### Telemetry WebSocket

#### GET /api/telemetry/ws?simulate=1
**描述:** Edge Runtime WebSocket，返回简易三维轨迹点或回显客户端消息，用于无 ROS 环境下调试。

**连接方式:** 发送 `Upgrade: websocket`；`simulate=1` 时服务器每 100ms 推送 `{x,y,z,t}` JSON 文本。

**注意事项:**
- 该接口主要用于调试/展示，不与 `useMissionRuntime` 的 roslib 主流程冲突。
- 生产环境可扩展为真实遥测数据中转，需要接入鉴权与限流。
