# PCD Viewer 部署与分发指南

本文件说明：如何在本机构建 Next.js + SQLite 项目，并将构建后的产物打包给其他人在另一台 Linux 机器上运行。

## 一、前置环境要求

**构建者与接收者两边都建议满足：**

- 操作系统：Linux x86_64
- Node.js：推荐 20.x（使用 nvm 管理更方便）
- 包管理器：`pnpm`（本项目基于 pnpm）
- 原生编译环境（为 `better-sqlite3` 编译）：
  - `python3`
  - `build-essential`（或等价的 C/C++ 编译工具链，如 `make`, `g++`）

Ubuntu/Debian 安装示例：

```bash
sudo apt update
sudo apt install -y build-essential python3
```

## 二、在本机构建生产版本

在项目根目录（本仓库顶层）执行：

```bash
# 安装依赖（如已安装过可跳过）
pnpm install

# 构建生产版本
pnpm run build
```

构建成功时，你会看到类似输出：

- `✓ Collecting page data`
- `✓ Generating static pages (.../...)`
- `✓ Finalizing page optimization`
- 路由列表（`Route (app) ...`），无 Error。

同时日志里会有：

```text
Database initialized at: /path/to/pcd-viewer/data/missions.db
```

说明 SQLite 数据库文件 `data/missions.db` 已经就绪。

## 三、打包需要分发的文件

在项目根目录执行以下命令，打一个压缩包（示例名：`pcd-viewer-dist.tar.gz`）：

```bash
cd /home/hotuns/pcd-viewer   # 按实际路径修改

# 打包核心运行所需内容
 tar czf pcd-viewer-dist.tar.gz \
  .next \
  public \
  data \
  package.json \
  pnpm-lock.yaml \
  pnpm-workspace.yaml \
  next.config.ts \
  tsconfig.json
```

说明：

- `.next/`：Next.js 构建产物（Server + 静态资源）。
- `public/`：静态文件（如 glTF 模型等）。
- `data/`：SQLite 数据库目录，包含 `missions.db`。
- `package.json` / `pnpm-lock.yaml` / `pnpm-workspace.yaml`：依赖与脚本定义，保证对方安装依赖一致。
- `next.config.ts` / `tsconfig.json`：Next.js 与 TypeScript 配置。

**不建议打包：**

- `node_modules/`（让接收方根据 lockfile 自行安装即可）。
- `.git/` 等版本控制信息（如果只是运行，不需要源码管理信息）。

将生成的 `pcd-viewer-dist.tar.gz` 通过 scp、U 盘、网盘等方式发给对方即可。

## 四、在接收方机器上部署与运行

以下假设接收方也是 Linux，已安装 Node 20 + pnpm。

### 1. 解压压缩包

```bash
# 将压缩包放到目标目录，例如 /opt/apps
cd /opt/apps

tar xzf /path/to/pcd-viewer-dist.tar.gz

cd pcd-viewer   # 解压出来的目录名称，如有变化按实际调整
```

### 2. 安装依赖

接收方在解压后的根目录运行：

```bash
# 安装运行所需依赖（会自动根据 pnpm-lock.yaml 安装正确版本）
pnpm install
```

如果看到类似提示：

```text
Ignored build scripts: better-sqlite3.
Run "pnpm approve-builds" to pick which dependencies should be allowed to run scripts.
```

则需要允许 `better-sqlite3` 运行构建脚本：

```bash
pnpm approve-builds
# 在交互界面中用方向键选中 better-sqlite3，按空格勾选，再回车确认

# 然后重新构建原生模块
pnpm rebuild better-sqlite3
```

> 注意：如果系统缺少编译工具链（`build-essential`, `python3` 等），`better-sqlite3` 的编译会失败，需要先安装依赖后再重试。

### 3. 如有需要，可以重新构建（可选）

理论上你已经随包带了 `.next/` 构建结果，对方可以直接启动；

但如果你希望在接收方机器上重新构建一次（避免平台差异），可以运行：

```bash
pnpm run build
```

### 4. 启动生产服务器

默认 `next start` 监听 3000 端口。

```bash
# 使用 3000 端口启动
pnpm start
```

启动成功后，浏览器访问：

```text
http://<服务器 IP 或主机名>:3000/
```

## 五、数据库文件说明

- SQLite 数据库文件路径：`data/missions.db`
- 构建/运行时，如果文件不存在，服务会自动在 `data/` 目录下创建数据库并初始化表结构。
- 如果你在本机已生成并有一些测试数据，可以连同 `data/missions.db` 一起打包，对方启动后会直接看到同样的数据。
- 如果希望对方从空数据库开始，可在打包前删除 `data/missions.db`，只打包空的 `data/` 目录。

## 六、常见问题排查

### 1. 启动时报错：`Could not locate the bindings file`（better-sqlite3）

**可能原因：**

- `better-sqlite3` 未成功编译；
- `pnpm` 忽略了构建脚本；
- 本机缺少 `build-essential` 等编译工具。

**解决步骤：**

```bash
# 安装编译工具（Ubuntu/Debian 示例）
sudo apt update
sudo apt install -y build-essential python3

# 允许 better-sqlite3 运行构建脚本
pnpm approve-builds
# 勾选 better-sqlite3

# 重新编译并启动
pnpm rebuild better-sqlite3
pnpm run build
pnpm start
```
