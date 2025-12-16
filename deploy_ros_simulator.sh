#!/bin/bash
# 一键部署并运行 ROS 任务模拟器

set -euo pipefail

########################################
# 可配置参数（也可通过环境变量覆盖）
########################################
ROS_HOST="${ROS_HOST:-192.168.203.30}"
ROS_USER="${ROS_USER:-root}"
ROS_SETUP="${ROS_SETUP:-/home/ren/catkin_ws/devel/setup.bash}"
REMOTE_TMP="${REMOTE_TMP:-/tmp}"
SCRIPT_PATH="${SCRIPT_PATH:-ros_mission_simulator.py}"
TRAJECTORY_PATH="${TRAJECTORY_PATH:-public/example-planned-path.json}"
AUTO_RUN="${AUTO_RUN:-true}"

SCRIPT_BASENAME="$(basename "$SCRIPT_PATH")"
TRAJECTORY_BASENAME="$(basename "$TRAJECTORY_PATH")"
REMOTE_SCRIPT="$REMOTE_TMP/$SCRIPT_BASENAME"
REMOTE_TRAJECTORY_DIR="$REMOTE_TMP/public"
REMOTE_TRAJECTORY="$REMOTE_TRAJECTORY_DIR/$TRAJECTORY_BASENAME"

########################################
# 通用函数
########################################
log()   { printf "\033[1;34m[INFO]\033[0m %s\n" "$*"; }
warn()  { printf "\033[1;33m[WARN]\033[0m %s\n" "$*"; }
err()   { printf "\033[1;31m[ERR ]\033[0m %s\n" "$*"; exit 1; }

usage() {
    cat <<EOF
用法: $(basename "$0") [选项]

选项:
    --host <ip>            指定 ROS 机器 IP（默认: $ROS_HOST）
    --user <name>          指定 SSH 用户（默认: $ROS_USER）
    --script <path>        本地模拟器脚本路径（默认: $SCRIPT_PATH）
    --trajectory <path>    本地轨迹 JSON 路径（默认: $TRAJECTORY_PATH）
    --ros-setup <path>     远端 ROS 环境脚本（默认: $ROS_SETUP）
    --remote-dir <path>    远端临时目录（默认: $REMOTE_TMP）
    --copy-only            只复制文件，不自动运行模拟器
    --run                  强制在复制后自动运行模拟器（默认行为）
    -h, --help             查看此帮助

也可以通过设置环境变量 ROS_HOST 等方式覆盖默认值。
EOF
}

########################################
# 解析参数
########################################
while [[ $# -gt 0 ]]; do
    case "$1" in
        --host) shift; ROS_HOST="$1" ;;
        --user) shift; ROS_USER="$1" ;;
        --script) shift; SCRIPT_PATH="$1"; SCRIPT_BASENAME="$(basename "$SCRIPT_PATH")"; REMOTE_SCRIPT="$REMOTE_TMP/$SCRIPT_BASENAME" ;;
        --trajectory) shift; TRAJECTORY_PATH="$1"; TRAJECTORY_BASENAME="$(basename "$TRAJECTORY_PATH")"; REMOTE_TRAJECTORY="$REMOTE_TRAJECTORY_DIR/$TRAJECTORY_BASENAME" ;;
        --ros-setup) shift; ROS_SETUP="$1" ;;
        --remote-dir) shift; REMOTE_TMP="$1"; REMOTE_SCRIPT="$REMOTE_TMP/$SCRIPT_BASENAME"; REMOTE_TRAJECTORY_DIR="$REMOTE_TMP/public"; REMOTE_TRAJECTORY="$REMOTE_TRAJECTORY_DIR/$TRAJECTORY_BASENAME" ;;
        --copy-only) AUTO_RUN="false" ;;
        --run) AUTO_RUN="true" ;;
        -h|--help) usage; exit 0 ;;
        *) err "未知参数: $1" ;;
    esac
    shift
done

########################################
# 前置检查
########################################
command -v ssh >/dev/null 2>&1 || err "未检测到 ssh 命令"
command -v scp >/dev/null 2>&1 || err "未检测到 scp 命令"

[[ -f "$SCRIPT_PATH" ]] || err "找不到模拟器脚本 $SCRIPT_PATH"
[[ -f "$TRAJECTORY_PATH" ]] || err "找不到轨迹文件 $TRAJECTORY_PATH"

log "目标机器: $ROS_USER@$ROS_HOST (远端临时目录: $REMOTE_TMP)"

########################################
# 步骤 1: 复制文件
########################################
log "创建远端目录并复制文件..."
ssh "$ROS_USER@$ROS_HOST" "mkdir -p '$REMOTE_TRAJECTORY_DIR'"
scp "$SCRIPT_PATH" "$ROS_USER@$ROS_HOST:$REMOTE_SCRIPT" >/dev/null
scp "$TRAJECTORY_PATH" "$ROS_USER@$ROS_HOST:$REMOTE_TRAJECTORY" >/dev/null
log "文件复制完成"

########################################
# 步骤 2: （可选）远端启动模拟器
########################################
if [[ "$AUTO_RUN" == "true" ]]; then
    log "开始在远端启动模拟器..."
    ssh "$ROS_USER@$ROS_HOST" bash -s <<EOF
set -euo pipefail
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo "⚠️  提示: 找不到 $ROS_SETUP，跳过 source"
fi
cd "$REMOTE_TMP"
    echo "🔁 使用轨迹目录: $REMOTE_TRAJECTORY_DIR"
    python3 "$REMOTE_SCRIPT"
EOF
    log "模拟器执行完毕（若为长时间运行程序，可在远端终端查看输出）"
else
    warn "已根据参数跳过自动运行。可执行以下命令手动启动:"
    cat <<EOF
ssh $ROS_USER@$ROS_HOST <<'REMOTE'
source $ROS_SETUP
cd $REMOTE_TMP
    python3 $REMOTE_SCRIPT
REMOTE
EOF
fi

########################################
# 步骤 3: 生成快捷脚本
########################################
cat > connect_ros.sh <<EOF
#!/bin/bash
ssh $ROS_USER@$ROS_HOST <<'REMOTE'
set -euo pipefail
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
fi
cd "$REMOTE_TMP"
python3 "$REMOTE_SCRIPT"
REMOTE
EOF
chmod +x connect_ros.sh
log "已更新 connect_ros.sh，可随时运行 ./connect_ros.sh 重新启动模拟器"

cat <<'EOF'
📊 模拟器默认发布以下话题:
    - /odom_visualization/pose      (无人机位置)
    - /mission/waypoint_reached    (航点到达)
    - /mission/complete            (任务完成)

✅ 请确认远端机器已手动或通过服务方式启动以下 ROS 组件:
    1) roslaunch ego_planner run_in_sim.launch
    2) roslaunch rosbridge_server rosbridge_websocket.launch

然后在 PCD Viewer 中使用 ws://<ROS_HOST>:9999 进行连接。
EOF
