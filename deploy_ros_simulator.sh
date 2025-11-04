#!/bin/bash
# ROS模拟器部署和测试脚本

echo "🚀 ROS任务模拟器部署脚本"
echo "========================="

# 配置参数
ROS_HOST="192.168.203.30"
ROS_USER="root"
ROS_PASSWORD="gZg!p95L"
SCRIPT_NAME="ros_mission_simulator.py"
EXAMPLE_JSON="public/example-planned-path.json"

echo "📋 部署步骤："
echo "1. 将模拟器脚本和示例轨迹复制到ROS机器"
echo "2. 确保ROS环境已启动"
echo "3. 运行模拟器节点"
echo ""

# 检查本地脚本文件
if [ ! -f "$SCRIPT_NAME" ]; then
    echo "❌ 错误: 找不到 $SCRIPT_NAME 文件"
    exit 1
fi

if [ ! -f "$EXAMPLE_JSON" ]; then
    echo "❌ 错误: 找不到 $EXAMPLE_JSON 文件"
    exit 1
fi

echo "📤 复制模拟器脚本与示例轨迹到ROS机器..."
echo "目标: $ROS_USER@$ROS_HOST"

# 创建远端目录并复制文件
ssh "$ROS_USER@$ROS_HOST" "mkdir -p /tmp/public"
scp "$SCRIPT_NAME" "$ROS_USER@$ROS_HOST:/tmp/"
scp "$EXAMPLE_JSON" "$ROS_USER@$ROS_HOST:/tmp/public/"

if [ $? -eq 0 ]; then
    echo "✅ 文件复制成功"
else
    echo "❌ 文件复制失败"
    exit 1
fi

echo ""
echo "🔧 接下来的手动步骤："
echo "1. SSH连接到ROS机器:"
echo "   ssh $ROS_USER@$ROS_HOST"
echo ""
echo "2. 加载ROS环境:"
echo "   source /home/root/catkin_ws/devel/setup.bash"
echo ""
echo "3. 确保以下服务已启动:"
echo "   终端1: roslaunch ego_planner run_in_sim.launch"
echo "   终端2: roslaunch rosbridge_server rosbridge_websocket.launch"
echo ""
echo "4. 运行模拟器:"
echo "   cd /tmp"
echo "   python3 ros_mission_simulator.py"
echo ""
echo "5. 在PCD Viewer中连接ROS:"
echo "   WebSocket URL: ws://192.168.203.30:9999"
echo ""

# 生成快速连接脚本
cat > connect_ros.sh << 'EOF'
#!/bin/bash
echo "🔗 快速连接ROS机器并运行模拟器"
ssh root@192.168.203.30 << 'ENDSSH'
source /home/root/catkin_ws/devel/setup.bash
mkdir -p /tmp/public
cd /tmp
python3 ros_mission_simulator.py
ENDSSH
EOF

chmod +x connect_ros.sh

echo "💡 提示: 运行 './connect_ros.sh' 可以快速连接并启动模拟器"
echo "📊 模拟器将发布以下话题:"
echo "   - /odom_visualization/pose (无人机位置)"
echo "   - /mission/waypoint_reached (航点到达)"
echo "   - /mission/complete (任务完成)"
echo ""
echo "🎯 测试航点路径:"
echo "   来自 public/example-planned-path.json（本地 XYZ 螺旋上升示例）"
