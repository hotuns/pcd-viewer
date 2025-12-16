# 测试真实 ROS 消息连接

## 前提条件
确保你的 ROS 系统正在运行，并且以下话题正在发布数据：
- `/odom_visualization/pose` (geometry_msgs/PoseStamped)
- `/grid_map/occupancy_inflate` (sensor_msgs/PointCloud2) - **注意：这是点云类型，不是 OccupancyGrid**

## 测试步骤

### 1. 检查 ROS 话题是否存在
```bash
# 列出所有话题
rostopic list

# 查看位置话题
rostopic echo /odom_visualization/pose -n 1

# 查看点云地图话题
rostopic echo /grid_map/occupancy_inflate -n 1

# 检查话题类型
rostopic type /grid_map/occupancy_inflate
# 应该输出: sensor_msgs/PointCloud2

# 查看话题频率
rostopic hz /odom_visualization/pose
rostopic hz /grid_map/occupancy_inflate
```

### 2. 检查 rosbridge 是否运行
```bash
# 确保 rosbridge_websocket 正在运行
roslaunch rosbridge_server rosbridge_websocket.launch
```

### 3. 在前端测试连接

1. 启动前端应用
   ```bash
   npm run dev
   ```

2. 打开浏览器访问 http://localhost:3000

3. 点击右上角"连接 ROS"按钮

4. 输入 rosbridge 地址（默认: `ws://192.168.203.30:9999`）

5. 连接成功后，查看左侧面板：
   - **ROS 消息状态** 应该显示位置消息计数增加
   - **点云地图** 应该显示点数和最后更新时间
   - **无人机位置** 应该显示实时的 X、Y、Z 坐标

### 4. 预期效果

- ✅ 位置消息每秒更新（取决于话题发布频率）
- ✅ 点云地图显示点数和坐标系信息
- ✅ 3D 视图中可以看到无人机模型（如果有位置数据）
- ✅ 点云地图可视化（待实现 3D 渲染）

## 故障排查

### 问题: 消息计数一直是 0
- 检查 ROS 话题是否正在发布：`rostopic hz /odom_visualization/pose`
- 检查 rosbridge 是否运行：`rosnode list | grep rosbridge`
- 检查防火墙设置，确保端口 9999 开放
- 检查浏览器控制台是否有错误信息

### 问题: ROS 报错 "Tried to register topic with type X but it is already established with type Y"
- 这是因为话题类型不匹配
- 使用 `rostopic type /话题名` 检查实际类型
- `/grid_map/occupancy_inflate` 实际是 `sensor_msgs/PointCloud2` 而不是 `nav_msgs/OccupancyGrid`
- 前端代码已更新为正确的类型

### 问题: 连接失败
- 确认 rosbridge IP 地址正确
- 如果是 HTTPS 页面，需要使用 `wss://` 而不是 `ws://`
- 检查网络连接

### 问题: 看不到点云地图
- 确认点云地图话题正在发布
- 确认"显示点云地图"开关已打开
- 检查浏览器控制台日志，应该看到 "Received point cloud map: XXX points"
- 点云地图的 3D 渲染功能还需要进一步实现

### 问题: rosdep 警告
- `the rosdep view is empty` 这是警告，不影响功能
- 如果需要解决：`sudo rosdep init && rosdep update`
