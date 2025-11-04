#!/usr/bin/env python3
"""
示例ROS节点，用于测试PCD Viewer的任务执行流程
发布无人机位置和任务状态消息
"""

import rospy
import json
import time
import os
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math

class MissionSimulator:
    def __init__(self):
        rospy.init_node('mission_simulator', anonymous=True)
        
        # 发布者
        self.pose_pub = rospy.Publisher('/odom_visualization/pose', PoseStamped, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/mission/waypoint_reached', String, queue_size=10)
        self.mission_complete_pub = rospy.Publisher('/mission/complete', String, queue_size=10)
        
        # 任务参数
        self.mission_id = "test_mission_001"
        # 默认矩形巡航路线（若无法加载示例 JSON 则使用）
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "z": 1.0},
            {"x": 5.0, "y": 0.0, "z": 1.0},
            {"x": 5.0, "y": 5.0, "z": 1.0},
            {"x": 0.0, "y": 5.0, "z": 1.0},
            {"x": 0.0, "y": 0.0, "z": 1.0}
        ]
        self.t_values = None  # 来自示例 JSON 的时间戳序列（可选）
        # 尝试加载仓库示例规划路径：public/example-planned-path.json
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            json_path = os.path.join(script_dir, "public", "example-planned-path.json")
            if os.path.exists(json_path):
                with open(json_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                pts = data.get("points", [])
                if isinstance(pts, list) and len(pts) > 1:
                    self.waypoints = [
                        {"x": float(p.get("x", 0.0)), "y": float(p.get("y", 0.0)), "z": float(p.get("z", 0.0))}
                        for p in pts
                    ]
                    # 可选时间序列，用于控制段间移动时长
                    t_seq = []
                    ok = True
                    for p in pts:
                        t = p.get("t", None)
                        if t is None:
                            ok = False
                            break
                        try:
                            t_seq.append(float(t))
                        except Exception:
                            ok = False
                            break
                    if ok and len(t_seq) == len(self.waypoints):
                        self.t_values = t_seq
                        rospy.loginfo("Loaded planned path with timing from example-planned-path.json")
                    else:
                        rospy.loginfo("Loaded planned path (no timing) from example-planned-path.json")
            else:
                rospy.logwarn("example-planned-path.json not found, using default rectangular route")
        except Exception as e:
            rospy.logwarn(f"Failed to load example-planned-path.json: {e}; using default route")
        
        self.current_waypoint = 0
        self.start_time = time.time()
        
        rospy.loginfo("Mission Simulator started")
        rospy.loginfo(f"Mission ID: {self.mission_id}")
        rospy.loginfo(f"Waypoints: {len(self.waypoints)}")
    
    def simulate_flight(self):
        """模拟无人机飞行过程"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and self.current_waypoint < len(self.waypoints):
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            # 计算当前位置（简单线性插值）
            if self.current_waypoint == 0:
                # 起始阶段，停留在第一个点
                pos = self.waypoints[0]
                # 若示例路径带有时间戳，且首段 t[0] 为 0，则等待 1 秒；否则仍等待 2 秒
                wait_time = 2.0
                if self.t_values is not None and len(self.t_values) > 0:
                    wait_time = 1.0
                if elapsed > wait_time:  # 等待后开始移动到下一个点
                    self.current_waypoint = 1
                    self.start_time = current_time
            else:
                # 在航点间移动
                start_wp = self.waypoints[self.current_waypoint - 1]
                target_wp = self.waypoints[self.current_waypoint]
                
                # 计算移动时间
                # 优先使用示例 JSON 的 t 序列（段间时长 = t[i]-t[i-1]，不少于 0.2s）
                if self.t_values is not None:
                    dt = max(0.2, float(self.t_values[self.current_waypoint] - self.t_values[self.current_waypoint - 1]))
                    move_time = dt
                else:
                    # 回退：按距离 1m/s，最少 1.5 秒
                    distance = math.sqrt(
                        (target_wp["x"] - start_wp["x"])**2 + 
                        (target_wp["y"] - start_wp["y"])**2 + 
                        (target_wp["z"] - start_wp["z"])**2
                    )
                    move_time = max(distance, 1.5)
                
                if elapsed <= move_time:
                    # 线性插值计算当前位置
                    t = elapsed / move_time
                    pos = {
                        "x": start_wp["x"] + (target_wp["x"] - start_wp["x"]) * t,
                        "y": start_wp["y"] + (target_wp["y"] - start_wp["y"]) * t,
                        "z": start_wp["z"] + (target_wp["z"] - start_wp["z"]) * t
                    }
                else:
                    # 到达航点
                    pos = target_wp
                    self.reach_waypoint()
            
            # 发布当前位置
            self.publish_pose(pos["x"], pos["y"], pos["z"])
            rate.sleep()
        
        # 任务完成
        self.complete_mission()
    
    def publish_pose(self, x, y, z):
        """发布无人机位置"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # 简单的朝向计算（朝向下一个航点）
        if self.current_waypoint < len(self.waypoints):
            target = self.waypoints[self.current_waypoint]
            dx = target["x"] - x
            dy = target["y"] - y
            yaw = math.atan2(dy, dx)
            
            # 转换为四元数
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
            pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)
    
    def reach_waypoint(self):
        """航点到达处理"""
        waypoint_msg = {
            "waypoint_index": self.current_waypoint - 1,
            "timestamp": int(time.time()),
            "position": self.waypoints[self.current_waypoint - 1]
        }
        
        msg = String()
        msg.data = json.dumps(waypoint_msg)
        self.waypoint_pub.publish(msg)
        
        rospy.loginfo(f"Waypoint {self.current_waypoint - 1} reached")
        
        # 移动到下一个航点
        self.current_waypoint += 1
        self.start_time = time.time()
        
        # 在航点停留1秒
        time.sleep(1.0)
    
    def complete_mission(self):
        """任务完成处理"""
        complete_msg = {
            "mission_id": self.mission_id,
            "status": "completed",
            "timestamp": int(time.time()),
            "reason": "All waypoints reached successfully"
        }
        
        msg = String()
        msg.data = json.dumps(complete_msg)
        self.mission_complete_pub.publish(msg)
        
        rospy.loginfo("Mission completed successfully!")
        rospy.loginfo(f"Total waypoints: {len(self.waypoints)}")
        rospy.loginfo(f"Total time: {time.time() - self.start_time:.1f} seconds")

if __name__ == '__main__':
    try:
        simulator = MissionSimulator()
        simulator.simulate_flight()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mission simulation interrupted")
