#!/usr/bin/env python3
"""
Missionlogic demo simulator.
在远程 ROS 服务器上运行本脚本，可模拟 mission_msgs 中的主要话题，
便于前端 Missionlogic 面板完整走一遍流程。
"""

import math
import random
import threading
import time
from typing import Optional, List

import rospy
import struct
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from mission_msgs.msg import (
    MissionList,
    MissionStatus,
    WaypointPosition,
    HangarChargeStatus,
    Control,
    TaskOpt,
)

EMERGENCY_WAYPOINT_INFO = "__emergency_waypoint__"


CMD_TAKEOFF = 1
CMD_LAND = 2
CMD_EXECUTE = 3
CMD_RETURN_HOME = 4
CMD_ARM_OFF = 5
CMD_EMERGENCY_LAND = 6

TASK_OPT_START = 1
TASK_OPT_PAUSE = 2
TASK_OPT_STOP = 3
TASK_OPT_EMERGENCY = 4
TASK_OPT_EMERGENCY = 4

STATUS_HANGAR_READY = 0
STATUS_MISSION_UPLOAD = 1
STATUS_TAKEOFF_READY = 2
STATUS_TAKEOFF = 3
STATUS_EXECUTING = 4
STATUS_RETURN = 5
STATUS_LANDING = 6
STATUS_CHARGING = 7


class MissionLogicSimulator:
    def __init__(self) -> None:
        self.status_pub = rospy.Publisher("/mission/status", MissionStatus, queue_size=10, latch=True)
        self.waypoint_pub = rospy.Publisher("/mission/waypoint_feedback", WaypointPosition, queue_size=50)
        self.charge_pub = rospy.Publisher("/hangar/charge_status", HangarChargeStatus, queue_size=10, latch=True)
        self.battery_pub = rospy.Publisher("/battery/status", BatteryState, queue_size=10, latch=True)
        self.pose_pub = rospy.Publisher("/odom_visualization/pose", Odometry, queue_size=20)
        self.cloud_pub = rospy.Publisher("/grid_map/occupancy_inflate", PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher("/mission/trajectory_marker", Marker, queue_size=10, latch=True)
        self.marker_array_pub = rospy.Publisher("/mission/trajectory_markers", MarkerArray, queue_size=10, latch=True)

        rospy.Subscriber("/mission/list", MissionList, self.on_mission_list)
        rospy.Subscriber("/mission/control", Control, self.on_control)
        rospy.Subscriber("/mission/task_opt", TaskOpt, self.on_task_opt)

        self._lock = threading.Lock()
        self._current_mission: Optional[MissionList] = None
        self._mission_thread: Optional[threading.Thread] = None
        self._running = False
        self._stop_requested = False
        self._paused = False
        self._resume_index = 0
        self._battery_percentage = 100.0
        self._home_pose = None
        self._mission_id = 0
        self._cruise_speed = rospy.get_param("~cruise_speed_mps", 4.0)
        self._hover_time = rospy.get_param("~hover_time", 1.0)
        self._consumption_per_meter = rospy.get_param("~consumption_per_meter", 0.6)
        self._return_threshold = rospy.get_param("~return_threshold", 35.0)
        self._last_completed_index = -1
        self._trajectory_points: List[Point] = []

        self.publish_charge_status(charge_status=2, percentage=100.0)
        self.publish_battery()
        rospy.loginfo("MissionLogic simulator ready. 等待 MissionList 和 Control 指令…")

    # ---------------------- ROS 消息处理 ----------------------
    def on_mission_list(self, msg: MissionList) -> None:
        with self._lock:
            filtered = []
            for wp in msg.PosList:
                if wp.info == EMERGENCY_WAYPOINT_INFO:
                    rospy.loginfo("忽略迫降航点，不纳入航线进度")
                    continue
                filtered.append(wp)
            msg.PosList = filtered
            msg.PosNum = len(filtered)
            self._current_mission = msg
            self._home_pose = msg.HomePos
            self._mission_id = msg.id
            self._last_completed_index = -1
            self._trajectory_points = []
        rospy.loginfo("收到 MissionList: %d 个航点", msg.PosNum)
        self.publish_status(STATUS_MISSION_UPLOAD)

    def on_control(self, msg: Control) -> None:
        cmd = msg.cmd
        rospy.loginfo("收到 Control 指令: %d", cmd)
        if cmd == CMD_TAKEOFF:
            self.publish_status(STATUS_TAKEOFF)
        elif cmd == CMD_EXECUTE:
            self.start_mission()
        elif cmd == CMD_RETURN_HOME:
            self.request_return_home()
        elif cmd == CMD_LAND:
            self.finish_mission(land_only=True)
        elif cmd == CMD_ARM_OFF:
            self.publish_charge_cycle()
        else:
            rospy.logwarn("未知的 Control 指令: %s", cmd)

    def on_task_opt(self, msg: TaskOpt) -> None:
        rospy.loginfo("收到 TaskOpt: opt=%d id=%d", msg.opt, msg.id)
        if msg.opt == TASK_OPT_START:
            self.start_mission(resume=True)
        elif msg.opt == TASK_OPT_PAUSE:
            self.pause_mission()
        elif msg.opt == TASK_OPT_STOP:
            rospy.loginfo("收到停止任务指令，触发返航流程")
            with self._lock:
                self._stop_requested = True
                self._paused = False
            self.request_return_home()
        elif msg.opt == TASK_OPT_EMERGENCY:
            rospy.loginfo("收到紧急迫降指令，立即降落")
            self.finish_mission(land_only=True)
        else:
            rospy.logwarn("未知的 TaskOpt opt=%d", msg.opt)

    # ---------------------- 状态发布 ----------------------
    def publish_status(self, status_code: int, completed: Optional[int] = None) -> None:
        msg = MissionStatus()
        if completed is not None:
            self._last_completed_index = completed
        msg.id = self._mission_id
        msg.status = status_code
        if self._current_mission:
            msg.HomePos = self._current_mission.HomePos
            msg.PosNum = self._current_mission.PosNum
            msg.PosList = []
            for idx, wp in enumerate(self._current_mission.PosList):
                item = WaypointPosition()
                item.x = wp.x
                item.y = wp.y
                item.z = wp.z
                item.pass_type = idx <= self._last_completed_index
                item.task_type = str(wp.task_type) if wp.task_type is not None else "0"
                item.info = wp.info or f"wp-{idx}"
                msg.PosList.append(item)
        else:
            msg.PosNum = 0
            msg.PosList = []
        self.status_pub.publish(msg)

    def publish_charge_status(self, charge_status: int, percentage: float, power: float = 0.0) -> None:
        msg = HangarChargeStatus()
        msg.header = Header(stamp=rospy.Time.now())
        msg.charge_status = charge_status
        msg.battery_percentage = int(percentage)
        msg.charge_power = power
        msg.charge_duration = 0.0
        msg.error_message = ""
        self.charge_pub.publish(msg)

    def publish_battery(self) -> None:
        msg = BatteryState()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        msg.percentage = self._battery_percentage / 100.0
        msg.voltage = 15.8 * msg.percentage
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(msg)

    def publish_pose(self, x: float, y: float, z: float) -> None:
        odom = Odometry()
        odom.header = Header(stamp=rospy.Time.now(), frame_id="map")
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = 1.0
        self.pose_pub.publish(odom)
        self._trajectory_points.append(Point(x=x, y=y, z=z))
        self.publish_traj_markers()
        self.publish_point_cloud(x, y, z)

    def publish_point_cloud(self, x: float, y: float, z: float) -> None:
        if self.cloud_pub.get_num_connections() == 0:
            return
        points = []
        for _ in range(2000):
            offset_r = random.uniform(0.3, 4.0)
            theta = random.uniform(0, math.pi * 2)
            phi = random.uniform(-math.pi / 6, math.pi / 6)
            px = x + offset_r * math.cos(theta)
            py = y + offset_r * math.sin(theta)
            pz = z + offset_r * math.sin(phi)
            points.extend([px, py, pz])

        if not points:
            return

        binary = struct.pack('<' + 'f' * len(points), *points)
        msg = PointCloud2()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg.height = 1
        msg.width = len(points) // 3
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = binary
        self.cloud_pub.publish(msg)

    # ---------------------- 任务执行逻辑 ----------------------
    def start_mission(self, resume: bool = False) -> None:
        with self._lock:
            if self._running:
                rospy.logwarn("任务已在执行中，忽略重复指令")
                return
            if not self._current_mission or not self._current_mission.PosList:
                rospy.logwarn("尚未收到 MissionList，无法开始执行")
                return
            if resume and self._resume_index > 0:
                rospy.loginfo("继续执行任务（从航点 #%d）", self._resume_index)
            else:
                self._resume_index = 0
                self._trajectory_points = []
            self._running = True
            self._stop_requested = False
            self._paused = False
            self._mission_thread = threading.Thread(target=self._mission_worker, daemon=True)
            self._mission_thread.start()
            rospy.loginfo("开始执行任务，共 %d 个航点", len(self._current_mission.PosList))

    def request_return_home(self) -> None:
        with self._lock:
            if not self._running:
                rospy.logwarn("当前没有执行中的任务，忽略返航指令")
                return
            rospy.loginfo("收到返航指令，任务将提前结束")
            self._stop_requested = True

    def finish_mission(self, land_only: bool = False) -> None:
        with self._lock:
            if not self._running and not land_only:
                return
            self._running = False
            self._paused = False
        if land_only:
            self.publish_status(STATUS_LANDING)
            time.sleep(1.0)
        self.publish_charge_cycle()

    def publish_charge_cycle(self) -> None:
        self.publish_status(STATUS_CHARGING)
        # 模拟逐步充电
        target = 100.0
        while self._battery_percentage < target and not rospy.is_shutdown():
            self._battery_percentage = min(target, self._battery_percentage + 5.0)
            self.publish_charge_status(charge_status=1, percentage=self._battery_percentage, power=250.0)
            self.publish_battery()
            time.sleep(0.6)
        self.publish_charge_status(charge_status=2, percentage=100.0, power=0.0)
        self.publish_status(STATUS_HANGAR_READY)
        rospy.loginfo("充电完成，机库 ready")

    def _mission_worker(self) -> None:
        self.publish_status(STATUS_TAKEOFF_READY)
        time.sleep(1.0)
        self.publish_status(STATUS_TAKEOFF)
        time.sleep(1.0)
        self.publish_status(STATUS_EXECUTING)

        assert self._current_mission is not None
        def distance(p1, p2):
            return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

        current_position = self._home_pose.pose.position if self._home_pose else self._current_mission.PosList[0]

        start_index = self._resume_index if self._resume_index < len(self._current_mission.PosList) else 0

        for idx, wp in enumerate(self._current_mission.PosList):
            if idx < start_index:
                continue
            if rospy.is_shutdown():
                break
            if self._stop_requested:
                rospy.loginfo("任务被要求返航，跳出航点循环，下一次从航点 #%d 继续", idx)
                self._resume_index = idx
                break
            if self._paused:
                rospy.loginfo("任务已暂停，记录当前航点 #%d", idx)
                self._resume_index = idx
                return
            segment_distance = distance(current_position, wp)
            travel_time = max(1.5, segment_distance / max(0.5, self._cruise_speed))
            steps = max(5, int(travel_time / 0.3))
            for step in range(steps):
                if rospy.is_shutdown() or self._stop_requested:
                    break
                ratio = (step + 1) / steps
                interp_x = current_position.x + (wp.x - current_position.x) * ratio
                interp_y = current_position.y + (wp.y - current_position.y) * ratio
                interp_z = current_position.z + (wp.z - current_position.z) * ratio
                noise = lambda: random.uniform(-0.05, 0.05)
                self.publish_pose(interp_x + noise(), interp_y + noise(), interp_z + noise())
                self._battery_percentage = max(5.0, self._battery_percentage - segment_distance / steps * self._consumption_per_meter)
                self.publish_battery()
                time.sleep(travel_time / steps)
            if self._stop_requested:
                rospy.loginfo("任务返航中断于航点 #%d，未标记完成", idx)
                self._resume_index = idx
                break
            current_position = wp
            time.sleep(self._hover_time)
            feedback = WaypointPosition()
            feedback.x = wp.x
            feedback.y = wp.y
            feedback.z = wp.z
            feedback.pass_type = True
            feedback.task_type = str(wp.task_type) if wp.task_type is not None else "0"
            feedback.info = wp.info or f"wp-{idx}"
            self.waypoint_pub.publish(feedback)
            self.publish_status(STATUS_EXECUTING, completed=idx)
            time.sleep(1.0)

        if not self._stop_requested:
            self._resume_index = 0
        self.publish_status(STATUS_RETURN)
        if self._home_pose:
            home_pos = self._home_pose.pose.position
            self.publish_pose(home_pos.x, home_pos.y, max(1.0, home_pos.z))
            time.sleep(2.0)
        self.publish_status(STATUS_LANDING)
        time.sleep(1.0)
        self.finish_mission()

    def publish_traj_markers(self) -> None:
        if not self._trajectory_points:
            return

        header = Header(stamp=rospy.Time.now(), frame_id="map")

        line_marker = Marker()
        line_marker.header = header
        line_marker.ns = "mission_traj"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.2
        line_marker.color.g = 0.8
        line_marker.color.b = 0.6
        line_marker.color.a = 0.85
        line_marker.points = self._trajectory_points
        self.marker_pub.publish(line_marker)

        marker_array = MarkerArray()
        start_marker = Marker()
        start_marker.header = header
        start_marker.ns = "mission_traj_points"
        start_marker.id = 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.3
        start_marker.color.r = 0.2
        start_marker.color.g = 0.8
        start_marker.color.b = 0.2
        start_marker.color.a = 0.9
        start_marker.pose.position = self._trajectory_points[0]
        marker_array.markers.append(start_marker)

        current_marker = Marker()
        current_marker.header = header
        current_marker.ns = "mission_traj_points"
        current_marker.id = 2
        current_marker.type = Marker.SPHERE
        current_marker.action = Marker.ADD
        current_marker.scale.x = 0.25
        current_marker.scale.y = 0.25
        current_marker.scale.z = 0.25
        current_marker.color.r = 1.0
        current_marker.color.g = 0.4
        current_marker.color.b = 0.1
        current_marker.color.a = 0.95
        current_marker.pose.position = self._trajectory_points[-1]
        marker_array.markers.append(current_marker)

        self.marker_array_pub.publish(marker_array)


def main() -> None:
    rospy.init_node("mission_logic_sim")
    MissionLogicSimulator()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
