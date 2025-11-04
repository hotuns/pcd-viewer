"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { PCDCanvas, PCDCanvasHandle } from "./PCDCanvas";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Separator } from "@/components/ui/separator";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Maximize, PlayCircle, StopCircle, Plug, X } from "lucide-react";
import { useRosConnection } from "@/hooks/useRosConnection";
import { createGridMapDataFromPointCloud2 } from "@/lib/pointCloudParser";
import { Mission, DronePosition, Waypoint, WaypointReachedMessage, MissionCompleteMessage, PointCloudFrame, GridMapData } from "@/types/mission";
// @ts-expect-error - roslib 没有官方类型定义
import ROSLIB from "roslib";

interface MissionControllerProps {
  initialMission?: Mission;
}

export default function MissionController({ initialMission }: MissionControllerProps) {
  const canvasRef = useRef<PCDCanvasHandle | null>(null);
  const [selectedMission, setSelectedMission] = useState<Mission | null>(null);
  const [pointSize, setPointSize] = useState(0.01);
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(true);
  const [loading, setLoading] = useState(false);
  const [colorMode, setColorMode] = useState<"none" | "rgb" | "intensity" | "height">("none");
  const [pointSizeMax, setPointSizeMax] = useState(0.1);
  const [showSceneCloud, setShowSceneCloud] = useState(true); // 场景点云显示开关
  const [followDrone, setFollowDrone] = useState(false); // 视角跟随飞机
  
  // ROS 连接（使用 hook）
  const { rosUrl, setRosUrl, rosConnected, rosRef, connectROS, disconnectROS } = useRosConnection();
  const [missionRunning, setMissionRunning] = useState(false);
  const [dronePosition, setDronePosition] = useState<DronePosition | null>(null);
  const odomTopicRef = useRef<typeof ROSLIB.Topic | null>(null);
  const gridMapTopicRef = useRef<typeof ROSLIB.Topic | null>(null);
  const [rosModalOpen, setRosModalOpen] = useState(false);
  const [subscribed, setSubscribed] = useState(false);
  const [msgCount, setMsgCount] = useState(0);
  // 任务完成提示
  const [completionOpen, setCompletionOpen] = useState(false);
  const [completionData, setCompletionData] = useState<{ status: 'completed' | 'failed'; reason?: string } | null>(null);
  // 消息时间与最近事件
  const [lastPoseAt, setLastPoseAt] = useState<Date | null>(null);
  const [lastWaypointIndex, setLastWaypointIndex] = useState<number | null>(null);
  const [lastWaypointAt, setLastWaypointAt] = useState<Date | null>(null);
  const [lastCompleteAt, setLastCompleteAt] = useState<Date | null>(null);
  
  // 实时点云数据
  const [realtimeFrames, setRealtimeFrames] = useState<PointCloudFrame[]>([]);
  const [gridMapData, setGridMapData] = useState<GridMapData | null>(null);
  const [maxFrames, setMaxFrames] = useState(10);
  const [showRealtimeCloud, setShowRealtimeCloud] = useState(true);
  const [showGridMap, setShowGridMap] = useState(true);
  const [lastCloudAt, setLastCloudAt] = useState<Date | null>(null);
  const [lastGridMapAt, setLastGridMapAt] = useState<Date | null>(null);

  // 点云和包围盒信息
  const [count, setCount] = useState<number>(0);
  const [bbox, setBbox] = useState<{ min: [number, number, number]; max: [number, number, number] } | null>(null);
  // 规划航线（用于渲染）
  const [plannedPoints, setPlannedPoints] = useState<Array<{ x: number; y: number; z: number }> | null>(null);
  const [plannedVisible, setPlannedVisible] = useState(true);
  const [plannedEditable, setPlannedEditable] = useState(false);
  const [plannedPointSize, setPlannedPointSize] = useState(0.02);
  const [plannedPointSizeMax, setPlannedPointSizeMax] = useState(0.1);
  const [plannedDragPlane, setPlannedDragPlane] = useState<'xy'|'xz'|'yz'>('xy');
  const [autoOrientPlane, setAutoOrientPlane] = useState<boolean>(false);

  // 扩展 disconnectROS 以清理所有订阅
  const handleDisconnect = useCallback(() => {
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }
    if (gridMapTopicRef.current) {
      gridMapTopicRef.current.unsubscribe();
      gridMapTopicRef.current = null;
    }
    disconnectROS();
  }, [disconnectROS]);

  // ROS 连接成功后的回调
  useEffect(() => {
    if (rosConnected) {
      setRosModalOpen(false);
    }
  }, [rosConnected]);

  // 处理任务更新
  const handleMissionUpdate = useCallback((updatedMission: Mission) => {
    setSelectedMission(updatedMission);
    
    // 保存到localStorage
    const savedMissions = localStorage.getItem('pcd-viewer-missions');
    if (savedMissions) {
      try {
        const missions = JSON.parse(savedMissions);
        const updatedMissions = missions.map((m: Mission & { createdAt: string; completedAt?: string; startedAt?: string }) => 
          m.id === updatedMission.id ? {
            ...updatedMission,
            createdAt: updatedMission.createdAt.toISOString(),
            startedAt: updatedMission.startedAt?.toISOString(),
            completedAt: updatedMission.completedAt?.toISOString(),
          } : m
        );
        localStorage.setItem('pcd-viewer-missions', JSON.stringify(updatedMissions));
      } catch (error) {
        console.error('Failed to save mission update:', error);
      }
    }
    
    // 如果进入planning状态，开启航线编辑模式
    if (updatedMission.status === 'planning') {
      setPlannedEditable(true);
    } else if (updatedMission.status === 'ready') {
      setPlannedEditable(false);
    }
  }, []);

  // 处理航点到达
  const handleWaypointReached = useCallback((msg: WaypointReachedMessage) => {
    setSelectedMission(currentMission => {
      if (!currentMission || !currentMission.waypoints) return currentMission;

      const waypointIndex = msg.waypoint_index;
      if (waypointIndex < 0 || waypointIndex >= currentMission.waypoints.length) return currentMission;

      // 直接在现有数组上更新，不扩充长度
      const updatedWaypoints = currentMission.waypoints.map((wp, index) => {
        if (index <= waypointIndex) {
          // 将所有 <= 当前索引的航点标记为 completed
          return {
            ...wp,
            status: 'completed' as const,
            completedAt: wp.completedAt ?? new Date((msg.timestamp ?? Math.floor(Date.now()/1000)) * 1000),
          };
        }
        return wp;
      });

      // 设置下一个为 active（如果存在）
      const nextIndex = waypointIndex + 1;
      if (nextIndex < updatedWaypoints.length && updatedWaypoints[nextIndex].status !== 'completed') {
        updatedWaypoints[nextIndex] = { ...updatedWaypoints[nextIndex], status: 'active' };
      }

      const updatedMission: Mission = {
        ...currentMission,
        waypoints: updatedWaypoints,
        currentWaypointIndex: nextIndex < updatedWaypoints.length ? nextIndex : undefined,
        executionLog: [
          ...(currentMission.executionLog || []),
          { timestamp: new Date(), event: 'waypoint_reached', details: { waypointIndex, position: msg.position } }
        ]
      };

      console.log(`Waypoint ${waypointIndex} reached`);
      return updatedMission;
    });
  }, []);  // 处理任务完成
  const handleMissionComplete = useCallback((msg: MissionCompleteMessage) => {
    setSelectedMission(currentMission => {
      if (!currentMission) return currentMission;
      
      const updatedMission = {
        ...currentMission,
        status: msg.status === 'completed' ? 'completed' as const : 'failed' as const,
        completedAt: new Date(msg.timestamp * 1000),
        executionLog: [
          ...(currentMission.executionLog || []),
          {
            timestamp: new Date(),
            event: 'mission_complete',
            details: { status: msg.status, reason: msg.reason }
          }
        ]
      };
      
      console.log(`Mission ${msg.status}: ${msg.reason || 'No reason provided'}`);
      return updatedMission;
    });
    
    setMissionRunning(false);
    // 弹出任务完成/失败提示
    setCompletionData({ status: msg.status === 'completed' ? 'completed' : 'failed', reason: msg.reason });
    setCompletionOpen(true);
  }, []);

  // 处理实时点云数据
  const handleRealtimePointCloud = useCallback((msg: {
    header: { stamp: { sec: number; nsec: number }; frame_id: string };
    points: Array<{ x: number; y: number; z: number; intensity?: number; rgb?: number }>;
  }) => {
    const timestamp = new Date(msg.header.stamp.sec * 1000 + msg.header.stamp.nsec / 1000000);
    setLastCloudAt(timestamp);

    const points = msg.points.map(p => ({
      x: p.x,
      y: p.y,
      z: p.z,
      intensity: p.intensity,
      // RGB数据解析（如果存在）
      ...(p.rgb ? {
        r: (p.rgb >> 16) & 0xFF,
        g: (p.rgb >> 8) & 0xFF,
        b: p.rgb & 0xFF
      } : {})
    }));

    const newFrame: PointCloudFrame = {
      id: `frame_${timestamp.getTime()}`,
      timestamp,
      points,
      frameId: msg.header.frame_id
    };

    setRealtimeFrames(prevFrames => {
      const updated = [newFrame, ...prevFrames];
      // 保持最近的N帧
      return updated.slice(0, maxFrames);
    });
  }, [maxFrames]);

  // 处理网格地图数据
  const handleGridMap = useCallback((msg: {
    info: {
      width: number;
      height: number;
      resolution: number;
      origin: { position: { x: number; y: number; z: number } };
    };
    data: number[];
  }) => {
    const timestamp = new Date();
    setLastGridMapAt(timestamp);

    const gridMap: GridMapData = {
      timestamp,
      width: msg.info.width,
      height: msg.info.height,
      resolution: msg.info.resolution,
      origin: {
        x: msg.info.origin.position.x,
        y: msg.info.origin.position.y,
        z: msg.info.origin.position.z
      },
      data: msg.data
    };

    setGridMapData(gridMap);
  }, []);

  // 初始化航点状态
  const initializeWaypoints = useCallback((mission: Mission, points: Array<{ x: number; y: number; z: number }>) => {
    if (!mission || mission.status !== 'ready') return mission;
    
    const waypoints: Waypoint[] = points.map((point, index) => ({
      ...point,
      id: `${mission.id}_wp_${index}`,
      status: 'pending',
      index
    }));
    if (waypoints.length > 0) {
      waypoints[0] = { ...waypoints[0], status: 'active' } as Waypoint;
    }
    
    const updatedMission = {
      ...mission,
      waypoints,
      currentWaypointIndex: 0,
      status: 'running' as const,
      startedAt: new Date()
    };
    
    setSelectedMission(updatedMission);
    return updatedMission;
  }, []);

  // 开始任务
  const startMission = useCallback(() => {
    if (!rosConnected || !rosRef.current) {
      alert('请先连接 ROS');
      return;
    }

    if (!selectedMission || selectedMission.status !== 'ready') {
      alert('请先选择一个就绪状态的任务');
      return;
    }

    if (!plannedPoints || plannedPoints.length === 0) {
      alert('未加载到规划航线，请先配置/加载航线文件');
      return;
    }

    // 初始化航点状态
    const mission = initializeWaypoints(selectedMission, plannedPoints || []);
    if (!mission) return;

    // 订阅无人机位置话题（PoseStamped）
    const odomTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/odom_visualization/pose',
      messageType: 'geometry_msgs/PoseStamped'
    });

    // 订阅航点到达话题
    const waypointTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/mission/waypoint_reached',
      messageType: 'std_msgs/String'
    });

    // 订阅任务完成话题
    const missionCompleteTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/mission/complete',
      messageType: 'std_msgs/String'
    });

    console.log('Subscribing to mission topics...');
    setMsgCount(0);

    // 位置更新
    odomTopic.subscribe((msg: {
      pose: { position: { x: number; y: number; z: number }; orientation: { x: number; y: number; z: number; w: number } };
    }) => {
      setMsgCount((c) => c + 1);
      setLastPoseAt(new Date());
      const pos = msg.pose.position;
      const orient = msg.pose.orientation;

      setDronePosition({
        x: pos.x,
        y: pos.y,
        z: pos.z,
        orientation: {
          x: orient.x,
          y: orient.y,
          z: orient.z,
          w: orient.w
        }
      });
    });

    // 航点到达处理
    waypointTopic.subscribe((msg: { data: string }) => {
      try {
        const waypointMsg: WaypointReachedMessage = JSON.parse(msg.data);
        setLastWaypointIndex(waypointMsg.waypoint_index);
        setLastWaypointAt(new Date((waypointMsg.timestamp ?? Math.floor(Date.now()/1000)) * 1000));
        handleWaypointReached(waypointMsg);
      } catch (error) {
        console.error('Failed to parse waypoint message:', error);
      }
    });

    // 任务完成处理
    missionCompleteTopic.subscribe((msg: { data: string }) => {
      try {
        const completeMsg: MissionCompleteMessage = JSON.parse(msg.data);
        setLastCompleteAt(new Date((completeMsg.timestamp ?? Math.floor(Date.now()/1000)) * 1000));
        handleMissionComplete(completeMsg);
      } catch (error) {
        console.error('Failed to parse mission complete message:', error);
      }
    });

    odomTopicRef.current = odomTopic;
    setMissionRunning(true);
    setSubscribed(true);
  }, [rosConnected, selectedMission, plannedPoints, initializeWaypoints, handleWaypointReached, handleMissionComplete]);

  // 停止任务
  const stopMission = useCallback(() => {
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }
    if (gridMapTopicRef.current) {
      gridMapTopicRef.current.unsubscribe();
      gridMapTopicRef.current = null;
    }
    setMissionRunning(false);
    setDronePosition(null);
    setSubscribed(false);
  }, []);

  // 处理初始任务
  useEffect(() => {
    if (initialMission) {
      setSelectedMission(initialMission);
    }
  }, [initialMission]);

  // 组件卸载时清理
  useEffect(() => {
    return () => {
      handleDisconnect();
    };
  }, [handleDisconnect]);

  // 监听实时数据显示开关变化
  useEffect(() => {
    if (!rosConnected || !rosRef.current || !missionRunning) return;

    // 取消之前的订阅
    if (gridMapTopicRef.current) {
      gridMapTopicRef.current.unsubscribe();
      gridMapTopicRef.current = null;
    }

    console.log('Subscribing to realtime data topics...');

    // 订阅点云地图 (实际是 PointCloud2 类型)
    if (showGridMap) {
      const gridTopic = new ROSLIB.Topic({
        ros: rosRef.current,
        name: '/grid_map/occupancy_inflate',
        messageType: 'sensor_msgs/PointCloud2'
      });

      gridTopic.subscribe((msg: {
        header: {
          stamp: { sec: number; nsec: number };
          frame_id: string;
        };
        height: number;
        width: number;
        fields: Array<{ name: string; offset: number; datatype: number; count: number }>;
        is_bigendian: boolean;
        point_step: number;
        row_step: number;
        data: string | number[]; // 可能是 base64 字符串或数组
        is_dense: boolean;
      }) => {
        try {
          const timestamp = new Date(msg.header.stamp.sec * 1000 + msg.header.stamp.nsec / 1000000);
          setLastGridMapAt(timestamp);

          const numPoints = msg.width * msg.height;
          
          console.log(`Received point cloud map: ${numPoints} points, frame: ${msg.header.frame_id}`);
          console.log('Fields:', msg.fields);
          console.log('Data length:', msg.data ? msg.data.length : 0);
          console.log('Point step:', msg.point_step);
          console.log('Is bigendian:', msg.is_bigendian);
          
          // 解析PointCloud2二进制数据
          // 找到 x, y, z 字段的偏移量
          const xField = msg.fields.find(f => f.name === 'x');
          const yField = msg.fields.find(f => f.name === 'y');
          const zField = msg.fields.find(f => f.name === 'z');
          
          console.log('X field:', xField);
          console.log('Y field:', yField);
          console.log('Z field:', zField);
          
          if (xField && yField && zField && msg.data && msg.data.length > 0) {
            try {
              // 确保 data 是字符串类型
              const dataString = typeof msg.data === 'string' ? msg.data : '';
              if (dataString) {
                const parsedData = createGridMapDataFromPointCloud2({
                  header: msg.header,
                  data: dataString,
                  point_step: msg.point_step,
                  row_step: msg.row_step,
                  width: msg.width,
                  height: msg.height,
                });
                
                // 扩展为完整的 GridMapData
                const fullGridMap: GridMapData = {
                  frameId: parsedData.frameId,
                  pointCount: parsedData.pointCount,
                  data: Array.from(parsedData.data), // Float32Array -> number[]
                  timestamp: new Date(msg.header.stamp.sec * 1000 + msg.header.stamp.nsec / 1000000),
                  width: msg.width,
                  height: msg.height,
                  resolution: 0.1,
                  origin: { x: 0, y: 0, z: 0 },
                };
                
                setGridMapData(fullGridMap);
              }
            } catch (e) {
              console.error('Failed to parse PointCloud2:', e);
            }
          } else {
            console.log('Missing required fields or data:', {
              hasXField: !!xField,
              hasYField: !!yField,
              hasZField: !!zField,
              hasData: !!(msg.data && msg.data.length > 0)
            });
          }
        } catch (error) {
          console.error('Error processing grid map data:', error);
        }
      });

      gridMapTopicRef.current = gridTopic;
      console.log('Subscribed to /grid_map/occupancy_inflate (sensor_msgs/PointCloud2)');
    }
  }, [rosConnected, showGridMap, missionRunning]);

  const bboxText = bbox 
    ? `min(${bbox.min[0].toFixed(3)}, ${bbox.min[1].toFixed(3)}, ${bbox.min[2].toFixed(3)})  max(${bbox.max[0].toFixed(3)}, ${bbox.max[1].toFixed(3)}, ${bbox.max[2].toFixed(3)})`
    : "-";
  
  // 稳定回调，避免引起子组件重复加载
  const handleLoaded = useCallback(({ bbox, count }: { bbox: { min: { x: number; y: number; z: number }; max: { x: number; y: number; z: number } }; count: number }) => {
    setCount(count);
    const minArr: [number, number, number] = [bbox.min.x, bbox.min.y, bbox.min.z];
    const maxArr: [number, number, number] = [bbox.max.x, bbox.max.y, bbox.max.z];
    setBbox({ min: minArr, max: maxArr });
    // 自适应点大小上限：取对角线长度的 1% 作为上限（至少 0.01）
    const dx = maxArr[0] - minArr[0];
    const dy = maxArr[1] - minArr[1];
    const dz = maxArr[2] - minArr[2];
    const diag = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const newMax = Math.max(0.01, diag * 0.01);
    setPointSizeMax(newMax);
    setPointSize((p) => Math.min(p, newMax));
    // 同步为规划航点提供一个独立的尺寸上限（更显著）
    // 航点尺寸上限：更保守的范围，避免调节过大
    const pathMax = Math.min(0.1, Math.max(0.01, diag * 0.02));
    setPlannedPointSizeMax(pathMax);
    setPlannedPointSize((s) => {
      const cur = typeof s === 'number' ? s : 0.02;
      const minV = Math.max(0.002, pathMax * 0.2);
      const maxV = Math.max(minV, pathMax * 0.8);
      return Math.min(Math.max(cur, minV), maxV);
    });
  }, []);
  const handleLoadingChange = useCallback((v: boolean) => setLoading(v), []);
  // 当开启自动对齐时，确保切换开关或当前平面变化时都会对齐一次
  useEffect(() => {
    if (autoOrientPlane) {
      canvasRef.current?.orientToPlane?.(plannedDragPlane);
    }
  }, [autoOrientPlane, plannedDragPlane]);

  // 当任务变化时，尝试从任务的轨迹来源加载点，用于渲染（若编辑器提供工作中的点，则以编辑器为准覆盖）
  useEffect(() => {
    let cancelled = false;
    async function loadPlanned() {
      if (!selectedMission?.trajectory) { setPlannedPoints(null); return; }
      try {
        let text = "";
        if (selectedMission.trajectory.type === "file") {
          text = await selectedMission.trajectory.file.text();
        } else {
          const res = await fetch(selectedMission.trajectory.url);
          text = await res.text();
        }
        if (cancelled) return;
        const parsed = JSON.parse(text);
        const ptsUnknown = Array.isArray(parsed.points) ? parsed.points : [];
        const norm = ptsUnknown.map((p: unknown) => {
          const obj = (typeof p === "object" && p !== null ? p as Record<string, unknown> : {});
          const toNum = (v: unknown) => {
            const n = typeof v === "number" ? v : (typeof v === "string" ? Number(v) : NaN);
            return Number.isFinite(n) ? n : 0;
          };
          return { x: toNum(obj["x"]), y: toNum(obj["y"]), z: toNum(obj["z"]) };
        });
        setPlannedPoints(norm);
      } catch (e) {
        console.warn("Failed to load planned path from mission:", e);
        setPlannedPoints(null);
      }
    }
    loadPlanned();
    return () => { cancelled = true; };
  }, [selectedMission?.trajectory]);

  return (
    <div className="w-full h-[calc(100vh-2rem)] grid grid-cols-[320px_1fr] gap-4 p-4">
      <aside className="bg-card border border-border rounded-lg p-4 space-y-4 overflow-y-auto">
        {/* 任务配置 */}
        <TaskConfiguration 
          selectedMission={selectedMission}
          onMissionUpdate={handleMissionUpdate}
        />

        <Separator />

        {/* 任务控制 */}
        <div className="space-y-2">
          <Label>任务控制</Label>
          {selectedMission ? (
            <div className="text-sm space-y-1">
              <div>场景: {selectedMission.scene ? "✓" : "✗"}</div>
              <div>规划航线: {selectedMission.trajectory ? "✓" : "✗"}</div>
            </div>
          ) : (
            <div className="text-xs text-muted-foreground">未选择任务，也可直接订阅/取消订阅位姿。</div>
          )}
          <Button
            onClick={missionRunning ? stopMission : startMission}
            disabled={!rosConnected || !selectedMission || selectedMission.status !== 'ready' || !plannedPoints || plannedPoints.length === 0}
            className="w-full"
            variant={missionRunning ? "destructive" : "default"}
          >
            {missionRunning ? (
              <>
                <StopCircle className="h-4 w-4 mr-2" /> 停止任务
              </>
            ) : (
              <>
                <PlayCircle className="h-4 w-4 mr-2" /> 开始任务
              </>
            )}
          </Button>
          {!missionRunning && rosConnected && selectedMission?.status === 'ready' && (!plannedPoints || plannedPoints.length === 0) && (
            <div className="text-[11px] text-yellow-600 mt-1">未检测到规划航线，请先在任务配置中加载 .json</div>
          )}
        </div>

        {/* 无人机状态 */}
        {dronePosition && (
          <>
            <Separator />
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label>无人机位置</Label>
                <div className="flex items-center gap-2">
                  <Switch checked={followDrone} onCheckedChange={setFollowDrone} id="follow-drone" />
                  <Label htmlFor="follow-drone" className="text-xs cursor-pointer">跟随</Label>
                </div>
              </div>
              <div className="text-xs font-mono space-y-1">
                <div>X: {dronePosition.x.toFixed(3)} m</div>
                <div>Y: {dronePosition.y.toFixed(3)} m</div>
                <div>Z: {dronePosition.z.toFixed(3)} m</div>
                {dronePosition.velocity && (
                  <>
                    <Separator className="my-1" />
                    <div>速度: {Math.hypot(dronePosition.velocity.x, dronePosition.velocity.y, dronePosition.velocity.z).toFixed(2)} m/s</div>
                  </>
                )}
              </div>
            </div>
          </>
        )}

        {/* 任务进度统计 */}
        {selectedMission?.waypoints && selectedMission.waypoints.length > 0 && (
          <>
            <Separator />
            <div className="space-y-1 text-xs">
              <Label>任务进度</Label>
              {(() => {
                const completed = selectedMission.waypoints!.filter(w => w.status === 'completed').length;
                const total = selectedMission.waypoints!.length;
                const pct = Math.max(0, Math.min(100, total > 0 ? Math.round((completed / total) * 100) : 0));
                return (
                  <div className="space-y-1">
                    <div className="text-muted-foreground">
                      已完成 {completed} / {total}（{pct}%）
                      {typeof selectedMission.currentWaypointIndex === 'number' && (
                        <span>（当前目标: P{selectedMission.currentWaypointIndex}）</span>
                      )}
                    </div>
                    <div className="h-2 w-full rounded bg-muted overflow-hidden">
                      <div className="h-full bg-primary" style={{ width: `${pct}%` }} />
                    </div>
                  </div>
                );
              })()}
            </div>
          </>
        )}

        {/* 消息/订阅状态 */}
        <>
          <Separator />
          <div className="space-y-1 text-xs">
            <Label>ROS 消息状态</Label>
            <div className="text-muted-foreground">
              位置消息: {msgCount}
              {lastPoseAt && <span>（最近: {lastPoseAt.toLocaleTimeString()}）</span>}
            </div>
            <div className="text-muted-foreground">
              点云地图: {gridMapData ? `${gridMapData.pointCount?.toLocaleString() || 0} 点` : '无数据'}
              {lastGridMapAt && <span>（更新: {lastGridMapAt.toLocaleTimeString()}）</span>}
            </div>
          </div>
          </>
        

        <Separator />

        {/* 显示设置 */}
        <div className="space-y-3">
          <Label>显示设置</Label>
          
          <div className="space-y-2">
            <Label className="text-sm">点大小</Label>
            <Slider 
              value={[pointSize]} 
              min={0.001} 
              max={pointSizeMax} 
              step={Math.max(pointSizeMax / 200, 0.0001)} 
              onValueChange={(v) => setPointSize(v[0] ?? 0.01)} 
            />
            <div className="text-xs text-muted-foreground">{pointSize.toFixed(3)}（上限 {pointSizeMax.toFixed(3)}）</div>
          </div>

          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Switch checked={showGrid} onCheckedChange={setShowGrid} id="grid" />
              <Label htmlFor="grid">网格</Label>
            </div>
            <div className="flex items-center gap-2">
              <Switch checked={showAxes} onCheckedChange={setShowAxes} id="axes" />
              <Label htmlFor="axes">坐标轴</Label>
            </div>
          </div>

          <div className="flex items-center justify-between mt-2">
            <div className="flex items-center gap-2">
              <Switch checked={showSceneCloud} onCheckedChange={setShowSceneCloud} id="scene-cloud" />
              <Label htmlFor="scene-cloud">场景点云</Label>
            </div>
          </div>

          <div className="space-y-2">
            <Label className="text-sm">着色模式</Label>
            <div className="flex gap-2 flex-wrap">
              {(["none","intensity","height"] as const).map(m => (
                <Button key={m} size="sm" variant={colorMode===m?"default":"outline"} onClick={() => setColorMode(m)}>
                  {m === 'none' ? '禁用' : m === 'intensity' ? '强度' : '高度'}
                </Button>
              ))}
            </div>
            <div className="text-xs text-muted-foreground">
              禁用：显示文件自带颜色或纯色；强度/高度：应用伪着色增强对比。
            </div>
          </div>

        </div>

        <Separator />

        {/* 点云地图显示 */}
        <div className="space-y-3">
          <Label>点云地图</Label>
          <div className="flex items-center gap-2">
            <Switch checked={showGridMap} onCheckedChange={setShowGridMap} id="grid-map" />
            <Label htmlFor="grid-map">显示点云地图</Label>
          </div>
          {gridMapData && (
            <div className="text-xs text-muted-foreground space-y-1">
              <div>点数: {gridMapData.pointCount?.toLocaleString() || 0}</div>
              <div>尺寸: {gridMapData.width} × {gridMapData.height}</div>
              {gridMapData.frameId && <div>坐标系: {gridMapData.frameId}</div>}
            </div>
          )}
        </div>

        <Separator />

        {/* 规划航线显示与编辑 */}
        <div className="space-y-3">
          <Label>规划航线</Label>
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Switch checked={plannedVisible} onCheckedChange={setPlannedVisible} id="planned-visible" />
              <Label htmlFor="planned-visible">显示</Label>
            </div>
            <div className="flex items-center gap-2">
              <Switch checked={plannedEditable} onCheckedChange={setPlannedEditable} id="planned-edit" />
              <Label htmlFor="planned-edit">修改模式</Label>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <Label className="text-sm">拖拽平面</Label>
            <Select value={plannedDragPlane} onValueChange={(v) => {
              const plane = v as 'xy'|'xz'|'yz';
              setPlannedDragPlane(plane);
              if (autoOrientPlane) {
                // 自动对齐视角到对应平面
                canvasRef.current?.orientToPlane?.(plane);
              }
            }}>
              <SelectTrigger className="w-[140px]">
                <SelectValue placeholder="XY (保持Z)" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="xy">XY (保持Z)</SelectItem>
                <SelectItem value="xz">XZ (保持Y)</SelectItem>
                <SelectItem value="yz">YZ (保持X)</SelectItem>
              </SelectContent>
            </Select>
            <div className="flex items-center gap-2 ml-2">
              <Switch id="auto-orient" checked={autoOrientPlane} onCheckedChange={(v) => setAutoOrientPlane(v)} />
              <Label htmlFor="auto-orient">切换时自动对齐视角</Label>
            </div>
          </div>
          <div className="space-y-2">
            <Label className="text-sm">轨迹点尺寸</Label>
            <Slider 
              value={[plannedPointSize]} 
              min={Math.max(0.001, plannedPointSizeMax / 100)} 
              max={plannedPointSizeMax} 
              step={Math.max(plannedPointSizeMax / 100, 0.001)}
              onValueChange={(v) => setPlannedPointSize(v[0] ?? plannedPointSize)} 
            />
            <div className="text-xs text-muted-foreground">{plannedPointSize.toFixed(3)}（上限 {plannedPointSizeMax.toFixed(3)}）</div>
            <div className="text-[11px] text-muted-foreground leading-snug">
              提示：开启“修改模式”后
              <br />- 拖拽航点即可移动（保持该点 z 高度）
              <br />- 按住 Shift 在 3D 中点击可添加新点（添加到参考平面）
              <br />- 按住 Alt 点击航点可删除该点
            </div>
          </div>
        </div>

        <Separator />

        {/* 点云信息 */}
          <div className="space-y-1 text-xs text-muted-foreground">
            <div>点数: {count.toLocaleString()}</div>
            <div className="break-all">包围盒: {bboxText}</div>
          </div>

        <Button 
          className="w-full" 
          variant="secondary" 
          onClick={() => canvasRef.current?.fitToView()}
        >
          <Maximize className="h-4 w-4 mr-2" /> 视图适配
        </Button>
      </aside>

      <section className="bg-card border border-border rounded-lg relative overflow-hidden">
        {/* 右上角 ROS 连接按钮 */}
        <div className="absolute top-3 right-3 z-20 flex items-center gap-2">
          <Button size="sm" variant={rosConnected ? "secondary" : "default"} onClick={() => setRosModalOpen(true)}>
            <Plug className={`h-4 w-4 mr-2 ${rosConnected ? 'text-green-500' : ''}`} />
            {rosConnected ? '已连接' : '连接 ROS'}
          </Button>
          {/* 订阅简要状态 */}
          {rosConnected && (
            <span className="text-xs text-muted-foreground">
              {subscribed ? `已订阅 (${msgCount})` : '未订阅'}
            </span>
          )}
        </div>

        {/* 连接弹窗 */}
        {rosModalOpen && (
          <div className="absolute inset-0 z-30 flex items-center justify-center">
            <div className="absolute inset-0 bg-black/40" onClick={() => setRosModalOpen(false)} />
            <div className="relative bg-background border border-border rounded-lg shadow-xl w-[420px] max-w-[90vw] p-4">
              <div className="flex items-center justify-between mb-3">
                <div className="font-medium">ROS 连接</div>
                <Button size="icon" variant="ghost" onClick={() => setRosModalOpen(false)}>
                  <X className="h-4 w-4" />
                </Button>
              </div>
              <div className="space-y-3">
                <div className="space-y-2">
                  <Label htmlFor="ros-url">ROSBridge 地址</Label>
                  <Input
                    id="ros-url"
                    placeholder="ws://host:port"
                    value={rosUrl}
                    onChange={(e) => setRosUrl(e.target.value)}
                    disabled={rosConnected}
                  />
                  {/* HTTPS 与 ws 的混合内容提示 */}
                  {typeof window !== 'undefined' && window.location.protocol === 'https:' && rosUrl.startsWith('ws://') && (
                    <div className="text-[11px] text-yellow-500">
                      当前页面为 HTTPS，浏览器会阻止 ws:// 连接，请改用 wss:// 地址。
                    </div>
                  )}
                </div>
                <div className="flex gap-2">
                  <Button onClick={connectROS} disabled={rosConnected} className="flex-1">
                    连接
                  </Button>
                  <Button onClick={handleDisconnect} disabled={!rosConnected} variant="outline" className="flex-1">
                    断开
                  </Button>
                </div>
                <div className="text-xs text-muted-foreground">
                  状态: {rosConnected ? "✓ 已连接" : "✗ 未连接"}
                </div>
              </div>
            </div>
          </div>
        )}

        {/* 任务完成/失败提示弹窗 */}
        {completionOpen && completionData && (
          <div className="absolute inset-0 z-30 flex items-center justify-center">
            <div className="absolute inset-0 bg-black/40" onClick={() => setCompletionOpen(false)} />
            <div className="relative bg-background border border-border rounded-lg shadow-xl w-[420px] max-w-[90vw] p-4">
              <div className="flex items-center justify-between mb-3">
                <div className="font-medium">任务{completionData.status === 'completed' ? '完成' : '失败'}</div>
                <Button size="icon" variant="ghost" onClick={() => setCompletionOpen(false)}>
                  <X className="h-4 w-4" />
                </Button>
              </div>
              <div className="space-y-2 text-sm">
                <div className={completionData.status === 'completed' ? 'text-green-600' : 'text-red-600'}>
                  {completionData.status === 'completed' ? '任务已顺利完成' : '任务执行失败'}
                </div>
                {completionData.reason && (
                  <div className="text-muted-foreground">原因：{completionData.reason}</div>
                )}
              </div>
              <div className="mt-4 flex gap-2 justify-end">
                <Button variant="outline" onClick={() => setCompletionOpen(false)}>关闭</Button>
              </div>
            </div>
          </div>
        )}

        <div className="w-full h-full min-h-[400px] relative">
          {loading && (
            <div className="absolute inset-0 z-10 flex items-center justify-center bg-background/60 backdrop-blur-sm">
              <div className="flex flex-col items-center gap-3 text-muted-foreground">
                <div className="h-8 w-8 animate-spin rounded-full border-2 border-primary border-t-transparent" />
                <div className="text-xs">加载中...</div>
              </div>
            </div>
          )}
          <PCDCanvas
            ref={canvasRef}
            source={selectedMission?.scene ?? null}
            pointSize={pointSize}
            showGrid={showGrid}
            showAxes={showAxes}
            showSceneCloud={showSceneCloud}
            colorMode={colorMode}
            onLoadedAction={handleLoaded}
            onLoadingChange={handleLoadingChange}
            plannedPathPoints={plannedPoints ?? undefined}
            plannedPathVisible={plannedVisible}
            plannedPointSize={plannedPointSize}
            plannedPathEditable={plannedEditable}
            onPlannedPointsChange={(pts) => {
              setPlannedPoints(pts);
            }}
            plannedDragPlane={plannedDragPlane}
            dronePosition={dronePosition}
            followDrone={followDrone}
            waypoints={selectedMission?.waypoints}
            currentWaypointIndex={selectedMission?.currentWaypointIndex}
            realtimeFrames={realtimeFrames}
            showRealtimeCloud={showRealtimeCloud}
            gridMapData={gridMapData}
            showGridMap={showGridMap}
          />
        </div>
      </section>
    </div>
  );
}

// 任务配置组件
interface TaskConfigurationProps {
  selectedMission: Mission | null;
  onMissionUpdate: (mission: Mission) => void;
}

function TaskConfiguration({ selectedMission, onMissionUpdate }: TaskConfigurationProps) {
  const sceneFileRef = useRef<HTMLInputElement>(null);
  const trajectoryFileRef = useRef<HTMLInputElement>(null);

  const getStatusText = (status: string) => {
    const statusMap: Record<string, string> = {
      draft: '草稿',
      configured: '已配置',
      planning: '规划中', 
      ready: '就绪',
      running: '执行中',
      paused: '暂停',
      completed: '完成',
      failed: '失败'
    };
    return statusMap[status] || status;
  };

  const updateMission = (updates: Partial<Mission>) => {
    if (!selectedMission) return;
    const updated = { ...selectedMission, ...updates };
    
    // 自动更新状态逻辑
    if (!selectedMission.scene && updated.scene && updated.trajectory) {
      updated.status = 'configured';
    } else if (!selectedMission.trajectory && updated.trajectory && updated.scene) {
      updated.status = 'configured';
    }
    
    onMissionUpdate(updated);
  };

  const updateScene = (file: File) => {
    updateMission({ scene: { type: "file", file } });
  };

  const updateSceneURL = (url: string) => {
    if (!url.trim()) return;
    updateMission({ scene: { type: "url", url } });
  };

  const updateTrajectory = (file: File) => {
    updateMission({ trajectory: { type: "file", file } });
  };

  const updateTrajectoryURL = (url: string) => {
    if (!url.trim()) return;
    updateMission({ trajectory: { type: "url", url } });
  };

  const clearScene = () => {
    const { scene, ...rest } = selectedMission!;
    updateMission(rest);
  };

  const clearTrajectory = () => {
    const { trajectory, ...rest } = selectedMission!;
    updateMission(rest);
  };

  const startPlanning = () => {
    updateMission({ status: 'planning' });
  };

  const confirmTrajectory = () => {
    updateMission({ status: 'ready' });
  };

  if (!selectedMission) {
    return (
      <div className="space-y-4">
        <h2 className="text-lg font-semibold">任务配置</h2>
        <div className="text-sm text-muted-foreground">
          请从启动页选择一个任务
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      <div className="flex items-center justify-between">
        <h2 className="text-lg font-semibold">任务配置</h2>
        <span className="text-xs px-2 py-0.5 rounded-full bg-blue-100 text-blue-600">
          {getStatusText(selectedMission.status)}
        </span>
      </div>

      {/* 基本信息 */}
      <div className="text-sm space-y-1">
        <div>任务: {selectedMission.name}</div>
        <div>状态: {getStatusText(selectedMission.status)}</div>
        {selectedMission.waypoints && (
          <div>
            进度: {selectedMission.waypoints.filter(w => w.status === 'completed').length}/{selectedMission.waypoints.length}
          </div>
        )}
      </div>

      {/* 草稿/已配置阶段：配置场景和航线 */}
      {(selectedMission.status === 'draft' || selectedMission.status === 'configured') && (
        <>
          <Separator />
          
          {/* 场景配置 */}
          <div className="space-y-2">
            <Label className="flex items-center gap-2">
              场景文件 (.pcd)
              {selectedMission.scene && <span className="text-xs text-green-600">✓</span>}
            </Label>
            {!selectedMission.scene ? (
              <div className="flex gap-2">
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => sceneFileRef.current?.click()}
                >
                  选择文件
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => {
                    const url = prompt("输入场景文件URL:");
                    if (url) updateSceneURL(url);
                  }}
                >
                  输入URL
                </Button>
              </div>
            ) : (
              <div className="flex items-center justify-between text-xs">
                <span className="text-muted-foreground">
                  {selectedMission.scene.type === "file" 
                    ? selectedMission.scene.file.name 
                    : selectedMission.scene.url}
                </span>
                <Button size="sm" variant="ghost" onClick={clearScene}>
                  清除
                </Button>
              </div>
            )}
          </div>

          <Separator />

          {/* 航线配置 */}
          <div className="space-y-2">
            <Label className="flex items-center gap-2">
              规划航线 (.json)
              {selectedMission.trajectory && <span className="text-xs text-green-600">✓</span>}
            </Label>
            {!selectedMission.trajectory ? (
              <div className="flex gap-2">
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => trajectoryFileRef.current?.click()}
                >
                  选择文件
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => {
                    const url = prompt("输入航线文件URL:");
                    if (url) updateTrajectoryURL(url);
                  }}
                >
                  输入URL
                </Button>
              </div>
            ) : (
              <div className="flex items-center justify-between text-xs">
                <span className="text-muted-foreground">
                  {selectedMission.trajectory.type === "file" 
                    ? selectedMission.trajectory.file.name 
                    : selectedMission.trajectory.url}
                </span>
                <Button size="sm" variant="ghost" onClick={clearTrajectory}>
                  清除
                </Button>
              </div>
            )}
          </div>
        </>
      )}

      {/* 已配置阶段：可以开始规划 */}
      {selectedMission.status === 'configured' && (
        <>
          <Separator />
          <Button onClick={startPlanning} className="w-full">
            开始规划航线
          </Button>
        </>
      )}

      {/* 规划阶段：显示编辑提示和确认按钮 */}
      {selectedMission.status === 'planning' && (
        <>
          <Separator />
          <div className="space-y-2">
            <div className="text-sm text-muted-foreground">
              在3D视图中编辑航线：
              <br />• 双击选中航点
              <br />• 拖拽移动位置
              <br />• Shift+点击添加航点  
              <br />• Alt+点击删除航点
            </div>
            <Button onClick={confirmTrajectory} className="w-full">
              确认航线，准备执行
            </Button>
          </div>
        </>
      )}

      {/* 隐藏的文件输入 */}
      <input
        ref={sceneFileRef}
        type="file"
        accept=".pcd"
        className="hidden"
        onChange={(e) => {
          const file = e.target.files?.[0];
          if (file) updateScene(file);
        }}
      />
      <input
        ref={trajectoryFileRef}
        type="file"
        accept=".json"
        className="hidden"
        onChange={(e) => {
          const file = e.target.files?.[0];
          if (file) updateTrajectory(file);
        }}
      />
    </div>
  );
}
