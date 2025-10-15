"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { PCDCanvas, PCDCanvasHandle } from "./PCDCanvas";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Separator } from "@/components/ui/separator";
import { Maximize, PlayCircle, StopCircle } from "lucide-react";
import MissionManager from "@/components/mission/MissionManager";
import { Mission, DronePosition } from "@/types/mission";
// @ts-expect-error - roslib 没有官方类型定义
import ROSLIB from "roslib";

export default function MissionController() {
  const canvasRef = useRef<PCDCanvasHandle | null>(null);
  const [selectedMission, setSelectedMission] = useState<Mission | null>(null);
  const [pointSize, setPointSize] = useState(0.01);
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(true);
  const [loading, setLoading] = useState(false);
  // 已去除性能面板与航线绘制
  
  // ROS 连接
  const [rosUrl, setRosUrl] = useState("ws://101.132.193.179:7001");
  const [rosConnected, setRosConnected] = useState(false);
  const [missionRunning, setMissionRunning] = useState(false);
  const [dronePosition, setDronePosition] = useState<DronePosition | null>(null);
  const rosRef = useRef<typeof ROSLIB.Ros | null>(null);
  const odomTopicRef = useRef<typeof ROSLIB.Topic | null>(null);

  // 点云和包围盒信息
  const [count, setCount] = useState<number>(0);
  const [bbox, setBbox] = useState<{ min: [number, number, number]; max: [number, number, number] } | null>(null);

  // 需求：不绘制航线，仅保存任务里的轨迹文件或 URL，可用于业务逻辑但不渲染

  // ROS 连接管理
  const connectROS = useCallback(() => {
    if (!rosUrl) return;
    
    try {
      // 断开旧连接
      if (rosRef.current) {
        rosRef.current.close();
      }

      const ros = new ROSLIB.Ros({
        url: rosUrl
      });

      ros.on('connection', () => {
        console.log('Connected to ROS');
        setRosConnected(true);
      });

      ros.on('error', (error: unknown) => {
        console.error('ROS connection error:', error);
        setRosConnected(false);
      });

      ros.on('close', () => {
        console.log('ROS connection closed');
        setRosConnected(false);
      });

      rosRef.current = ros;
    } catch (error) {
      console.error('Failed to connect to ROS:', error);
      alert('ROS 连接失败');
    }
  }, [rosUrl]);

  const disconnectROS = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }
    setRosConnected(false);
  }, []);

  // 开始任务
  const startMission = useCallback(() => {
    if (!rosConnected || !rosRef.current) {
      alert('请先连接 ROS');
      return;
    }

    // 订阅无人机位置话题
    const odomTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/mavros/local_position/odom',
      messageType: 'nav_msgs/msg/Odometry'
    });

    odomTopic.subscribe((msg: {
      pose: { pose: { position: { x: number; y: number; z: number }; orientation: { x: number; y: number; z: number; w: number } } };
      twist: { twist: { linear: { x: number; y: number; z: number } } };
    }) => {
      const pos = msg.pose.pose.position;
      const orient = msg.pose.pose.orientation;
      const vel = msg.twist.twist.linear;

      setDronePosition({
        x: pos.x,
        y: pos.y,
        z: pos.z,
        orientation: {
          x: orient.x,
          y: orient.y,
          z: orient.z,
          w: orient.w
        },
        velocity: {
          x: vel.x,
          y: vel.y,
          z: vel.z
        }
      });
    });

    odomTopicRef.current = odomTopic;
    setMissionRunning(true);
  }, [rosConnected]);

  // 停止任务
  const stopMission = useCallback(() => {
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }
    setMissionRunning(false);
    setDronePosition(null);
  }, []);

  // 组件卸载时清理
  useEffect(() => {
    return () => {
      disconnectROS();
    };
  }, [disconnectROS]);

  const bboxText = bbox 
    ? `min(${bbox.min[0].toFixed(3)}, ${bbox.min[1].toFixed(3)}, ${bbox.min[2].toFixed(3)})  max(${bbox.max[0].toFixed(3)}, ${bbox.max[1].toFixed(3)}, ${bbox.max[2].toFixed(3)})`
    : "-";

  return (
    <div className="w-full h-[calc(100vh-2rem)] grid grid-cols-[320px_1fr] gap-4 p-4">
      <aside className="bg-card border border-border rounded-lg p-4 space-y-4 overflow-y-auto">
        {/* 任务管理 */}
        <MissionManager
          selectedMission={selectedMission}
          onMissionSelect={setSelectedMission}
        />

        <Separator />

        {/* ROS 连接 */}
        <div className="space-y-2">
          <Label>ROS 连接</Label>
          <Input
            placeholder="ws://host:port"
            value={rosUrl}
            onChange={(e) => setRosUrl(e.target.value)}
            disabled={rosConnected}
          />
          <div className="flex gap-2">
            <Button
              onClick={connectROS}
              disabled={rosConnected}
              className="flex-1"
            >
              连接
            </Button>
            <Button
              onClick={disconnectROS}
              disabled={!rosConnected}
              variant="outline"
              className="flex-1"
            >
              断开
            </Button>
          </div>
          <div className="text-xs text-muted-foreground">
            状态: {rosConnected ? "✓ 已连接" : "✗ 未连接"}
          </div>
        </div>

        <Separator />

        {/* 任务控制 */}
        <div className="space-y-2">
          <Label>任务控制</Label>
          {!selectedMission ? (
            <div className="text-sm text-muted-foreground">请先创建并选择任务</div>
          ) : (
            <>
              <div className="text-sm space-y-1">
                <div>场景: {selectedMission.scene ? "✓" : "✗"}</div>
                <div>航线: {selectedMission.trajectory ? "✓" : "✗"}</div>
              </div>
              <Button
                onClick={missionRunning ? stopMission : startMission}
                disabled={!rosConnected}
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
            </>
          )}
        </div>

        {/* 无人机状态 */}
        {dronePosition && (
          <>
            <Separator />
            <div className="space-y-2">
              <Label>无人机位置</Label>
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

        <Separator />

        {/* 显示设置 */}
        <div className="space-y-3">
          <Label>显示设置</Label>
          
          <div className="space-y-2">
            <Label className="text-sm">点大小</Label>
            <Slider 
              value={[pointSize]} 
              min={0.001} 
              max={0.1} 
              step={0.001} 
              onValueChange={(v) => setPointSize(v[0] ?? 0.01)} 
            />
            <div className="text-xs text-muted-foreground">{pointSize.toFixed(3)}</div>
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

          {/* 航线显示与性能监控已移除 */}
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
            onLoadedAction={({ bbox, count }) => {
              setCount(count);
              setBbox({
                min: [bbox.min.x, bbox.min.y, bbox.min.z],
                max: [bbox.max.x, bbox.max.y, bbox.max.z],
              });
            }}
            onLoadingChange={setLoading}
          />
        </div>
      </section>
    </div>
  );
}
