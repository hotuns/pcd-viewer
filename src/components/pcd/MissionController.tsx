"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { PCDCanvas, PCDCanvasHandle } from "./PCDCanvas";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Separator } from "@/components/ui/separator";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Maximize, PlayCircle, StopCircle, Plug, X } from "lucide-react";
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
  const [colorMode, setColorMode] = useState<"none" | "rgb" | "intensity" | "height">("none");
  const [pointSizeMax, setPointSizeMax] = useState(0.1);
  // 已去除性能面板与航线绘制
  
  // ROS 连接
  const [rosUrl, setRosUrl] = useState("ws://192.168.203.30:9999");
  const [rosConnected, setRosConnected] = useState(false);
  const [missionRunning, setMissionRunning] = useState(false);
  const [dronePosition, setDronePosition] = useState<DronePosition | null>(null);
  const rosRef = useRef<typeof ROSLIB.Ros | null>(null);
  const odomTopicRef = useRef<typeof ROSLIB.Topic | null>(null);
  const [rosModalOpen, setRosModalOpen] = useState(false);
  const [subscribed, setSubscribed] = useState(false);
  const [msgCount, setMsgCount] = useState(0);

  // 点云和包围盒信息
  const [count, setCount] = useState<number>(0);
  const [bbox, setBbox] = useState<{ min: [number, number, number]; max: [number, number, number] } | null>(null);
  // 规划航线（用于渲染）
  const [plannedPoints, setPlannedPoints] = useState<Array<{ x: number; y: number; z: number }> | null>(null);
  const [plannedVisible, setPlannedVisible] = useState(true);
  const [plannedEditable, setPlannedEditable] = useState(false);
  const [plannedPointSize, setPlannedPointSize] = useState(0.05);
  const [plannedPointSizeMax, setPlannedPointSizeMax] = useState(0.25);
  const [plannedDragPlane, setPlannedDragPlane] = useState<'xy'|'xz'|'yz'>('xy');
  const [autoOrientPlane, setAutoOrientPlane] = useState<boolean>(false);

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
        // 连接成功后关闭弹窗
        setRosModalOpen(false);
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

    // 订阅无人机位置话题（PoseStamped）
    const odomTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/odom_visualization/pose',
      messageType: 'geometry_msgs/PoseStamped'
    });

    console.log('Subscribing to /odom_visualization/pose (geometry_msgs/PoseStamped)');
    setMsgCount(0);
    odomTopic.subscribe((msg: {
      pose: { position: { x: number; y: number; z: number }; orientation: { x: number; y: number; z: number; w: number } };
    }) => {
      setMsgCount((c) => c + 1);
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
        // PoseStamped 无速度信息
      });
    });

    odomTopicRef.current = odomTopic;
    setMissionRunning(true);
    setSubscribed(true);
  }, [rosConnected]);

  // 停止任务
  const stopMission = useCallback(() => {
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }
    setMissionRunning(false);
    setDronePosition(null);
    setSubscribed(false);
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
    const pathMax = Math.max(0.01, diag * 0.05);
    setPlannedPointSizeMax(pathMax);
    setPlannedPointSize((s) => Math.min(Math.max(s, pathMax * 0.02), pathMax * 0.5));
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
        {/* 任务管理 */}
        <MissionManager
          selectedMission={selectedMission}
          onMissionSelect={setSelectedMission}
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

          {/* 航线显示与性能监控已移除 */}
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
              min={Math.max(0.001, plannedPointSizeMax / 200)} 
              max={plannedPointSizeMax} 
              step={plannedPointSizeMax / 200}
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
                  <Button onClick={disconnectROS} disabled={!rosConnected} variant="outline" className="flex-1">
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
          />
        </div>
      </section>
    </div>
  );
}
