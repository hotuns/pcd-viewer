"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type React from "react";
import { PCDCanvas, PCDCanvasHandle } from "./PCDCanvas";
import { TaskInfo } from "@/components/pcd/TaskInfo";
import { TaskConfigPanel } from "@/components/pcd/TaskConfigPanel";
import { MissionRuntimePanel } from "@/components/mission/MissionRuntimePanel";
import { MissionHomeForm } from "@/components/mission/MissionHomeForm";
import TrajectoryEditor from "@/components/mission/TrajectoryEditor";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Switch } from "@/components/ui/switch";
import { useMissionDatabase } from "@/hooks/useMissionDatabase";
import { useRosConnection } from "@/hooks/useRosConnection";
import { useMissionRuntime } from "@/hooks/useMissionRuntime";
import type { Mission, MissionHomePosition, PlannedPoint, Waypoint } from "@/types/mission";
import { PlugZap, SatelliteDish, Undo2, Crosshair, CheckCircle2, Edit3, FileCheck2, ArrowRight, ChevronsLeft, ChevronsRight, Bug, X } from "lucide-react";
import { normalizeTaskType } from "@/lib/taskTypes";
import { convertBodyPositionToViewer, convertViewerPositionToBody } from "@/lib/frameTransforms";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { cn } from "@/lib/utils";

interface MissionControllerProps {
  initialMission?: Mission | null;
  onBack?: () => void;
}

const DEFAULT_HOME: MissionHomePosition = {
  frameId: "map",
  position: { x: 0, y: 0, z: 0 },
  yaw: 0,
};
const createDefaultHomePose = (): MissionHomePosition => ({
  frameId: DEFAULT_HOME.frameId,
  position: { ...DEFAULT_HOME.position },
  yaw: DEFAULT_HOME.yaw,
});
const parseTrajectoryPoints = (raw: unknown): PlannedPoint[] => {
  if (!Array.isArray(raw)) return [];
  const toNum = (value: unknown) => {
    if (typeof value === "number") return value;
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : 0;
  };
  const toStr = (value: unknown) => (typeof value === "string" ? value : undefined);
  return raw.map((item) => {
    if (typeof item === "object" && item !== null) {
      const record = item as Record<string, unknown>;
      return {
        x: toNum(record.x),
        y: toNum(record.y),
        z: toNum(record.z),
        w: record.w == null ? undefined : toNum(record.w),
        t: record.t == null ? undefined : toNum(record.t),
        task_type: normalizeTaskType(record.task_type),
        info: toStr(record.info),
      };
    }
    return { x: 0, y: 0, z: 0 };
  });
};

export default function MissionController({ initialMission, onBack }: MissionControllerProps) {
  const [selectedMission, setSelectedMission] = useState<Mission | null>(initialMission ?? null);
  const [plannedPoints, setPlannedPoints] = useState<PlannedPoint[]>([]);
  const [homePose, setHomePose] = useState<MissionHomePosition>(initialMission?.home ?? DEFAULT_HOME);
  const [emergencyPose, setEmergencyPose] = useState<MissionHomePosition>(initialMission?.emergency ?? initialMission?.home ?? DEFAULT_HOME);
  const [pathLineWidth, setPathLineWidth] = useState(2);
  const [waypointSize, setWaypointSize] = useState(0.05);
  const [followDrone, setFollowDrone] = useState(false);
  const [showDetails, setShowDetails] = useState(true);
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);
  const [selectedPathIndex, setSelectedPathIndex] = useState<number | null>(null);
  const [showDebugPanel, setShowDebugPanel] = useState(false);
  const [editorHeight, setEditorHeight] = useState(280);
  const editorResizeRef = useRef<{ startY: number; startHeight: number } | null>(null);
  const canvasRef = useRef<PCDCanvasHandle | null>(null);
  const { updateMission: updateMissionDB } = useMissionDatabase();
  const { rosUrl, setRosUrl, rosConnected, rosRef, connectROS, disconnectROS, connectionError } = useRosConnection();

  const missionRuntime = useMissionRuntime({
    rosConnected,
    rosRef,
    mission: selectedMission,
    plannedPoints,
    home: selectedMission?.home ?? homePose,
    emergency: selectedMission?.emergency ?? emergencyPose,
    options: { droneId: 1, lowBatteryThreshold: 30 },
  });

  const handleMissionUpdate = useCallback(async (mission: Mission) => {
    setSelectedMission(mission);
    try {
      await updateMissionDB(mission.id, mission);
    } catch (error) {
      console.error("Failed to update mission", error);
    }
  }, [updateMissionDB]);

  useEffect(() => {
    setSelectedMission(initialMission ?? null);
    if (initialMission?.home) {
      setHomePose(initialMission.home);
    }
    if (initialMission?.emergency) {
      setEmergencyPose(initialMission.emergency);
    } else if (initialMission?.home) {
      setEmergencyPose(initialMission.home);
    }
  }, [initialMission]);

  useEffect(() => {
    if (selectedMission?.home) {
      setHomePose(selectedMission.home);
    }
  }, [selectedMission?.home]);

  useEffect(() => {
    if (selectedMission?.emergency) {
      setEmergencyPose(selectedMission.emergency);
    }
  }, [selectedMission?.emergency]);

  useEffect(() => {
    if (!selectedMission) return;
    if (!selectedMission.home) {
      setHomePose(createDefaultHomePose());
    }
    if (!selectedMission.emergency) {
      const defaultEmergency = createDefaultHomePose();
      setEmergencyPose(defaultEmergency);
      void handleMissionUpdate({ ...selectedMission, emergency: defaultEmergency });
    }
  }, [selectedMission, handleMissionUpdate]);

  const handleEmergencyChange = useCallback((pose: MissionHomePosition) => {
    setEmergencyPose(pose);
    if (selectedMission) {
      void handleMissionUpdate({ ...selectedMission, emergency: pose });
    }
  }, [handleMissionUpdate, selectedMission]);

  const fileToDataUrl = useCallback((file: File) => {
    return new Promise<string>((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => resolve(reader.result as string);
      reader.onerror = reject;
      reader.readAsDataURL(file);
    });
  }, []);

  const handleTrajectorySave = useCallback(async (file: File, _json: string) => {
    void _json;
    if (!selectedMission) return;
    try {
      const dataUrl = await fileToDataUrl(file);
      await handleMissionUpdate({ ...selectedMission, trajectory: { type: "url", url: dataUrl } });
      try {
        const text = await file.text();
        const parsed = JSON.parse(text);
        const pts = parseTrajectoryPoints(parsed.points);
        setPlannedPoints(pts);
      } catch (err) {
        console.error("Failed to reload trajectory after save", err);
      }
    } catch (error) {
      console.error("Failed to save trajectory file", error);
    }
  }, [fileToDataUrl, handleMissionUpdate, selectedMission]);

  const handleTrajectoryPointsChange = useCallback((points: PlannedPoint[]) => {
    setPlannedPoints(points.map((p) => ({
      x: p.x,
      y: p.y,
      z: p.z,
      w: p.w,
      t: p.t,
      task_type: p.task_type,
      info: p.info,
    })));
  }, []);

  const viewerPlannedPoints = useMemo(() => {
    return plannedPoints.map((p) => {
      const viewerPos = convertBodyPositionToViewer({ x: p.x, y: p.y, z: p.z });
      return { ...p, x: viewerPos.x, y: viewerPos.y, z: viewerPos.z };
    });
  }, [plannedPoints]);

  const handleCanvasPointsChange = useCallback((points: PlannedPoint[]) => {
    const rosPoints = points.map((p) => {
      const bodyPos = convertViewerPositionToBody({ x: p.x, y: p.y, z: p.z });
      return { ...p, x: bodyPos.x, y: bodyPos.y, z: bodyPos.z };
    });
    handleTrajectoryPointsChange(rosPoints);
  }, [handleTrajectoryPointsChange]);

  const handleEditorResizeMove = useCallback((event: MouseEvent) => {
    if (!editorResizeRef.current) return;
    const delta = event.clientY - editorResizeRef.current.startY;
    const next = Math.min(480, Math.max(180, editorResizeRef.current.startHeight - delta));
    setEditorHeight(next);
  }, []);

  const handleEditorResizeEnd = useCallback(() => {
    editorResizeRef.current = null;
    window.removeEventListener("mousemove", handleEditorResizeMove);
    window.removeEventListener("mouseup", handleEditorResizeEnd);
  }, [handleEditorResizeMove]);

  const handleEditorResizeStart = useCallback((event: React.MouseEvent<HTMLDivElement>) => {
    editorResizeRef.current = { startY: event.clientY, startHeight: editorHeight };
    window.addEventListener("mousemove", handleEditorResizeMove);
    window.addEventListener("mouseup", handleEditorResizeEnd);
  }, [editorHeight, handleEditorResizeEnd, handleEditorResizeMove]);

  useEffect(() => {
    return () => {
      window.removeEventListener("mousemove", handleEditorResizeMove);
      window.removeEventListener("mouseup", handleEditorResizeEnd);
    };
  }, [handleEditorResizeEnd, handleEditorResizeMove]);

  useEffect(() => {
    let cancelled = false;
    async function loadTrajectoryFromSource() {
      const traj = selectedMission?.trajectory;
      if (!traj) {
        setPlannedPoints([]);
        return;
      }
      try {
        let text = "";
        if (traj.type === "file") {
          text = await traj.file.text();
        } else {
          const res = await fetch(traj.url);
          text = await res.text();
        }
        if (cancelled) return;
        const parsed = JSON.parse(text);
        const pts = parseTrajectoryPoints(parsed.points);
        setPlannedPoints(pts);
      } catch (error) {
        console.error("Failed to load trajectory source", error);
        if (!cancelled) setPlannedPoints([]);
      }
    }
    loadTrajectoryFromSource();
    return () => {
      cancelled = true;
    };
  }, [selectedMission?.trajectory]);

  const isPlanning = selectedMission?.status === "planning";

  useEffect(() => {
    if (!plannedPoints || selectedPathIndex == null) return;
    if (selectedPathIndex >= plannedPoints.length) {
      setSelectedPathIndex(null);
    }
  }, [plannedPoints, selectedPathIndex]);

  useEffect(() => {
    if (!isPlanning) {
      setSelectedPathIndex(null);
    }
  }, [isPlanning]);

  const decoratedWaypoints = useMemo<Waypoint[]>(() => {
    if (!plannedPoints || plannedPoints.length === 0) return [];
    const completedCount = missionRuntime.progress?.completed ?? 0;
    return plannedPoints.map((pt, idx) => {
      let status: Waypoint["status"] = "pending";
      if (idx < completedCount) {
        status = "completed";
      } else if (idx === completedCount && completedCount < plannedPoints.length) {
        status = "active";
      }
      return {
        x: pt.x,
        y: pt.y,
        z: pt.z,
        yaw: pt.w,
        taskType: pt.task_type,
        id: `${selectedMission?.id ?? "mission"}-${idx}`,
        status,
        index: idx,
      };
    });
  }, [plannedPoints, missionRuntime.progress?.completed, selectedMission?.id]);

  const currentWaypointIndex = useMemo(() => {
    const completedCount = missionRuntime.progress?.completed ?? 0;
    if (!plannedPoints || plannedPoints.length === 0) return undefined;
    if (completedCount >= plannedPoints.length) return undefined;
    return completedCount;
  }, [missionRuntime.progress?.completed, plannedPoints]);

  const emergencyConfigured = !!selectedMission?.emergency;
  const sceneConfigured = !!selectedMission?.scene;
  const hasMandatoryPoints = emergencyConfigured;
  const canEnterPlanning = sceneConfigured && hasMandatoryPoints;
  const planningTip = useMemo(() => {
    if (!sceneConfigured) return "请先上传场景点云";
    if (!emergencyConfigured) return "请设置迫降点";
    return "进入规划阶段";
  }, [sceneConfigured, emergencyConfigured]);
  const missionPlanReady = useMemo(() => plannedPoints.length > 0 && !!selectedMission && hasMandatoryPoints, [plannedPoints.length, selectedMission, hasMandatoryPoints]);

  const currentMission = selectedMission;

  type MissionStage = "setup" | "planning" | "runtime";
  const missionStage = useMemo<MissionStage>(() => {
    const status = currentMission?.status;
    if (!status || status === "draft" || status === "configured") return "setup";
    if (status === "planning") return "planning";
    return "runtime";
  }, [currentMission?.status]);
  const stageMeta: Record<MissionStage, { label: string; description: string }> = {
    setup: {
      label: "配置阶段",
      description: "上传场景与航线、配置迫降点。",
    },
    planning: {
      label: "规划阶段",
      description: "拖拽航点、微调路线，完成后进入执行。",
    },
    runtime: {
      label: "执行阶段",
      description: "监控任务运行状态，必要时返航或重新规划。",
    },
  };

  const handleEnterPlanning = useCallback(() => {
    if (!currentMission || !canEnterPlanning) return;
    void handleMissionUpdate({ ...currentMission, status: "planning" });
  }, [canEnterPlanning, currentMission, handleMissionUpdate]);

  const handleManualComplete = useCallback(() => {
    if (!currentMission) return;
    const confirmed = window.confirm(`确认将任务「${currentMission.name}」标记为完成？`);
    if (!confirmed) return;
    void handleMissionUpdate({
      ...currentMission,
      status: "completed",
      completedAt: new Date(),
    });
  }, [currentMission, handleMissionUpdate]);

  const canManualComplete = !!currentMission && currentMission.status !== "completed" && currentMission.status !== "failed";

  return (
    <div className="w-full h-full flex flex-col bg-slate-950 text-slate-100 overflow-hidden">

      {connectionError && (
        <div className="px-4 py-2 bg-red-500/10 text-red-300 text-xs border-b border-red-500/30">
          {connectionError}
        </div>
      )}

    <div className="flex flex-1 min-h-0">
      <aside className={cn("border-r border-slate-800 bg-slate-900 flex flex-col min-h-0 transition-all duration-300", sidebarCollapsed ? "w-10" : "w-80")}>
        <div className="border-b border-slate-800 px-2 py-2 flex flex-col gap-2">
          <div className="flex items-center gap-2">
            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-7 w-7 text-slate-300"
                  onClick={() => setSidebarCollapsed((prev) => !prev)}
                >
                  {sidebarCollapsed ? <ChevronsRight className="h-4 w-4" /> : <ChevronsLeft className="h-4 w-4" />}
                </Button>
              </TooltipTrigger>
              <TooltipContent>{sidebarCollapsed ? "展开左侧面板" : "折叠左侧面板"}</TooltipContent>
            </Tooltip>
            {onBack && (
              <Tooltip>
                <TooltipTrigger asChild>
                  <Button variant="outline" size="icon" className="h-7 w-7" onClick={onBack}>
                    <Undo2 className="h-4 w-4" />
                  </Button>
                </TooltipTrigger>
                <TooltipContent>返回</TooltipContent>
              </Tooltip>
            )}
            {!sidebarCollapsed && (
              <div className="flex-1 min-w-0">
                <div className="text-sm font-semibold truncate">{currentMission?.name ?? "未命名任务"}</div>
                <div className="text-[10px] text-slate-500">{rosConnected ? "ROS 已连接" : "ROS 未连接"}</div>
              </div>
            )}
            <Tooltip>
              <TooltipTrigger asChild>
                <Button variant="outline" size="icon" className="h-7 w-7" onClick={handleManualComplete} disabled={!canManualComplete}>
                  <CheckCircle2 className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>标记完成</TooltipContent>
            </Tooltip>
          </div>
          {!sidebarCollapsed && (
            <div className="flex items-center gap-2">
              <Input
                value={rosUrl}
                onChange={(e) => setRosUrl(e.target.value)}
                placeholder="ws://host:port"
                className="h-8 w-full text-xs bg-slate-900 border-slate-700 text-slate-100"
              />
              <Tooltip>
                <TooltipTrigger asChild>
                  <Button onClick={rosConnected ? disconnectROS : connectROS} variant={rosConnected ? "destructive" : "secondary"} className="h-8 w-8 flex items-center justify-center">
                    {rosConnected ? <PlugZap className="h-4 w-4" /> : <SatelliteDish className="h-4 w-4" />}
                  </Button>
                </TooltipTrigger>
                <TooltipContent>{rosConnected ? "断开连接" : "连接 ROS"}</TooltipContent>
              </Tooltip>
            </div>
          )}
        </div>

        {!sidebarCollapsed ? (
          <div className="flex-1 overflow-y-auto p-4 space-y-4">
            <TaskInfo selectedMission={currentMission} phase={missionRuntime.phase} />

            {currentMission && (
              <div className="border border-slate-800 bg-slate-900/70 p-3 rounded">
                <div className="flex items-start justify-between gap-2">
                  <div>
                    <div className="text-[11px] text-slate-400 uppercase tracking-wide">当前阶段</div>
                    <div className="text-sm font-semibold">{stageMeta[missionStage].label}</div>
                  </div>
                  {missionStage === "runtime" && (
                    <Tooltip>
                      <TooltipTrigger asChild>
                        <Button
                          size="icon"
                          variant="outline"
                          className="h-8 w-8"
                          onClick={() => {
                            if (!currentMission) return;
                            void handleMissionUpdate({ ...currentMission, status: "planning" });
                          }}
                        >
                          <Edit3 className="h-4 w-4" />
                        </Button>
                      </TooltipTrigger>
                      <TooltipContent>返回规划</TooltipContent>
                    </Tooltip>
                  )}
                </div>
                <p className="text-[11px] text-slate-500 mt-2">{stageMeta[missionStage].description}</p>
              </div>
            )}

            {missionStage === "setup" && currentMission && (
              <div className="space-y-4">
                <div>
                  <div className="text-xs font-semibold text-slate-300 mb-2">场景点云</div>
                  <TaskConfigPanel selectedMission={currentMission} onMissionUpdate={handleMissionUpdate} />
                </div>
                <div>
                  <div className="text-xs font-semibold text-slate-300 mb-2">设置</div>
                  <MissionHomeForm label="迫降点" value={selectedMission?.emergency ?? emergencyPose} onChange={handleEmergencyChange} />
                </div>
                <div className="border border-slate-800 bg-slate-900/60 rounded px-3 py-2 flex items-center justify-between text-xs text-slate-400">
                  <div>
                    <div className="font-medium text-slate-200">进入规划阶段</div>
                    <p className="text-[11px] text-slate-500">上传场景/航线并配置迫降点后可开始规划。</p>
                  </div>
                  <Tooltip>
                    <TooltipTrigger asChild>
                      <span className="inline-flex">
                        <Button
                          size="icon"
                          variant="outline"
                          className="h-8 w-8"
                          disabled={!canEnterPlanning}
                          onClick={handleEnterPlanning}
                        >
                          <ArrowRight className="h-4 w-4" />
                        </Button>
                      </span>
                    </TooltipTrigger>
                    <TooltipContent>{planningTip}</TooltipContent>
                  </Tooltip>
                </div>
              </div>
            )}

            {missionStage === "planning" && currentMission && (
              <div className="space-y-3 border border-slate-800 rounded p-3 bg-slate-900/50 text-xs text-slate-400">
                {!hasMandatoryPoints && (
                  <div className="text-amber-300 bg-amber-500/10 rounded px-3 py-2 border border-amber-500/40">
                    请先设置迫降点后再编辑航线。
                  </div>
                )}
                <p>航点属性面板位于 3D 视图底部，可在完成调整后标记规划完成。</p>
                <Tooltip>
                  <TooltipTrigger asChild>
                    <Button
                      size="icon"
                      variant="outline"
                      className="h-8 w-8"
                      onClick={() => {
                        void handleMissionUpdate({ ...currentMission, status: "ready" });
                      }}
                    >
                      <FileCheck2 className="h-4 w-4" />
                    </Button>
                  </TooltipTrigger>
                  <TooltipContent>完成规划</TooltipContent>
                </Tooltip>
              </div>
            )}

            {missionStage === "runtime" && (
              <MissionRuntimePanel
                phase={missionRuntime.phase}
                rosConnected={rosConnected}
                hangar={missionRuntime.hangar}
                nestTelemetry={missionRuntime.nest}
                battery={missionRuntime.battery}
                progress={missionRuntime.progress}
                lastWaypoint={missionRuntime.lastWaypoint}
                events={missionRuntime.events}
                busyAction={missionRuntime.busyAction}
                missionReady={missionPlanReady}
                pendingCount={missionRuntime.pendingCount}
                canResume={missionRuntime.canResume}
                runtimeError={missionRuntime.runtimeError}
                onOpenHangar={missionRuntime.actions.openHangar}
                onCloseHangar={missionRuntime.actions.closeHangar}
                onUploadMission={missionRuntime.actions.uploadMission}
                onResumeMission={missionRuntime.actions.resumeMission}
                onExecuteMission={missionRuntime.actions.executeMission}
                onTakeoff={missionRuntime.actions.takeoff}
                onReturnHome={missionRuntime.actions.returnHome}
                onUploadEmergencyMission={missionRuntime.actions.uploadEmergencyMission}
                onLand={missionRuntime.actions.land}
                onArmOff={missionRuntime.actions.armOff}
              />
            )}
          </div>
        ) : (
          <div className="flex-1 flex flex-col items-center justify-center py-4 text-[10px] text-slate-500 select-none">
            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-8 w-8 text-slate-400"
                  onClick={() => setSidebarCollapsed(false)}
                >
                  <ChevronsRight className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>展开面板</TooltipContent>
            </Tooltip>
          </div>
        )}
      </aside>

        <main className="flex-1 relative bg-slate-900 min-h-0">
          <PCDCanvas
            ref={canvasRef}
            source={currentMission?.scene}
            livePointClouds={missionRuntime.pointClouds ?? []}
            plannedPathPoints={viewerPlannedPoints}
            plannedPathVisible
            plannedPathEditable={isPlanning}
            plannedPointSize={waypointSize}
            plannedPathLineWidth={pathLineWidth}
            onPlannedPointsChange={isPlanning ? handleCanvasPointsChange : undefined}
            selectedPointIndex={selectedPathIndex}
            onSelectPoint={isPlanning ? setSelectedPathIndex : undefined}
            waypoints={decoratedWaypoints}
            currentWaypointIndex={currentWaypointIndex}
            dronePosition={missionRuntime.dronePosition}
            followDrone={followDrone}
            showSceneCloud
            showGrid
            showAxes
          />
          <div className="absolute top-4 right-4 flex items-center gap-2 bg-slate-800/80 backdrop-blur rounded-full px-4 py-2 shadow">
            <label className="text-xs flex items-center gap-2 text-slate-200">
              <Switch checked={followDrone} onCheckedChange={setFollowDrone} />
              跟随
            </label>
            <Button variant="outline" size="sm" onClick={() => canvasRef.current?.fitToView?.()} className="flex items-center gap-1">
              <Crosshair className="h-3.5 w-3.5" /> 自适应
            </Button>
            <Button variant="outline" size="sm" className="flex items-center gap-1 text-xs" onClick={() => setShowDetails((prev) => !prev)}>
              {showDetails ? "隐藏面板" : "显示面板"}
            </Button>
            <Button variant={showDebugPanel ? "default" : "outline"} size="sm" className="flex items-center gap-1 text-xs" onClick={() => setShowDebugPanel((prev) => !prev)}>
              <Bug className="h-3.5 w-3.5" /> 调试
            </Button>
          </div>

          {showDebugPanel && (
            <div className="absolute top-20 right-4 w-96 bg-slate-950/95 border border-slate-800 rounded-lg shadow-2xl p-3 space-y-2 text-xs">
              <div className="flex items-center justify-between">
                <div className="font-semibold text-slate-200">ROS 调试面板</div>
                <div className="flex items-center gap-2">
                  <Button variant="ghost" size="sm" onClick={() => missionRuntime.clearDebugLogs?.()}>
                    清空
                  </Button>
                  <Button variant="ghost" size="icon" onClick={() => setShowDebugPanel(false)}>
                    <X className="h-4 w-4" />
                  </Button>
                </div>
              </div>
              <div className="max-h-64 overflow-y-auto space-y-2 font-mono text-[11px] pr-1">
                {missionRuntime.debugLogs && missionRuntime.debugLogs.length > 0 ? (
                  missionRuntime.debugLogs.map((log) => (
                    <div key={log.id} className="border border-slate-800 rounded p-2 bg-slate-900/70">
                      <div className="flex items-center justify-between text-slate-400">
                        <span>{log.label}</span>
                        <span>{log.timestamp.toLocaleTimeString()}</span>
                      </div>
                      <pre className="mt-1 whitespace-pre-wrap break-all text-slate-200">{log.payload}</pre>
                    </div>
                  ))
                ) : (
                  <div className="text-slate-500 text-center py-4">暂无 ROS 消息</div>
                )}
              </div>
            </div>
          )}

          {showDetails && (
            <div className="absolute inset-x-4 bottom-4 bg-slate-950/95 border border-slate-800 shadow-2xl rounded-md">
              <div className="px-4 py-2 flex flex-wrap items-center gap-4 border-b border-slate-800 text-xs text-slate-400">
                <div className="text-sm font-semibold text-slate-200">航点编辑</div>
                <label className="flex items-center gap-2">
                  宽度
                  <Input
                    type="number"
                    min="1"
                    step="0.5"
                    value={pathLineWidth}
                    onChange={(e) => {
                      const value = Number(e.target.value);
                      setPathLineWidth(Number.isFinite(value) ? Math.max(1, value) : 1);
                    }}
                    className="h-7 w-20 text-xs bg-slate-900 border-slate-700 text-slate-100"
                  />
                </label>
                <label className="flex items-center gap-2">
                  点大小
                  <Input
                    type="number"
                    min="0.01"
                    step="0.01"
                    value={waypointSize}
                    onChange={(e) => {
                      const value = Number(e.target.value);
                      setWaypointSize(Number.isFinite(value) ? Math.max(0.01, value) : 0.05);
                    }}
                    className="h-7 w-20 text-xs bg-slate-900 border-slate-700 text-slate-100"
                  />
                </label>
                <div className="ml-auto text-slate-500">{plannedPoints.length} 个航点</div>
              </div>
              <div className="mt-3 border-t border-slate-800 relative overflow-hidden">
                <div
                  className="absolute -top-2 left-0 right-0 h-3 cursor-row-resize flex items-center justify-center"
                  onMouseDown={handleEditorResizeStart}
                >
                  <div className="w-10 h-0.5 bg-slate-600 rounded-full" />
                </div>
                <div className="h-full overflow-y-auto" style={{ height: `${editorHeight}px` }}>
                  <TrajectoryEditor
                    mission={currentMission!}
                    onSaveAction={handleTrajectorySave}
                    onPointsChangeAction={handleTrajectoryPointsChange}
                    externalPoints={plannedPoints}
                    editable={isPlanning && hasMandatoryPoints}
                    selectedIndex={selectedPathIndex}
                    onSelectIndex={setSelectedPathIndex}
                    lockAnchors={false}
                  />
                </div>
              </div>
            </div>
          )}
        </main>
      </div>
    </div>
  );
}
