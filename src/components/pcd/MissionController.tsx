"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
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
import { cn } from "@/lib/utils";
import { PlugZap, SatelliteDish, Undo2, Crosshair, MapPin, CheckCircle2, Edit3, FileCheck2, ArrowRight } from "lucide-react";
import { normalizeTaskType } from "@/lib/taskTypes";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";

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
const createAnchorPoint = (pose: MissionHomePosition): PlannedPoint => ({
  x: pose.position.x,
  y: pose.position.y,
  z: pose.position.z,
  w: pose.yaw,
  t: 0,
  task_type: 0,
  info: pose.frameId ?? "home",
});

const isSameAnchor = (a: PlannedPoint | undefined, b: PlannedPoint | null | undefined) => {
  if (!a || !b) return false;
  return (
    Math.abs(a.x - b.x) < 1e-3 &&
    Math.abs(a.y - b.y) < 1e-3 &&
    Math.abs(a.z - b.z) < 1e-3 &&
    Math.abs((a.w ?? 0) - (b.w ?? 0)) < 1e-3
  );
};

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
  const [showDetails, setShowDetails] = useState(true);
  const [followDrone, setFollowDrone] = useState(false);
  const [selectedPathIndex, setSelectedPathIndex] = useState<number | null>(null);
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
    let changed = false;
    const nextMission: Mission = { ...selectedMission };
    if (!selectedMission.home) {
      const defaultHome = createDefaultHomePose();
      nextMission.home = defaultHome;
      setHomePose(defaultHome);
      changed = true;
    }
    if (!selectedMission.emergency) {
      const defaultEmergency = createDefaultHomePose();
      nextMission.emergency = defaultEmergency;
      setEmergencyPose(defaultEmergency);
      changed = true;
    }
    if (changed) {
      void handleMissionUpdate(nextMission);
    }
  }, [selectedMission, handleMissionUpdate]);

  const handleMissionUpdate = useCallback(async (mission: Mission) => {
    setSelectedMission(mission);
    try {
      await updateMissionDB(mission.id, mission);
    } catch (error) {
      console.error("Failed to update mission", error);
    }
  }, [updateMissionDB]);

  const handleHomeChange = useCallback((home: MissionHomePosition) => {
    setHomePose(home);
    if (selectedMission) {
      void handleMissionUpdate({ ...selectedMission, home });
    }
  }, [handleMissionUpdate, selectedMission]);

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
        setPlannedPoints(ensureAnchoredPoints(pts));
      } catch (err) {
        console.error("Failed to reload trajectory after save", err);
      }
    } catch (error) {
      console.error("Failed to save trajectory file", error);
    }
  }, [fileToDataUrl, handleMissionUpdate, selectedMission]);

  const homeAnchor = useMemo(() => createAnchorPoint(homePose), [homePose]);
  const homeConfigured = !!selectedMission?.home;
  const ensureAnchoredPoints = useCallback((points: PlannedPoint[]) => {
    if (!homeConfigured) {
      return points;
    }
    const startMatches = points.length > 0 && isSameAnchor(points[0], homeAnchor);
    const endMatches = points.length > 1 && isSameAnchor(points[points.length - 1], homeAnchor);
    if (startMatches && endMatches) {
      return points;
    }
    const startIndex = startMatches ? 1 : 0;
    const endIndex = endMatches ? points.length - 1 : points.length;
    const middle = points.slice(startIndex, endIndex);
    if (middle.length === 0) {
      return [homeAnchor, homeAnchor];
    }
    return [homeAnchor, ...middle, homeAnchor];
  }, [homeAnchor, homeConfigured]);

  useEffect(() => {
    if (homeConfigured) {
      setPlannedPoints((prev) => ensureAnchoredPoints(prev));
    }
  }, [homeConfigured, ensureAnchoredPoints]);

  const handleTrajectoryPointsChange = useCallback((points: PlannedPoint[]) => {
    setPlannedPoints(ensureAnchoredPoints(points.map((p) => ({
      x: p.x,
      y: p.y,
      z: p.z,
      w: p.w,
      t: p.t,
      task_type: p.task_type,
      info: p.info,
    }))));
  }, [ensureAnchoredPoints]);
  const handleCanvasPointsChange = useCallback((points: PlannedPoint[]) => {
    handleTrajectoryPointsChange(points);
  }, [handleTrajectoryPointsChange]);

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
  }, [selectedMission?.trajectory, ensureAnchoredPoints]);

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
  const trajectoryConfigured = !!selectedMission?.trajectory;
  const hasMandatoryPoints = homeConfigured && emergencyConfigured;
  const canEnterPlanning = sceneConfigured && trajectoryConfigured && hasMandatoryPoints;
  const planningTip = useMemo(() => {
    if (!sceneConfigured) return "请先上传场景点云";
    if (!trajectoryConfigured) return "请上传航线文件";
    if (!homeConfigured) return "请设置 HomePos";
    if (!emergencyConfigured) return "请设置迫降点";
    return "进入规划阶段";
  }, [sceneConfigured, trajectoryConfigured, homeConfigured, emergencyConfigured]);
  const missionPlanReady = useMemo(() => plannedPoints.length > 0 && !!selectedMission && hasMandatoryPoints, [plannedPoints.length, selectedMission, hasMandatoryPoints]);

  const currentMission = selectedMission;

  type MissionStage = "setup" | "planning" | "runtime";
  const missionStage = useMemo<MissionStage>(() => {
    const status = currentMission?.status;
    if (!status || status === "draft" || status === "configured") return "setup";
    if (status === "planning") return "planning";
    return "runtime";
  }, [currentMission?.status]);
  const showTrajectorySidebar = missionStage === "planning" && !!currentMission;

  const stageMeta: Record<MissionStage, { label: string; description: string }> = {
    setup: {
      label: "配置阶段",
      description: "上传场景与航线、设置 HomePos。",
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
    <div className="w-full min-h-screen flex flex-col bg-slate-950 text-slate-100">
      <div className="px-4 py-3 border-b border-slate-800 bg-slate-900 flex items-center justify-between gap-4">
        <div className="flex items-center gap-3">
          {onBack && (
            <Tooltip>
              <TooltipTrigger asChild>
                <Button variant="outline" size="icon" className="h-8 w-8" onClick={onBack}>
                  <Undo2 className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>返回</TooltipContent>
            </Tooltip>
          )}
          <div>
            <div className="text-sm font-semibold">{currentMission?.name ?? "未命名任务"}</div>
            <div className="text-[11px] text-slate-500">{rosConnected ? "ROS 已连接" : "ROS 未连接"}</div>
          </div>
        </div>
        <div className="flex items-center gap-2">
          <Input
            value={rosUrl}
            onChange={(e) => setRosUrl(e.target.value)}
            placeholder="ws://host:port"
            className="w-56 h-8 text-xs bg-slate-900 border-slate-700 text-slate-100"
          />
          <Tooltip>
            <TooltipTrigger asChild>
              <Button onClick={rosConnected ? disconnectROS : connectROS} variant={rosConnected ? "destructive" : "secondary"} className="h-8 w-8 flex items-center justify-center">
                {rosConnected ? <PlugZap className="h-4 w-4" /> : <SatelliteDish className="h-4 w-4" />}
              </Button>
            </TooltipTrigger>
            <TooltipContent>{rosConnected ? "断开连接" : "连接 ROS"}</TooltipContent>
          </Tooltip>
          <Tooltip>
            <TooltipTrigger asChild>
              <Button variant="outline" size="icon" className="h-8 w-8" onClick={handleManualComplete} disabled={!canManualComplete}>
                <CheckCircle2 className="h-4 w-4" />
              </Button>
            </TooltipTrigger>
            <TooltipContent>标记完成</TooltipContent>
          </Tooltip>
        </div>
      </div>

      {connectionError && (
        <div className="px-4 py-2 bg-red-500/10 text-red-300 text-xs border-b border-red-500/30">
          {connectionError}
        </div>
      )}

      <div className="flex flex-1 min-h-0">
        <aside className="w-80 border-r border-slate-800 bg-slate-900 flex flex-col min-h-0">
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
                  <div className="text-xs font-semibold text-slate-300 mb-2">HomePos</div>
                  <MissionHomeForm value={selectedMission?.home ?? homePose} onChange={handleHomeChange} />
                </div>
                <div>
                  <div className="text-xs font-semibold text-slate-300 mb-2">迫降点</div>
                  <MissionHomeForm label="迫降点" value={selectedMission?.emergency ?? emergencyPose} onChange={handleEmergencyChange} />
                </div>
                <div className="border border-slate-800 bg-slate-900/60 rounded px-3 py-2 flex items-center justify-between text-xs text-slate-400">
                  <div>
                    <div className="font-medium text-slate-200">进入规划阶段</div>
                    <p className="text-[11px] text-slate-500">上传场景/航线并配置 HomePos、迫降点后可开始规划。</p>
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
                    请先配置 HomePos 与迫降点后再编辑航线。
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
                onUploadReturnMission={missionRuntime.actions.uploadReturnMission}
                onUploadEmergencyMission={missionRuntime.actions.uploadEmergencyMission}
                onLand={missionRuntime.actions.land}
                onArmOff={missionRuntime.actions.armOff}
              />
            )}
          </div>
        </aside>

        <main className="flex-1 relative bg-slate-900 min-h-0">
          <PCDCanvas
            ref={canvasRef}
            source={currentMission?.scene}
            livePointClouds={missionRuntime.pointClouds ?? []}
            plannedPathPoints={plannedPoints}
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
          </div>

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
              <div className="max-h-48 overflow-y-auto">
                <TrajectoryEditor
                  mission={currentMission!}
                  onSaveAction={handleTrajectorySave}
                  onPointsChangeAction={handleTrajectoryPointsChange}
                  externalPoints={plannedPoints}
                  editable={isPlanning && hasMandatoryPoints}
                  selectedIndex={selectedPathIndex}
                  onSelectIndex={setSelectedPathIndex}
                  lockAnchors={homeConfigured}
                />
              </div>
            </div>
          )}
        </main>
      </div>
    </div>
  );
}
