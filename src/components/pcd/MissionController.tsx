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
import type { Mission, MissionHomePosition, Waypoint } from "@/types/mission";
import { cn } from "@/lib/utils";
import { PlugZap, SatelliteDish, Undo2, Crosshair, MapPin, CheckCircle2 } from "lucide-react";

interface MissionControllerProps {
  initialMission?: Mission | null;
  onBack?: () => void;
}

const DEFAULT_HOME: MissionHomePosition = {
  frameId: "map",
  position: { x: 0, y: 0, z: 0 },
  yaw: 0,
};

const parseTrajectoryPoints = (raw: unknown): Array<{ x: number; y: number; z: number }> => {
  if (!Array.isArray(raw)) return [];
  const toNum = (value: unknown) => {
    if (typeof value === "number") return value;
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : 0;
  };
  return raw.map((item) => {
    if (typeof item === "object" && item !== null) {
      const record = item as Record<string, unknown>;
      return {
        x: toNum(record.x),
        y: toNum(record.y),
        z: toNum(record.z),
      };
    }
    return { x: 0, y: 0, z: 0 };
  });
};

export default function MissionController({ initialMission, onBack }: MissionControllerProps) {
  const [selectedMission, setSelectedMission] = useState<Mission | null>(initialMission ?? null);
  const [plannedPoints, setPlannedPoints] = useState<Array<{ x: number; y: number; z: number }>>([]);
  const [homePose, setHomePose] = useState<MissionHomePosition>(initialMission?.home ?? DEFAULT_HOME);
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
    options: { droneId: 1, lowBatteryThreshold: 30 },
  });

  useEffect(() => {
    setSelectedMission(initialMission ?? null);
    if (initialMission?.home) {
      setHomePose(initialMission.home);
    }
  }, [initialMission]);

  useEffect(() => {
    if (selectedMission?.home) {
      setHomePose(selectedMission.home);
    }
  }, [selectedMission?.home]);

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

  const handleTrajectoryPointsChange = useCallback((points: Array<{ x: number; y: number; z: number; t?: number }>) => {
    setPlannedPoints(points.map((p) => ({ x: p.x, y: p.y, z: p.z })));
  }, []);
  const handleCanvasPointsChange = useCallback((points: Array<{ x: number; y: number; z: number }>) => {
    handleTrajectoryPointsChange(points);
  }, [handleTrajectoryPointsChange]);

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

  const missionPlanReady = useMemo(() => plannedPoints.length > 0 && !!selectedMission, [plannedPoints.length, selectedMission]);

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
    <div className="w-full h-screen flex flex-col bg-background">
      <header className="flex items-center justify-between px-6 py-4 border-b bg-card/80 backdrop-blur">
        <div className="flex items-center gap-3">
          <Button
            variant="ghost"
            size="sm"
            onClick={onBack}
            className={cn("transition-opacity", !onBack && "opacity-0 pointer-events-none")}
          >
            <Undo2 className="h-4 w-4 mr-1" /> 返回
          </Button>
          <div>
            <div className="text-lg font-semibold">
              {currentMission?.name ?? "未命名任务"}
            </div>
            <div className="text-xs text-muted-foreground">{rosConnected ? "ROS 已连接" : "ROS 未连接"}</div>
          </div>
        </div>
        <div className="flex items-center gap-2">
          <Input
            value={rosUrl}
            onChange={(e) => setRosUrl(e.target.value)}
            placeholder="ws://host:port"
            className="w-64 text-xs"
          />
          <Button onClick={rosConnected ? disconnectROS : connectROS} variant={rosConnected ? "destructive" : "default"} className="flex items-center gap-2">
            {rosConnected ? <PlugZap className="h-4 w-4" /> : <SatelliteDish className="h-4 w-4" />}
            {rosConnected ? "断开" : "连接"}
          </Button>
          {canManualComplete && (
            <Button variant="outline" size="sm" className="flex items-center gap-1 text-xs" onClick={handleManualComplete}>
              <CheckCircle2 className="h-4 w-4" /> 标记完成
            </Button>
          )}
        </div>
      </header>

      {connectionError && (
        <div className="px-6 py-2 bg-destructive/10 text-destructive text-xs border-b border-destructive/30">
          {connectionError}，请检查 ws 地址并重试。
        </div>
      )}

      <div className="flex flex-1 flex-col lg:flex-row overflow-hidden min-h-0">
        <div className="w-full lg:flex-[0_0_420px] lg:max-w-[420px] bg-card/40 backdrop-blur flex flex-col min-h-[320px] lg:min-h-0 border-b lg:border-b-0 lg:border-r h-full">
          <div className="flex-1 overflow-y-auto">
            <div className="p-5 space-y-6 pb-8">
              <TaskInfo selectedMission={currentMission} phase={missionRuntime.phase} />

              {currentMission && (
                <div className="rounded-lg border border-border/60 bg-card/70 p-4">
                  <div className="flex items-start justify-between gap-3">
                    <div>
                      <div className="text-xs text-muted-foreground uppercase tracking-wide">当前阶段</div>
                      <div className="text-sm font-semibold">{stageMeta[missionStage].label}</div>
                      <p className="text-xs text-muted-foreground mt-1">{stageMeta[missionStage].description}</p>
                    </div>
                    {missionStage === "runtime" && (
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          void handleMissionUpdate({ ...currentMission, status: "planning" });
                        }}
                      >
                        进入规划
                      </Button>
                    )}
                  </div>
                </div>
              )}

              {missionStage === "setup" && currentMission && (
                <div className="space-y-5">
                  <div className="space-y-4">
                    <div className="text-xs font-semibold text-muted-foreground">场景点云</div>
                    <TaskConfigPanel selectedMission={currentMission} onMissionUpdate={handleMissionUpdate} />
                  </div>
                  <div className="space-y-4">
                    <div className="text-xs font-semibold text-muted-foreground">HomePos</div>
                    <MissionHomeForm value={selectedMission?.home ?? homePose} onChange={handleHomeChange} />
                  </div>
                  <div className="rounded-lg border border-dashed border-primary/40 p-3 text-xs text-muted-foreground">
                    完成配置后，即可进入规划阶段。
                    <Button
                      size="sm"
                      className="mt-3 w-full"
                      onClick={() => {
                        if (!currentMission) return;
                        void handleMissionUpdate({ ...currentMission, status: "planning" });
                      }}
                    >
                      进入规划
                    </Button>
                  </div>
                </div>
              )}

              {missionStage === "planning" && currentMission && (
                <div className="space-y-4 pb-4">
                  <div className="flex items-center justify-between text-xs text-muted-foreground font-semibold">
                    <span className="flex items-center gap-2">
                      <MapPin className="h-3.5 w-3.5" /> 航线编辑
                    </span>
                    <div className="flex items-center gap-2">
                      <Button
                        size="sm"
                        variant="outline"
                        className="h-7 text-xs"
                        onClick={() => {
                          void handleMissionUpdate({ ...currentMission, status: "ready" });
                        }}
                      >
                        完成规划
                      </Button>
                    </div>
                  </div>
                  <TrajectoryEditor
                    mission={currentMission}
                    onSaveAction={handleTrajectorySave}
                    onPointsChangeAction={handleTrajectoryPointsChange}
                    externalPoints={plannedPoints}
                    editable={isPlanning}
                    selectedIndex={selectedPathIndex}
                    onSelectIndex={setSelectedPathIndex}
                  />
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
                  onLand={missionRuntime.actions.land}
                  onArmOff={missionRuntime.actions.armOff}
                />
              )}
            </div>
          </div>
        </div>

        <main className="flex-1 relative bg-muted/30 min-h-[320px] lg:min-h-0">
          <PCDCanvas
            ref={canvasRef}
            source={currentMission?.scene}
            livePointClouds={missionRuntime.pointClouds ?? []}
            plannedPathPoints={plannedPoints}
            plannedPathVisible
            plannedPathEditable={isPlanning}
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
          <div className="absolute top-4 right-4 flex items-center gap-2 bg-background/80 backdrop-blur rounded-full px-4 py-2 shadow">
            <label className="text-xs flex items-center gap-2">
              <Switch checked={followDrone} onCheckedChange={setFollowDrone} />
              跟随无人机
            </label>
            <Button variant="outline" size="sm" onClick={() => canvasRef.current?.fitToView?.()} className="flex items-center gap-1">
              <Crosshair className="h-3.5 w-3.5" /> 自适应
            </Button>
          </div>
        </main>
      </div>
    </div>
  );
}
