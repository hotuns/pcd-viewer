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
import { ScrollArea } from "@/components/ui/scroll-area";
import { Switch } from "@/components/ui/switch";
import { useMissionDatabase } from "@/hooks/useMissionDatabase";
import { useRosConnection } from "@/hooks/useRosConnection";
import { useMissionRuntime } from "@/hooks/useMissionRuntime";
import type { Mission, MissionHomePosition, Waypoint } from "@/types/mission";
import { cn } from "@/lib/utils";
import { PlugZap, SatelliteDish, Undo2, Crosshair, MapPin } from "lucide-react";

interface MissionControllerProps {
  initialMission?: Mission | null;
  onBack?: () => void;
}

const DEFAULT_HOME: MissionHomePosition = {
  frameId: "map",
  position: { x: 0, y: 0, z: 0 },
  yaw: 0,
};

export default function MissionController({ initialMission, onBack }: MissionControllerProps) {
  const [selectedMission, setSelectedMission] = useState<Mission | null>(initialMission ?? null);
  const [plannedPoints, setPlannedPoints] = useState<Array<{ x: number; y: number; z: number }>>([]);
  const [homePose, setHomePose] = useState<MissionHomePosition>(initialMission?.home ?? DEFAULT_HOME);
  const [followDrone, setFollowDrone] = useState(false);
  const canvasRef = useRef<PCDCanvasHandle | null>(null);
  const { updateMission: updateMissionDB } = useMissionDatabase();
  const { rosUrl, setRosUrl, rosConnected, rosRef, connectROS, disconnectROS } = useRosConnection();

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

  const handleTrajectorySave = useCallback((file: File, _json: string) => {
    void _json;
    if (!selectedMission) return;
    void handleMissionUpdate({ ...selectedMission, trajectory: { type: "file", file } });
  }, [handleMissionUpdate, selectedMission]);

  const handleTrajectoryPointsChange = useCallback((points: Array<{ x: number; y: number; z: number; t?: number }>) => {
    setPlannedPoints(points.map((p) => ({ x: p.x, y: p.y, z: p.z })));
  }, []);

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
        </div>
      </header>

      <div className="flex flex-1 overflow-hidden">
        <aside className="w-[420px] border-r bg-card/40 backdrop-blur flex flex-col min-h-0">
          <ScrollArea className="flex-1">
            <div className="p-5 space-y-6">
              <TaskInfo selectedMission={currentMission} phase={missionRuntime.phase} />

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

              <div className="space-y-4">
                <div className="text-xs font-semibold text-muted-foreground">场景点云</div>
                <TaskConfigPanel selectedMission={currentMission} onMissionUpdate={handleMissionUpdate} />
              </div>

              <div className="space-y-4">
                <div className="text-xs font-semibold text-muted-foreground">HomePos</div>
                <MissionHomeForm value={selectedMission?.home ?? homePose} onChange={handleHomeChange} />
              </div>

              {currentMission && (
                <div className="space-y-4 pb-6">
                  <div className="text-xs font-semibold text-muted-foreground flex items-center gap-2">
                    <MapPin className="h-3.5 w-3.5" /> 航线编辑
                  </div>
                  <TrajectoryEditor
                    mission={currentMission}
                    onSaveAction={handleTrajectorySave}
                    onPointsChangeAction={handleTrajectoryPointsChange}
                  />
                </div>
              )}
            </div>
          </ScrollArea>
        </aside>

        <main className="flex-1 relative bg-muted/30">
          <PCDCanvas
            ref={canvasRef}
            source={currentMission?.scene}
            plannedPathPoints={plannedPoints}
            plannedPathVisible
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
