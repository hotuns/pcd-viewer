import { MissionPhase, HangarChargeTelemetry, BatteryTelemetry, MissionRuntimeEvent } from "@/types/mission";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { BatteryCharging, Home, Upload, PlaneLanding, RefreshCw, Warehouse, PowerOff } from "lucide-react";

const phaseLabelMap: Record<MissionPhase, string> = {
  idle: "未执行",
  hangar_ready: "机库就绪",
  mission_upload: "上传任务",
  takeoff_ready: "起飞前检查",
  takeoff: "起飞中",
  executing: "任务执行",
  returning: "返航",
  landing: "降落",
  charging: "充电中",
  error: "异常",
};

const hangarStatusMap: Record<number, { text: string; tone: string }> = {
  0: { text: "待命", tone: "text-muted-foreground" },
  1: { text: "充电中", tone: "text-amber-500" },
  2: { text: "充电完成", tone: "text-green-500" },
  3: { text: "充电错误", tone: "text-red-500" },
  4: { text: "设备离线", tone: "text-muted-foreground" },
};

interface MissionRuntimePanelProps {
  phase: MissionPhase;
  rosConnected: boolean;
  hangar?: HangarChargeTelemetry | null;
  battery?: BatteryTelemetry | null;
  progress?: { completed: number; total: number } | null;
  lastWaypoint?: { index: number; info?: string; position?: { x: number; y: number; z: number } } | null;
  events: MissionRuntimeEvent[];
  busyAction?: string | null;
  missionReady: boolean;
  pendingCount?: number;
  canResume?: boolean;
  runtimeError?: string | null;
  onOpenHangar: () => Promise<void> | void;
  onCloseHangar: () => Promise<void> | void;
  onUploadMission: () => Promise<void> | void;
  onResumeMission?: () => Promise<void> | void;
  onExecuteMission: () => Promise<void> | void;
  onTakeoff?: () => Promise<void> | void;
  onReturnHome: () => Promise<void> | void;
  onLand?: () => Promise<void> | void;
  onArmOff: () => Promise<void> | void;
}

export function MissionRuntimePanel({
  phase,
  rosConnected,
  hangar,
  battery,
  progress,
  lastWaypoint,
  events,
  busyAction,
  missionReady,
  pendingCount = 0,
  canResume = false,
  runtimeError,
  onOpenHangar,
  onCloseHangar,
  onUploadMission,
  onResumeMission,
  onExecuteMission,
  onTakeoff,
  onReturnHome,
  onLand,
  onArmOff,
}: MissionRuntimePanelProps) {
  const actionDisabled = !rosConnected || !!busyAction;
  const batteryPct = battery?.percentage ?? null;
  const progressPct = progress && progress.total > 0 ? (progress.completed / progress.total) * 100 : 0;
  const handleStartMission = async () => {
    if (actionDisabled) return;
    try {
      if (onTakeoff) {
        await onTakeoff();
      }
    } catch (err) {
      console.warn("Takeoff command failed", err);
    }
    await onExecuteMission();
  };

  const handleReturnAndLand = async () => {
    if (actionDisabled) return;
    try {
      await onReturnHome();
    } catch (err) {
      console.warn("Return home command failed", err);
    }
    try {
      await onLand?.();
    } catch (err) {
      console.warn("Land command failed", err);
    }
  };

  return (
    <div className="space-y-5">
      <div className="flex items-center justify-between">
        <div>
          <div className="text-xs text-muted-foreground">Missionlogic 状态</div>
          <div className="text-lg font-semibold flex items-center gap-2">
            {phaseLabelMap[phase]}
            <Badge variant={phase === "error" ? "destructive" : "outline"} className="text-[10px]">
              {rosConnected ? "ROS 已连接" : "ROS 未连接"}
            </Badge>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-2 gap-3">
        <div className="rounded-xl border p-3 space-y-2">
          <div className="flex items-center justify-between text-xs text-muted-foreground">
            <span className="flex items-center gap-1"><Warehouse className="h-3.5 w-3.5" />机库</span>
            <span className={hangarStatusMap[hangar?.status ?? 0]?.tone}>{hangarStatusMap[hangar?.status ?? 0]?.text}</span>
          </div>
          <div className="text-sm text-muted-foreground">
            充电: {hangar?.percentage ?? 0}% · 功率 {hangar?.power?.toFixed?.(1) ?? "0"}W
          </div>
        </div>
        <div className="rounded-xl border p-3 space-y-2">
          <div className="flex items-center justify-between text-xs text-muted-foreground">
            <span className="flex items-center gap-1"><BatteryCharging className="h-3.5 w-3.5" />电池</span>
            {batteryPct != null && (
              <span className={batteryPct < 25 ? "text-red-500" : "text-foreground"}>{batteryPct.toFixed(1)}%</span>
            )}
          </div>
          <div className="text-sm text-muted-foreground">
            电压 {battery?.voltage?.toFixed?.(2) ?? "--"} V
          </div>
        </div>
      </div>

      {progress && (
        <div className="space-y-2">
          <div className="flex items-center justify-between text-xs text-muted-foreground">
            <span>任务进度</span>
            <span>{progress.completed}/{progress.total}</span>
          </div>
          <div className="h-2 rounded-full bg-muted overflow-hidden">
            <div className="h-full bg-primary transition-all" style={{ width: `${progressPct}%` }} />
          </div>
          {lastWaypoint && (
            <div className="text-[11px] text-muted-foreground">
              最近航点 #{lastWaypoint.index + 1} {lastWaypoint.info?.trim()}
            </div>
          )}
        </div>
      )}

      {pendingCount > 0 && (
        <div className="text-[11px] text-muted-foreground">
          剩余 {pendingCount} 个航点待执行
        </div>
      )}

      {canResume && onResumeMission && (
        <Button
          size="sm"
          className="w-full h-9 text-xs gap-1.5 bg-emerald-600 hover:bg-emerald-500 text-white"
          onClick={onResumeMission}
          disabled={!!busyAction}
        >
          <RefreshCw className="h-4 w-4 mr-2" />
          继续任务（{pendingCount} 点）
        </Button>
      )}

      <div className="grid grid-cols-2 gap-2">
        <Button size="sm" variant="secondary" onClick={onOpenHangar} disabled title="待实现" className="justify-start">
          <Home className="h-4 w-4 mr-2" /> 打开机库
        </Button>
        <Button size="sm" variant="secondary" onClick={onCloseHangar} disabled title="待实现" className="justify-start">
          <Home className="h-4 w-4 mr-2" /> 关闭机库
        </Button>
        <Button size="sm" onClick={onUploadMission} disabled={actionDisabled || !missionReady} className="justify-start">
          <Upload className="h-4 w-4 mr-2" /> 上传任务
        </Button>
        <Button size="sm" variant="outline" onClick={handleStartMission} disabled={actionDisabled} className="justify-start col-span-2">
          <RefreshCw className="h-4 w-4 mr-2" /> 起飞并执行
        </Button>
        <Button size="sm" variant="outline" onClick={handleReturnAndLand} disabled={actionDisabled} className="justify-start col-span-2">
          <PlaneLanding className="h-4 w-4 mr-2" /> 返航并降落
        </Button>
        <Button size="sm" variant="destructive" onClick={onArmOff} disabled={actionDisabled} className="justify-start">
          <PowerOff className="h-4 w-4 mr-2" /> ARM OFF
        </Button>
      </div>
      {!missionReady && (
        <div className="text-[11px] text-amber-500 bg-amber-500/10 rounded px-3 py-1">
          请先配置航线与 HomePos 后再上传任务
        </div>
      )}

      {runtimeError && (
        <div className="text-xs text-red-500 bg-red-500/10 rounded px-3 py-2">
          {runtimeError}
        </div>
      )}

      <Separator />
      <div className="space-y-2">
        <div className="text-xs font-semibold text-muted-foreground">事件日志</div>
        <div className="space-y-1 max-h-48 overflow-y-auto pr-2">
          {events.length === 0 && (
            <div className="text-[11px] text-muted-foreground">暂无事件</div>
          )}
          {events.map((event) => (
            <div key={event.id} className="text-[11px] flex items-start gap-2">
              <span className={`w-1.5 h-1.5 rounded-full mt-1 ${event.level === "error" ? "bg-red-500" : event.level === "warning" ? "bg-amber-500" : "bg-green-500"}`} />
              <div>
                <div className="font-medium text-foreground/90">{event.message}</div>
                <div className="text-muted-foreground">{event.timestamp.toLocaleTimeString()}</div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
