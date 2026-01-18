import { useState } from "react";
import { MissionPhase, HangarChargeTelemetry, BatteryTelemetry, MissionRuntimeEvent } from "@/types/mission";
import type { NestTelemetry } from "@/types/nest";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { BatteryCharging, Home, Upload, PlaneLanding, RefreshCw, Warehouse, PowerOff, AlertTriangle, Shield, Map } from "lucide-react";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { cn } from "@/lib/utils";

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
  0: { text: "待命", tone: "text-slate-400" },
  1: { text: "充电中", tone: "text-amber-400" },
  2: { text: "充电完成", tone: "text-green-400" },
  3: { text: "充电错误", tone: "text-red-400" },
  4: { text: "设备离线", tone: "text-slate-500" },
};

interface MissionRuntimePanelProps {
  phase: MissionPhase;
  rosConnected: boolean;
  hangar?: HangarChargeTelemetry | null;
  nestTelemetry?: NestTelemetry | null;
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
  onUploadEmergencyMission?: () => Promise<void> | void;
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
  nestTelemetry,
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
  onUploadEmergencyMission,
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
  const [controlTab, setControlTab] = useState<"mission" | "hangar">("mission");
  const formatOptMode = (mode?: string | null) => {
    if (mode === "1") return "手动模式";
    if (mode === "0") return "自动模式";
    return mode ?? "--";
  };
  const formatAutoMode = (mode?: string | null) => {
    if (mode === "1") return "自动离巢";
    if (mode === "0") return "自动进巢";
    return mode ?? "--";
  };
  const formatUavState = (state?: string | null) => {
    if (state === "0") return "无人机在巢";
    if (state === "1") return "无人机离巢";
    if (state === "2") return "异常：在巢";
    return state ?? "--";
  };
  const runAsyncAction = async (fn?: () => Promise<void> | void) => {
    if (actionDisabled || !fn) return;
    try {
      await fn();
    } catch (err) {
      console.warn("Mission runtime action failed", err);
    }
  };
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
    <div className="space-y-4 text-slate-100">
      <div className="flex items-center justify-between">
        <div>
          <div className="text-[11px] uppercase text-slate-400 tracking-wide">Missionlogic</div>
          <div className="text-base font-semibold flex items-center gap-2">
            {phaseLabelMap[phase]}
            <Badge variant={phase === "error" ? "destructive" : "outline"} className="text-[10px]">
              {rosConnected ? "ROS 已连接" : "ROS 未连接"}
            </Badge>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-2 gap-3">
        <div className="border border-slate-700 bg-slate-900 p-3 space-y-1 rounded">
          <div className="flex items-center justify-between text-[11px] text-slate-400">
            <span className="flex items-center gap-1"><Warehouse className="h-3.5 w-3.5" />机库</span>
            <span className={hangarStatusMap[hangar?.status ?? 0]?.tone}>{hangarStatusMap[hangar?.status ?? 0]?.text}</span>
          </div>
          <div className="text-xs text-slate-300">
            充电 {hangar?.percentage ?? 0}% · 功率 {hangar?.power?.toFixed?.(1) ?? "0"}W
          </div>
        </div>
        <div className="border border-slate-700 bg-slate-900 p-3 space-y-1 rounded">
          <div className="flex items-center justify-between text-[11px] text-slate-400">
            <span className="flex items-center gap-1"><BatteryCharging className="h-3.5 w-3.5" />电池</span>
            {batteryPct != null && (
              <span className={batteryPct < 25 ? "text-red-400" : "text-slate-200"}>{batteryPct.toFixed(1)}%</span>
            )}
          </div>
          <div className="text-xs text-slate-300">
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
        <Tooltip>
          <TooltipTrigger asChild>
            <Button
              size="icon"
              className="h-9 w-9 bg-emerald-600 hover:bg-emerald-500 text-white"
              onClick={onResumeMission}
              disabled={!!busyAction}
            >
              <RefreshCw className="h-4 w-4" />
            </Button>
          </TooltipTrigger>
          <TooltipContent>继续任务（{pendingCount} 点）</TooltipContent>
        </Tooltip>
      )}

      <div className="flex items-center text-xs font-medium rounded-full border border-slate-700 bg-slate-900/70 overflow-hidden">
        <button
          className={cn(
            "px-3 py-1.5 flex items-center gap-1 transition-colors",
            controlTab === "mission" ? "bg-primary/20 text-primary" : "text-slate-400 hover:text-slate-200"
          )}
          onClick={() => setControlTab("mission")}
        >
          <Map className="h-3.5 w-3.5" /> 任务控制
        </button>
        <button
          className={cn(
            "px-3 py-1.5 flex items-center gap-1 transition-colors border-l border-slate-700",
            controlTab === "hangar" ? "bg-primary/20 text-primary" : "text-slate-400 hover:text-slate-200"
          )}
          onClick={() => setControlTab("hangar")}
        >
          <Warehouse className="h-3.5 w-3.5" /> 机库控制
        </button>
      </div>

      {controlTab === "mission" ? (
        <div className="flex flex-wrap gap-2">
          <Tooltip>
            <TooltipTrigger asChild>
              <Button
                size="icon"
                onClick={() => { void runAsyncAction(onUploadMission); }}
                disabled={actionDisabled || !missionReady}
              >
                <Upload className="h-4 w-4" />
              </Button>
            </TooltipTrigger>
            <TooltipContent>上传任务</TooltipContent>
          </Tooltip>
          <Tooltip>
            <TooltipTrigger asChild>
              <Button
                size="icon"
                variant="outline"
                onClick={handleStartMission}
                disabled={actionDisabled}
              >
                <RefreshCw className="h-4 w-4" />
              </Button>
            </TooltipTrigger>
            <TooltipContent>起飞并执行</TooltipContent>
          </Tooltip>
          <Tooltip>
            <TooltipTrigger asChild>
              <Button
                size="icon"
                variant="destructive"
                onClick={() => { void runAsyncAction(onUploadEmergencyMission); }}
                disabled={actionDisabled || !onUploadEmergencyMission}
              >
                <Shield className="h-4 w-4" />
              </Button>
            </TooltipTrigger>
            <TooltipContent>迫降（立即停止并降落）</TooltipContent>
          </Tooltip>
          <Tooltip>
            <TooltipTrigger asChild>
              <Button
                size="icon"
                variant="outline"
                onClick={handleReturnAndLand}
                disabled={actionDisabled}
              >
                <PlaneLanding className="h-4 w-4" />
              </Button>
            </TooltipTrigger>
            <TooltipContent>返航并降落</TooltipContent>
          </Tooltip>
          {!missionReady && (
            <div className="text-[11px] text-amber-500 bg-amber-500/10 rounded px-3 py-1 flex items-center gap-1">
              <AlertTriangle className="h-3.5 w-3.5" /> 请先配置航线与迫降点
            </div>
          )}
        </div>
      ) : (
        <div className="space-y-2">
          <div className="text-[11px] text-slate-500 space-y-1">
            <div>状态: <span className={hangarStatusMap[hangar?.status ?? 0]?.tone}>{hangarStatusMap[hangar?.status ?? 0]?.text}</span></div>
            <div>充电 {hangar?.percentage ?? 0}% · 功率 {hangar?.power?.toFixed?.(1) ?? "0"} W</div>
            <div>时长 {hangar?.duration?.toFixed?.(0) ?? "--"} s</div>
            {hangar?.error && (
              <div className="text-destructive flex items-center gap-1">
                <AlertTriangle className="h-3.5 w-3.5" /> {hangar.error}
              </div>
            )}
          </div>
          {nestTelemetry && (
            <div className="text-[11px] text-slate-400 space-y-2 border border-slate-800 rounded p-2 bg-slate-900/40">
              <div className="grid grid-cols-2 gap-x-4 gap-y-1">
                <div>机库模式: {formatOptMode(nestTelemetry.status?.optMode)}</div>
                <div>自动模式: {formatAutoMode(nestTelemetry.autoMode?.mode)}</div>
                <div>内部温度: {nestTelemetry.status?.temperature != null ? `${nestTelemetry.status.temperature.toFixed(1)} ℃` : "--"}</div>
                <div>内部湿度: {nestTelemetry.status?.humidity != null ? `${nestTelemetry.status.humidity.toFixed(0)} %` : "--"}</div>
                <div>无人机状态: {formatUavState(nestTelemetry.uavPresence?.state)}</div>
                <div>
                  均衡:
                  {nestTelemetry.battery?.batteryBalanceState
                    ? nestTelemetry.battery.batteryBalanceState === "1"
                      ? " 开启"
                      : " 关闭"
                    : " --"}
                </div>
              </div>
              <div className="grid grid-cols-2 gap-x-4 gap-y-1">
                <div>SOC: {nestTelemetry.battery?.soc != null ? `${nestTelemetry.battery.soc.toFixed(1)}%` : "--"}</div>
                <div>电压: {nestTelemetry.battery?.chargingVoltage != null ? `${nestTelemetry.battery.chargingVoltage.toFixed(2)} V` : "--"}</div>
                <div>电流: {nestTelemetry.battery?.chargingCurrent != null ? `${nestTelemetry.battery.chargingCurrent.toFixed(2)} A` : "--"}</div>
                <div>供电:
                  {nestTelemetry.battery?.batterySupplyState
                    ? nestTelemetry.battery.batterySupplyState === "1"
                      ? " 已供电"
                      : " 未供电"
                    : " --"}
                </div>
                <div>触点:
                  {nestTelemetry.battery?.chargingOk
                    ? nestTelemetry.battery.chargingOk === "1"
                      ? " 正常"
                      : " 异常"
                    : " --"}
                </div>
                <div>板卡温度: {nestTelemetry.battery?.pcbTemp1 != null ? `${nestTelemetry.battery.pcbTemp1.toFixed(1)} ℃` : "--"}</div>
                <div>电池温度1: {nestTelemetry.battery?.batteryTemp1 != null ? `${nestTelemetry.battery.batteryTemp1.toFixed(1)} ℃` : "--"}</div>
                <div>电池温度2: {nestTelemetry.battery?.batteryTemp2 != null ? `${nestTelemetry.battery.batteryTemp2.toFixed(1)} ℃` : "--"}</div>
              </div>
            </div>
          )}
          <div className="flex flex-wrap gap-2">
            <Tooltip>
              <TooltipTrigger asChild>
                <Button size="icon" variant="secondary" onClick={() => { void runAsyncAction(onOpenHangar); }} disabled={actionDisabled}>
                  <Home className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>打开机库门</TooltipContent>
            </Tooltip>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button size="icon" variant="secondary" onClick={() => { void runAsyncAction(onCloseHangar); }} disabled={actionDisabled}>
                  <Home className="h-4 w-4 rotate-180" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>关闭机库门</TooltipContent>
            </Tooltip>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button size="icon" variant="destructive" onClick={() => { void runAsyncAction(onArmOff); }} disabled={actionDisabled}>
                  <PowerOff className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>ARM OFF</TooltipContent>
            </Tooltip>
          </div>
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
