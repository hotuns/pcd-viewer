"use client";

import { useEffect, useRef, useState, useCallback } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { 
  ArrowUp, 
  ArrowDown, 
  Trash2, 
  Save, 
  Download, 
  Upload,
  MapPin,
  Clock,
  Plus
} from "lucide-react";
import type { Mission, PlannedPoint } from "@/types/mission";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { TASK_TYPE_OPTIONS, normalizeTaskType } from "@/lib/taskTypes";

type Waypoint = PlannedPoint;

export default function TrajectoryEditor({
  mission,
  onSaveAction,
  onPointsChangeAction,
  externalPoints,
  editable = true,
  selectedIndex,
  onSelectIndex,
  lockAnchors = false,
}: {
  mission: Mission;
  onSaveAction: (file: File, jsonText: string) => void | Promise<void>;
  onPointsChangeAction?: (points: Waypoint[]) => void;
  externalPoints?: Waypoint[];
  editable?: boolean;
  selectedIndex?: number | null;
  onSelectIndex?: (index: number | null) => void;
  lockAnchors?: boolean;
}) {
  const [loading, setLoading] = useState(false);
  const [meta, setMeta] = useState<{ name?: string; coord?: string; note?: string }>({});
  const [points, setPoints] = useState<Waypoint[]>([]);
  const [manualInput, setManualInput] = useState<{ x: string; y: string; z: string; w: string; task_type: string }>({ x: "", y: "", z: "", w: "", task_type: "0" });
  const [saveStatus, setSaveStatus] = useState<"idle" | "saving" | "success" | "error">("idle");
  const [lastSavedAt, setLastSavedAt] = useState<Date | null>(null);
  const hasTrajectory = !!mission.trajectory;
  const fileInputRef = useRef<HTMLInputElement | null>(null);
  const rowRefs = useRef<Array<HTMLDivElement | null>>([]);

  const parseTrajectoryJson = (text: string) => {
    const parsed = JSON.parse(text);
    const nextMeta = parsed.meta ?? {};
    const pts = Array.isArray(parsed.points) ? parsed.points : [];
    const normalized: Waypoint[] = pts.map((p: unknown) => {
      const obj = (typeof p === "object" && p !== null ? p as Record<string, unknown> : {});
      const num = (v: unknown) => {
        const n = typeof v === "string" ? Number(v) : (typeof v === "number" ? v : NaN);
        return Number.isFinite(n) ? n : 0;
      };
      const tRaw = obj["t"];
      const str = (value: unknown) => (typeof value === "string" ? value : undefined);
      return {
        x: num(obj["x"]),
        y: num(obj["y"]),
        z: num(obj["z"]),
        w: obj["w"] == null ? undefined : num(obj["w"]),
        t: tRaw == null ? undefined : num(tRaw),
        task_type: normalizeTaskType(obj["task_type"]),
        info: str(obj["info"]),
      };
    });
    return { meta: nextMeta, points: normalized };
  };

  useEffect(() => {
    let cancelled = false;
    async function load() {
      if (!mission.trajectory) {
        setMeta({});
        setPoints([]);
        return;
      }
      setLoading(true);
      try {
        let text = "";
        if (mission.trajectory.type === "file") {
          text = await mission.trajectory.file.text();
        } else {
          const res = await fetch(mission.trajectory.url);
          text = await res.text();
        }
        if (cancelled) return;
        const { meta: nextMeta, points: normalized } = parseTrajectoryJson(text);
        setMeta(nextMeta);
        setPoints(normalized);
        onPointsChangeAction?.(normalized);
      } catch (e) {
        console.error("Failed to parse trajectory:", e);
        setMeta({});
        setPoints([]);
      } finally {
        setLoading(false);
      }
    }
    load();
    return () => { cancelled = true; };
  }, [mission.trajectory, onPointsChangeAction]);

  const isEditable = editable !== false;

  const makePoint = (reference?: Waypoint, overrides?: Partial<Waypoint>): Waypoint => ({
    x: overrides?.x ?? reference?.x ?? 0,
    y: overrides?.y ?? reference?.y ?? 0,
    z: overrides?.z ?? reference?.z ?? 0,
    w: overrides?.w ?? reference?.w ?? 0,
    t: overrides?.t ?? (reference?.t ?? 0),
    task_type: overrides?.task_type ?? reference?.task_type ?? 0,
    info: overrides?.info ?? reference?.info,
  });

  const insertPoint = (insertIndex: number, reference?: Waypoint, overrides?: Partial<Waypoint>) => {
    if (!isEditable) return;
    const nextPoint = makePoint(reference, overrides);
    const next = points.slice();
    next.splice(insertIndex, 0, nextPoint);
    setPoints(next);
    onPointsChangeAction?.(next);
  };

  const addPoint = () => {
    const insertIndex = lockAnchors ? Math.max(points.length - 1, 1) : points.length;
    const reference = points[Math.max(insertIndex - 1, 0)];
    insertPoint(insertIndex, reference);
  };
  const addManualPoint = () => {
    if (!isEditable) return;
    const x = Number(manualInput.x);
    const y = Number(manualInput.y);
    const z = Number(manualInput.z);
    const wDeg = Number(manualInput.w);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || !Number.isFinite(wDeg)) {
      alert("请输入有效的数字坐标");
      return;
    }
    const taskTypeValue = normalizeTaskType(manualInput.task_type) ?? 0;
    const insertIndex = lockAnchors ? Math.max(points.length - 1, 1) : points.length;
    insertPoint(insertIndex, points[Math.max(insertIndex - 1, 0)], {
      x,
      y,
      z,
      w: (wDeg * Math.PI) / 180,
      t: (points.at(insertIndex - 1)?.t ?? points.at(-1)?.t ?? -1) + 1,
      task_type: taskTypeValue,
    });
    setManualInput({ x: "", y: "", z: "", w: "", task_type: taskTypeValue.toString() });
  };
  const removePoint = (idx: number) => {
    if (!isEditable) return;
    if (lockAnchors && (idx === 0 || idx === points.length - 1)) return;
    const next = points.filter((_, i) => i !== idx);
    setPoints(next);
    onPointsChangeAction?.(next);
  };
  const move = (idx: number, dir: -1 | 1) => {
    if (!isEditable) return;
    const j = idx + dir;
    if (j < 0 || j >= points.length) return;
    if (lockAnchors && (idx === 0 || idx === points.length - 1 || j === 0 || j === points.length - 1)) return;
    const arr = points.slice();
    [arr[idx], arr[j]] = [arr[j], arr[idx]];
    setPoints(arr);
    onPointsChangeAction?.(arr);
  };

  const updatePoint = useCallback((index: number, patch: Partial<Waypoint>) => {
    if (!isEditable) return;
    setPoints((prev) => {
      const next = prev.map((pt, i) => (i === index ? { ...pt, ...patch } : pt));
      onPointsChangeAction?.(next);
      return next;
    });
  }, [isEditable, onPointsChangeAction]);

  const insertAfter = (idx: number) => {
    if (!isEditable) return;
    const targetIndex = Math.min(idx + 1, lockAnchors ? Math.max(points.length - 1, 1) : points.length);
    const reference = points[idx];
    insertPoint(targetIndex, reference);
  };

  const save = async () => {
    if (!isEditable || (!hasTrajectory && points.length === 0)) return;
    setSaveStatus("saving");
    const json = JSON.stringify({ meta: { name: meta.name ?? mission.name, coord: meta.coord ?? "local-xyz", note: meta.note ?? "" }, points }, null, 2);
    const file = new File([json], `trajectory-${Date.now()}.json`, { type: "application/json" });
    try {
      await Promise.resolve(onSaveAction(file, json));
      setSaveStatus("success");
      setLastSavedAt(new Date());
      setTimeout(() => setSaveStatus("idle"), 2000);
    } catch (error) {
      console.error("Failed to save trajectory", error);
      setSaveStatus("error");
    }
  };

  const handleImport = async (file: File) => {
    if (!isEditable) return;
    try {
      const text = await file.text();
      const { meta: nextMeta, points: nextPoints } = parseTrajectoryJson(text);
      setMeta(nextMeta);
      setPoints(nextPoints);
      onPointsChangeAction?.(nextPoints);
    } catch (error) {
      console.error("Failed to import trajectory:", error);
      alert("无法解析该航线文件，请确认 JSON 结构包含 meta 与 points 数组。");
    }
  };

  // 外部 3D 拖拽修改 -> 同步进编辑器
  useEffect(() => {
    if (!externalPoints) return;
    // 合并 z/t：z 直接来自外部，t 保留原有
    setPoints((prev) => {
      const merged: Waypoint[] = externalPoints.map((p, i) => ({
        x: p.x,
        y: p.y,
        z: p.z,
        w: p.w ?? prev[i]?.w,
        t: p.t ?? prev[i]?.t,
        task_type: p.task_type ?? prev[i]?.task_type ?? 0,
        info: p.info ?? prev[i]?.info,
      }));
      return merged;
    });
  }, [externalPoints]);

  useEffect(() => {
    if (selectedIndex == null) return;
    const target = rowRefs.current[selectedIndex];
    if (target) {
      target.scrollIntoView({ block: "center", behavior: "smooth" });
    }
  }, [selectedIndex]);

  return (
    <div className="space-y-4 p-2 h-full flex flex-col">
      <div className="flex items-center justify-between shrink-0">
        <div className="flex items-center gap-2">
          <div className="p-2 bg-primary/10 rounded-lg">
            <MapPin className="w-4 h-4 text-primary" />
          </div>
          <div>
            <h3 className="font-medium leading-none">航线编辑</h3>
            <p className="text-xs text-muted-foreground mt-1">
              {loading ? "加载中..." : `${points.length} 个航点`}
            </p>
          </div>
        </div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-1 text-slate-400">
            <Button
              variant="ghost"
              size="icon"
              className="h-7 w-7"
              onClick={() => isEditable && fileInputRef.current?.click()}
              disabled={!isEditable}
              title="导入 JSON"
            >
              <Upload className="w-3.5 h-3.5" />
            </Button>
            <Button
              variant="ghost"
              size="icon"
              className="h-7 w-7"
              onClick={() => {
                const json = JSON.stringify({ meta: { name: meta.name ?? mission.name, coord: meta.coord ?? "local-xyz", note: meta.note ?? "" }, points }, null, 2);
                const blob = new Blob([json], { type: "application/json" });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url; a.download = `${mission.name}-trajectory.json`;
                a.click(); URL.revokeObjectURL(url);
              }}
              title="导出 JSON"
            >
              <Download className="w-3.5 h-3.5" />
            </Button>
            <Button
              size="icon"
              className="h-7 w-7"
              onClick={() => { void save(); }}
              disabled={!isEditable || (!hasTrajectory && points.length === 0)}
              title="保存更改"
            >
              <Save className="w-3.5 h-3.5" />
            </Button>
          </div>
          <div className="text-[11px] text-muted-foreground min-w-[90px] text-right">
            {saveStatus === "saving" && "保存中..."}
            {saveStatus === "success" && lastSavedAt && `已保存 ${lastSavedAt.toLocaleTimeString()}`}
            {saveStatus === "error" && <span className="text-red-400">保存失败</span>}
          </div>
        </div>
      </div>

      <div className="rounded-lg border border-dashed border-muted/60 p-3 text-xs space-y-2">
        <div className="text-xs font-semibold text-muted-foreground">手动输入航点</div>
        <div className="grid grid-cols-5 gap-2">
          {(["x", "y", "z"] as const).map((axis) => (
            <Input
              key={axis}
              type="text"
              inputMode="decimal"
              placeholder={axis.toUpperCase()}
              value={manualInput[axis]}
              onChange={(e) => setManualInput((prev) => ({ ...prev, [axis]: e.target.value }))}
              className="h-7 text-xs bg-slate-900 border-slate-700 text-slate-100"
              disabled={!isEditable}
            />
          ))}
          <Input
            type="text"
            inputMode="decimal"
            placeholder="Yaw (°)"
            value={manualInput.w}
            onChange={(e) => setManualInput((prev) => ({ ...prev, w: e.target.value }))}
            className="h-7 text-xs bg-slate-900 border-slate-700 text-slate-100"
            disabled={!isEditable}
          />
          <Select
            value={manualInput.task_type}
            onValueChange={(value) => setManualInput((prev) => ({ ...prev, task_type: value }))}
            disabled={!isEditable}
          >
            <SelectTrigger className="h-7 text-xs w-full bg-slate-900 border-slate-700 text-slate-100">
              <SelectValue placeholder="任务类型" />
            </SelectTrigger>
            <SelectContent>
              {TASK_TYPE_OPTIONS.map((option) => (
                <SelectItem key={option.value} value={option.value.toString()}>
                  {option.label}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
        <Button
          size="sm"
          className="h-7 text-xs px-3"
          onClick={addManualPoint}
          disabled={!isEditable}
        >
          添加
        </Button>
      </div>

      {!isEditable && (
        <div className="text-[11px] text-amber-400 bg-amber-500/10 rounded-md px-3 py-2 border border-amber-500/30">
          当前任务处于非规划阶段，请在上方切换到“编辑/规划”后调整航线。
        </div>
      )}

      <div className="grid grid-cols-2 gap-3 shrink-0 p-3 bg-muted/30 rounded-lg border border-border/50">
        <div className="space-y-0.5">
          <Label className="text-xs text-muted-foreground">名称</Label>
          <Input 
            className="h-6 text-[11px] bg-slate-900 border-slate-700 text-slate-100" 
            value={meta.name ?? ""} 
            onChange={(e) => setMeta({ ...meta, name: e.target.value })} 
            placeholder="航线名称"
            disabled={!isEditable}
          />
        </div>
        <div className="space-y-0.5">
          <Label className="text-xs text-muted-foreground">坐标系</Label>
          <Input 
            className="h-6 text-[11px] bg-slate-900 border-slate-700 text-slate-100" 
            value={meta.coord ?? "local-xyz"} 
            onChange={(e) => setMeta({ ...meta, coord: e.target.value })} 
            placeholder="local-xyz"
            disabled={!isEditable}
          />
        </div>
        <div className="col-span-2 space-y-1">
          <Label className="text-xs text-muted-foreground">备注</Label>
          <Input 
            className="h-6 text-[11px] bg-slate-900 border-slate-700 text-slate-100" 
            value={meta.note ?? ""} 
            onChange={(e) => setMeta({ ...meta, note: e.target.value })} 
            placeholder="添加备注信息..."
            disabled={!isEditable}
          />
        </div>
      </div>

      <div className="flex-1 border border-slate-800 rounded-md bg-slate-900/50 flex flex-col min-h-[200px]">
        <div className="grid grid-cols-[32px_repeat(5,minmax(0,1fr))_88px] gap-1.5 p-1.5 text-xs font-medium text-muted-foreground border-b bg-muted/50">
          <div className="text-center">#</div>
          <div className="text-center">X</div>
          <div className="text-center">Y</div>
          <div className="text-center">Z</div>
          <div className="text-center">偏航 (°)</div>
          <div className="text-center">任务类型</div>
          <div className="text-right pr-2">操作</div>
        </div>
        <div className="flex-1">
          <div className="divide-y divide-border/50">
            {points.map((p, i) => {
              const anchorLocked = lockAnchors && (i === 0 || i === points.length - 1);
              return (
                <div
                  key={i}
                  ref={(el) => {
                    rowRefs.current[i] = el;
                  }}
                  className={`grid grid-cols-[32px_repeat(5,minmax(0,1fr))_88px] gap-1.5 p-1.5 items-center transition-colors group cursor-pointer ${selectedIndex === i ? "bg-primary/10 border border-primary/30 rounded-md" : "hover:bg-muted/30"}`}
                  onClick={() => onSelectIndex?.(i)}
                >
                  <div className="text-[11px] text-muted-foreground text-center font-mono">{i + 1}</div>
                  {(["x", "y", "z"] as const).map((axis) => (
                    <Input
                      key={`${axis}-${i}`}
                      className="h-6 text-[11px] bg-slate-900 border-slate-700 text-slate-100"
                      type="text"
                      inputMode="decimal"
                      value={p[axis] ?? 0}
                      onChange={(e) => {
                        const value = Number(e.target.value);
                        if (!Number.isFinite(value)) return;
                        updatePoint(i, { [axis]: value } as Partial<Waypoint>);
                      }}
                      disabled={!isEditable || anchorLocked}
                      onPointerDown={(e) => e.stopPropagation()}
                      onClick={(e) => e.stopPropagation()}
                    />
                  ))}
                  <Input
                    className="h-6 text-[11px] bg-slate-900 border-slate-700 text-slate-100"
                    type="text"
                    inputMode="decimal"
                    value={((p.w ?? 0) * 180) / Math.PI}
                    onChange={(e) => {
                      const value = Number(e.target.value);
                      if (!Number.isFinite(value)) return;
                      updatePoint(i, { w: (value * Math.PI) / 180 });
                    }}
                    disabled={!isEditable || anchorLocked}
                    onPointerDown={(e) => e.stopPropagation()}
                    onClick={(e) => e.stopPropagation()}
                  />
                  <Select
                    value={(p.task_type ?? 0).toString()}
                    onValueChange={(value) => {
                      const parsed = normalizeTaskType(value) ?? 0;
                      updatePoint(i, { task_type: parsed } as Partial<Waypoint>);
                    }}
                    disabled={!isEditable || anchorLocked}
                  >
                    <SelectTrigger className="h-7 text-xs w-full bg-slate-900 border-slate-700 text-slate-100" onPointerDown={(e) => e.stopPropagation()}>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {TASK_TYPE_OPTIONS.map((option) => (
                        <SelectItem key={option.value} value={option.value.toString()}>
                          {option.label}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                  <div className="flex items-center justify-end gap-1 opacity-60 group-hover:opacity-100 transition-opacity">
                    <Button
                      size="icon"
                      variant="ghost"
                      className="h-5 w-5"
                      onClick={() => insertAfter(i)}
                      disabled={!isEditable || (lockAnchors && i >= points.length - 1)}
                      title="在下方插入航点"
                    >
                      <Plus className="w-3 h-3" />
                    </Button>
                    <Button
                      size="icon"
                      variant="ghost"
                      className="h-5 w-5"
                      onClick={() => move(i, -1)}
                      disabled={i === 0 || !isEditable || anchorLocked}
                      title="上移"
                    >
                      <ArrowUp className="w-3 h-3" />
                    </Button>
                    <Button
                      size="icon"
                      variant="ghost"
                      className="h-5 w-5"
                      onClick={() => move(i, 1)}
                      disabled={i === points.length - 1 || !isEditable || anchorLocked}
                      title="下移"
                    >
                      <ArrowDown className="w-3 h-3" />
                    </Button>
                    <Button
                      size="icon"
                      variant="ghost"
                      className="h-6 w-6 text-destructive hover:text-destructive hover:bg-destructive/10"
                      onClick={() => removePoint(i)}
                      disabled={!isEditable || anchorLocked}
                      title="删除"
                    >
                      <Trash2 className="w-3 h-3" />
                    </Button>
                  </div>
                  {(p.t != null || anchorLocked) && (
                    <div className="col-span-5 col-start-2 flex items-center gap-3 text-[10px] text-muted-foreground">
                      {p.t != null && (
                        <span className="flex items-center gap-1">
                          <Clock className="w-3 h-3" />
                          t={p.t}
                        </span>
                      )}
                      {anchorLocked && <span className="uppercase tracking-wide text-primary">已锁定 HomePos</span>}
                    </div>
                  )}
                </div>
              );
            })}
            {points.length === 0 && (
              <div className="flex flex-col items-center justify-center py-8 text-muted-foreground gap-2">
                <MapPin className="w-8 h-8 opacity-20" />
                <p className="text-xs">暂无航点数据</p>
                <Button variant="outline" size="sm" onClick={addPoint} className="mt-2 h-7 text-xs" disabled={!isEditable}>
                  添加第一个点
                </Button>
              </div>
            )}
          </div>
        </div>
      </div>

      <input
        ref={fileInputRef}
        type="file"
        accept="application/json,.json"
        className="hidden"
        onChange={(event) => {
          const file = event.target.files?.[0];
          if (file) {
            void handleImport(file);
            event.target.value = "";
          }
        }}
      />
    </div>
  );
}
