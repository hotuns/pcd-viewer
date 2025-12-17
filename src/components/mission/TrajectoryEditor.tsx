"use client";

import { useEffect, useRef, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { 
  ArrowUp, 
  ArrowDown, 
  Trash2, 
  Plus, 
  Save, 
  Download, 
  Upload,
  MapPin,
  Clock
} from "lucide-react";
import type { Mission } from "@/types/mission";

type Waypoint = { x: number; y: number; z: number; t?: number };

export default function TrajectoryEditor({
  mission,
  onSaveAction,
  onPointsChangeAction,
  externalPoints,
  editable = true,
  selectedIndex,
  onSelectIndex,
}: {
  mission: Mission;
  onSaveAction: (file: File, jsonText: string) => void;
  onPointsChangeAction?: (points: Waypoint[]) => void;
  externalPoints?: Array<{ x: number; y: number; z: number }>;
  editable?: boolean;
  selectedIndex?: number | null;
  onSelectIndex?: (index: number | null) => void;
}) {
  const [loading, setLoading] = useState(false);
  const [meta, setMeta] = useState<{ name?: string; coord?: string; note?: string }>({});
  const [points, setPoints] = useState<Waypoint[]>([]);
  const hasTrajectory = !!mission.trajectory;
  const fileInputRef = useRef<HTMLInputElement | null>(null);

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
      return { x: num(obj["x"]), y: num(obj["y"]), z: num(obj["z"]), t: tRaw == null ? undefined : num(tRaw) };
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

  const addPoint = () => {
    if (!isEditable) return;
    const next = [...points, { x: 0, y: 0, z: 0, t: (points.at(-1)?.t ?? -1) + 1 }];
    setPoints(next);
    onPointsChangeAction?.(next);
  };
  const removePoint = (idx: number) => {
    if (!isEditable) return;
    const next = points.filter((_, i) => i !== idx);
    setPoints(next);
    onPointsChangeAction?.(next);
  };
  const move = (idx: number, dir: -1 | 1) => {
    if (!isEditable) return;
    const j = idx + dir;
    if (j < 0 || j >= points.length) return;
    const arr = points.slice();
    [arr[idx], arr[j]] = [arr[j], arr[idx]];
    setPoints(arr);
    onPointsChangeAction?.(arr);
  };

  const save = () => {
    if (!isEditable) return;
    const json = JSON.stringify({ meta: { name: meta.name ?? mission.name, coord: meta.coord ?? "local-xyz", note: meta.note ?? "" }, points }, null, 2);
    const file = new File([json], `trajectory-${Date.now()}.json`, { type: "application/json" });
    onSaveAction(file, json);
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
      const merged: Waypoint[] = externalPoints.map((p, i) => ({ x: p.x, y: p.y, z: p.z, t: prev[i]?.t }));
      return merged;
    });
  }, [externalPoints]);

  return (
    <div className="space-y-4 h-full flex flex-col">
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
        <Button size="sm" onClick={addPoint} className="h-8 gap-1" disabled={!isEditable}>
          <Plus className="w-3.5 h-3.5" />
          <span className="hidden sm:inline">新增点</span>
        </Button>
      </div>

      {!isEditable && (
        <div className="text-[11px] text-amber-600 bg-amber-100/80 rounded-md px-3 py-2 border border-amber-200">
          当前任务未进入“中”状态，点击「进入编辑」后即可编辑航线。
        </div>
      )}

      <div className="grid grid-cols-2 gap-3 shrink-0 p-3 bg-muted/30 rounded-lg border border-border/50">
        <div className="space-y-1">
          <Label className="text-xs text-muted-foreground">名称</Label>
          <Input 
            className="h-7 text-xs bg-background" 
            value={meta.name ?? ""} 
            onChange={(e) => setMeta({ ...meta, name: e.target.value })} 
            placeholder="航线名称"
            disabled={!isEditable}
          />
        </div>
        <div className="space-y-1">
          <Label className="text-xs text-muted-foreground">坐标系</Label>
          <Input 
            className="h-7 text-xs bg-background" 
            value={meta.coord ?? "local-xyz"} 
            onChange={(e) => setMeta({ ...meta, coord: e.target.value })} 
            placeholder="local-xyz"
            disabled={!isEditable}
          />
        </div>
        <div className="col-span-2 space-y-1">
          <Label className="text-xs text-muted-foreground">备注</Label>
          <Input 
            className="h-7 text-xs bg-background" 
            value={meta.note ?? ""} 
            onChange={(e) => setMeta({ ...meta, note: e.target.value })} 
            placeholder="添加备注信息..."
            disabled={!isEditable}
          />
        </div>
      </div>

      <div className="flex-1 min-h-0 border rounded-md bg-background flex flex-col">
        <div className="grid grid-cols-[40px_1fr_100px] gap-2 p-2 text-xs font-medium text-muted-foreground border-b bg-muted/50"> 
          <div className="text-center">#</div>
          <div>坐标 (X, Y, Z)</div>
          <div className="text-right pr-2">操作</div>
        </div>
        <ScrollArea className="flex-1">
          <div className="divide-y divide-border/50">
            {points.map((p, i) => (
              <div
                key={i}
                className={`grid grid-cols-[40px_1fr_100px] gap-2 p-2 items-center transition-colors group cursor-pointer ${selectedIndex === i ? "bg-primary/10 border border-primary/30 rounded-md" : "hover:bg-muted/30"}`}
                onClick={() => onSelectIndex?.(i)}
              >
                <div className="text-xs text-muted-foreground text-center font-mono">{i + 1}</div>
                <div className="space-y-1">
                  <div className="text-xs font-mono text-foreground">
                    [{p.x.toFixed(2)}, {p.y.toFixed(2)}, {p.z.toFixed(2)}]
                  </div>
                  {p.t != null && (
                    <div className="flex items-center gap-1 text-[10px] text-muted-foreground">
                      <Clock className="w-3 h-3" />
                      <span>t={p.t}</span>
                    </div>
                  )}
                </div>
                <div className="flex items-center justify-end gap-1 opacity-60 group-hover:opacity-100 transition-opacity">
                  <Button 
                    size="icon" 
                    variant="ghost" 
                    className="h-6 w-6" 
                    onClick={() => move(i, -1)} 
                    disabled={i === 0 || !isEditable}
                    title="上移"
                  >
                    <ArrowUp className="w-3 h-3" />
                  </Button>
                  <Button 
                    size="icon" 
                    variant="ghost" 
                    className="h-6 w-6" 
                    onClick={() => move(i, 1)} 
                    disabled={i === points.length - 1 || !isEditable}
                    title="下移"
                  >
                    <ArrowDown className="w-3 h-3" />
                  </Button>
                  <Button 
                    size="icon" 
                    variant="ghost" 
                    className="h-6 w-6 text-destructive hover:text-destructive hover:bg-destructive/10" 
                    onClick={() => removePoint(i)}
                    disabled={!isEditable}
                    title="删除"
                  >
                    <Trash2 className="w-3 h-3" />
                  </Button>
                </div>
              </div>
            ))}
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
        </ScrollArea>
      </div>

      <div className="flex gap-2 pt-2 shrink-0">
        <Button 
          variant="outline" 
          size="sm" 
          className="flex-1 h-8 text-xs gap-1.5"
          onClick={() => isEditable && fileInputRef.current?.click()}
          disabled={!isEditable}
        >
          <Upload className="w-3.5 h-3.5" />
          导入 JSON
        </Button>
        <Button 
          variant="outline" 
          size="sm" 
          className="flex-1 h-8 text-xs gap-1.5"
          onClick={() => {
            const json = JSON.stringify({ meta: { name: meta.name ?? mission.name, coord: meta.coord ?? "local-xyz", note: meta.note ?? "" }, points }, null, 2);
            const blob = new Blob([json], { type: "application/json" });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url; a.download = `${mission.name}-trajectory.json`;
            a.click(); URL.revokeObjectURL(url);
          }}
        >
          <Download className="w-3.5 h-3.5" />
          导出 JSON
        </Button>
        <Button 
          size="sm" 
          className="flex-1 h-8 text-xs gap-1.5"
          onClick={save} 
          disabled={!isEditable || (!hasTrajectory && points.length === 0)}
        >
          <Save className="w-3.5 h-3.5" />
          保存更改
        </Button>
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
