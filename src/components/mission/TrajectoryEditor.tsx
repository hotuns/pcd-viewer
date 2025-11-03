"use client";

import { useEffect, useMemo, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import type { Mission, Source } from "@/types/mission";

type Waypoint = { x: number; y: number; z: number; t?: number };

export default function TrajectoryEditor({
  mission,
  onSaveAction,
  onPointsChangeAction,
  externalPoints,
}: {
  mission: Mission;
  onSaveAction: (file: File, jsonText: string) => void;
  onPointsChangeAction?: (points: Waypoint[]) => void;
  externalPoints?: Array<{ x: number; y: number; z: number }>;
}) {
  const [loading, setLoading] = useState(false);
  const [meta, setMeta] = useState<{ name?: string; coord?: string; note?: string }>({});
  const [points, setPoints] = useState<Waypoint[]>([]);
  const hasTrajectory = !!mission.trajectory;

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
        const parsed = JSON.parse(text);
        setMeta(parsed.meta ?? {});
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
  }, [mission.trajectory]);

  const addPoint = () => {
    const next = [...points, { x: 0, y: 0, z: 0, t: (points.at(-1)?.t ?? -1) + 1 }];
    setPoints(next);
    onPointsChangeAction?.(next);
  };
  const removePoint = (idx: number) => {
    const next = points.filter((_, i) => i !== idx);
    setPoints(next);
    onPointsChangeAction?.(next);
  };
  const move = (idx: number, dir: -1 | 1) => {
    const j = idx + dir;
    if (j < 0 || j >= points.length) return;
    const arr = points.slice();
    [arr[idx], arr[j]] = [arr[j], arr[idx]];
    setPoints(arr);
    onPointsChangeAction?.(arr);
  };
  const updateField = (idx: number, key: keyof Waypoint, value: string) => {
    const v = Number(value);
    const next = points.map((wp, i) => i === idx ? { ...wp, [key]: isNaN(v) ? 0 : v } : wp);
    setPoints(next);
    onPointsChangeAction?.(next);
  };

  const save = () => {
    const json = JSON.stringify({ meta: { name: meta.name ?? mission.name, coord: meta.coord ?? "local-xyz", note: meta.note ?? "" }, points }, null, 2);
    const file = new File([json], `trajectory-${Date.now()}.json`, { type: "application/json" });
    onSaveAction(file, json);
  };

  // 外部 3D 拖拽修改 -> 同步进编辑器
  useEffect(() => {
    if (!externalPoints) return;
    // 合并 z/t：z 直接来自外部，t 保留原有
    setPoints((prev) => {
      const merged: Waypoint[] = externalPoints.map((p, i) => ({ x: p.x, y: p.y, z: p.z, t: prev[i]?.t }));
      return merged;
    });
  }, [externalPoints?.length, JSON.stringify(externalPoints)]);

  return (
    <div className="space-y-3">
      <div className="flex items-center justify-between">
        <Label>编辑航线</Label>
        <div className="text-xs text-muted-foreground">{loading ? "加载中..." : `${points.length} 点`}</div>
      </div>
      <div className="grid grid-cols-4 gap-2">
        <div>
          <Label className="text-xs">名称</Label>
          <Input value={meta.name ?? ""} onChange={(e) => setMeta({ ...meta, name: e.target.value })} />
        </div>
        <div>
          <Label className="text-xs">坐标系</Label>
          <Input value={meta.coord ?? "local-xyz"} onChange={(e) => setMeta({ ...meta, coord: e.target.value })} />
        </div>
        <div className="col-span-2">
          <Label className="text-xs">备注</Label>
          <Input value={meta.note ?? ""} onChange={(e) => setMeta({ ...meta, note: e.target.value })} />
        </div>
      </div>
      <Separator />
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <Label className="text-sm">点列表（在 3D 中拖拽修改）</Label>
          <Button size="sm" onClick={addPoint}>新增点</Button>
        </div>
        <div className="max-h-56 overflow-auto border border-border rounded">
          <div className="grid grid-cols-[40px_1fr_120px] gap-2 p-2 text-xs text-muted-foreground sticky top-0 bg-card/80 backdrop-blur"> 
            <div>#</div><div>位置 (X,Y,Z)</div><div>操作</div>
          </div>
          {points.map((p, i) => (
            <div key={i} className="grid grid-cols-[40px_1fr_120px] gap-2 p-2 items-center border-t">
              <div className="text-xs text-muted-foreground">{i + 1}</div>
              <div className="text-xs font-mono text-muted-foreground">[{p.x.toFixed(3)}, {p.y.toFixed(3)}, {p.z.toFixed(3)}]{p.t!=null?`  t=${p.t}`:""}</div>
              <div className="flex gap-1">
                <Button size="sm" variant="outline" onClick={() => move(i, -1)} disabled={i === 0}>上移</Button>
                <Button size="sm" variant="outline" onClick={() => move(i, 1)} disabled={i === points.length - 1}>下移</Button>
                <Button size="sm" variant="destructive" onClick={() => removePoint(i)}>删除</Button>
              </div>
            </div>
          ))}
          {points.length === 0 && (
            <div className="p-3 text-xs text-muted-foreground">暂无点，点击“新增点”开始编辑。</div>
          )}
        </div>
      </div>
      <div className="flex gap-2 justify-end">
        <Button variant="outline" size="sm" onClick={() => {
          // 导出下载
          const json = JSON.stringify({ meta: { name: meta.name ?? mission.name, coord: meta.coord ?? "local-xyz", note: meta.note ?? "" }, points }, null, 2);
          const blob = new Blob([json], { type: "application/json" });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url; a.download = `${mission.name}-trajectory.json`;
          a.click(); URL.revokeObjectURL(url);
        }}>下载JSON</Button>
        <Button size="sm" onClick={save} disabled={!hasTrajectory && points.length === 0}>保存到任务</Button>
      </div>
    </div>
  );
}
