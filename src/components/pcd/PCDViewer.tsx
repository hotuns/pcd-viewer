"use client";

import { useCallback, useMemo, useRef, useState } from "react";
import { PCDCanvas, PCDCanvasHandle, Source } from "./PCDCanvas";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Separator } from "@/components/ui/separator";
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from "@/components/ui/tooltip";
import { RefreshCw, Maximize, Upload, Link as LinkIcon } from "lucide-react";
import { Select, SelectTrigger, SelectContent, SelectItem, SelectValue } from "@/components/ui/select";
// no annotations
import { useEffect } from "react";

export default function PCDViewer() {
  const canvasRef = useRef<PCDCanvasHandle | null>(null);
  const [source, setSource] = useState<Source | null>(null);
  const [url, setUrl] = useState("");
  const [pointSize, setPointSize] = useState(0.01);
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(true);
  const [sizeAttenuation, setSizeAttenuation] = useState(true);
  const [autoFit, setAutoFit] = useState(true);
  const [count, setCount] = useState<number>(0);
  const [bbox, setBbox] = useState<{ min: [number, number, number]; max: [number, number, number] } | null>(null);
  const [colorMode, setColorMode] = useState<"original" | "height" | "intensity" | "classification">("original");
  const [loading, setLoading] = useState(false);
  const [showPerf, setShowPerf] = useState<boolean>(false);
  // trajectory states
  const [plannedPath, setPlannedPath] = useState<[number, number, number][]>([]);
  const [livePath, setLivePath] = useState<[number, number, number][]>([]);
  const [showPlanned, setShowPlanned] = useState(true);
  const [showLive, setShowLive] = useState(true);
  const [plannedColor, setPlannedColor] = useState("#22c55e");
  const [liveColor, setLiveColor] = useState("#ef4444");
  const [tailLimit, setTailLimit] = useState(5000);
  const defaultWsUrl = (typeof window !== 'undefined' && window.location.hostname === 'localhost')
    ? 'ws://localhost:3001'
    : '/api/telemetry/ws?simulate=1';
  const [wsUrl, setWsUrl] = useState<string>(defaultWsUrl);
  const wsRef = useRef<WebSocket | null>(null);
  const [simulateLive, setSimulateLive] = useState<boolean>(true);
  const simTimerRef = useRef<number | null>(null);
  const simSegIndexRef = useRef<number>(0);
  const simURef = useRef<number>(0);

  // simulate live path when enabled (replay plannedPath if available)
  useEffect(() => {
    if (!simulateLive) {
      if (simTimerRef.current) { window.clearInterval(simTimerRef.current); simTimerRef.current = null; }
      return;
    }
    // close ws if any
    try { wsRef.current?.close(); } catch {}
    wsRef.current = null;
    // if we have a planned path, replay along it; else fallback to circle demo
    if (plannedPath && plannedPath.length >= 2) {
      // reset state
      simSegIndexRef.current = 0;
      simURef.current = 0;
      setLivePath([]);
      // precompute segment lengths
      const segLens: number[] = [];
      for (let i = 0; i < plannedPath.length - 1; i++) {
        const a = plannedPath[i], b = plannedPath[i + 1];
        const dx = b[0]-a[0], dy=b[1]-a[1], dz=b[2]-a[2];
        segLens.push(Math.hypot(dx, dy, dz));
      }
      const speed = 1.0; // meters per second
      const dt = 0.1; // seconds per tick (100ms)
      simTimerRef.current = window.setInterval(() => {
        let distStep = speed * dt;
        let i = simSegIndexRef.current;
        let u = simURef.current;
        while (distStep > 0 && i < segLens.length) {
          const L = Math.max(1e-6, segLens[i]);
          const rem = (1 - u) * L;
          if (distStep >= rem) {
            // advance to next segment end
            distStep -= rem;
            u = 0;
            i += 1;
          } else {
            u += distStep / L;
            distStep = 0;
          }
        }
        simSegIndexRef.current = i;
        simURef.current = u;
        if (i >= segLens.length) {
          // reached end; push final point once and stop
          setLivePath((prev) => {
            const end = plannedPath[plannedPath.length - 1];
            if (prev.length && prev[prev.length - 1][0] === end[0] && prev[prev.length - 1][1] === end[1] && prev[prev.length - 1][2] === end[2]) return prev;
            const next = prev.length >= tailLimit ? prev.slice(prev.length - tailLimit + 1) : prev.slice();
            next.push(end);
            return next;
          });
          if (simTimerRef.current) { window.clearInterval(simTimerRef.current); simTimerRef.current = null; }
          return;
        }
        const a = plannedPath[i];
        const b = plannedPath[i + 1];
        const x = a[0] + (b[0] - a[0]) * u;
        const y = a[1] + (b[1] - a[1]) * u;
        const z = a[2] + (b[2] - a[2]) * u;
        setLivePath((prev) => {
          const next = prev.length >= tailLimit ? prev.slice(prev.length - tailLimit + 1) : prev.slice();
          next.push([x, y, z]);
          return next;
        });
      }, 100);
    } else {
      // fallback circle demo
      let t = 0;
      simTimerRef.current = window.setInterval(() => {
        t += 0.1;
        const r = 1.5;
        const x = Math.cos(t) * r;
        const y = Math.sin(t) * r;
        const z = Math.sin(t * 0.5) * 0.5;
        setLivePath((prev) => {
          const next = prev.length >= tailLimit ? prev.slice(prev.length - tailLimit + 1) : prev.slice();
          next.push([x,y,z]);
          return next;
        });
      }, 100);
    }
    return () => { if (simTimerRef.current) { window.clearInterval(simTimerRef.current); simTimerRef.current = null; } };
  }, [simulateLive, tailLimit, plannedPath]);
  // no annotations handlers

  const onDrop = useCallback((e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    const f = e.dataTransfer.files?.[0];
    if (f) setSource({ type: "file", file: f });
  }, []);
  const [dragOver, setDragOver] = useState(false);
  const onDragOver = useCallback((e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    setDragOver(true);
  }, []);
  const onDragLeave = useCallback(() => setDragOver(false), []);
  const onDropWrapped = useCallback((e: React.DragEvent<HTMLDivElement>) => {
    setDragOver(false);
    onDrop(e);
  }, [onDrop]);
  const bboxText = useMemo(() => {
    if (!bbox) return "-";
    const [minx, miny, minz] = bbox.min;
    const [maxx, maxy, maxz] = bbox.max;
    const size = [maxx - minx, maxy - miny, maxz - minz];
    return `min(${minx.toFixed(3)}, ${miny.toFixed(3)}, ${minz.toFixed(3)})  max(${maxx.toFixed(3)}, ${maxy.toFixed(3)}, ${maxz.toFixed(3)})  size(${size[0].toFixed(3)}, ${size[1].toFixed(3)}, ${size[2].toFixed(3)})`;
  }, [bbox]);

  return (
    <div className="w-full h-[calc(100vh-2rem)] grid grid-cols-[300px_1fr] gap-4 p-4">
      <aside className="bg-card border border-border rounded-lg p-4 space-y-4">
        <h2 className="text-lg font-semibold">PCD 控制台</h2>
        {/* 标绘功能已移除以提升性能 */}
        <div className="space-y-2">
          <Label>从本地上传</Label>
          <div className="flex items-center gap-2">
            <Input type="file" accept=".pcd" onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) setSource({ type: "file", file: f });
            }} />
            <TooltipProvider>
              <Tooltip>
                <TooltipTrigger asChild>
                  <Button size="icon" variant="outline" onClick={() => setSource(null)}>
                    <RefreshCw className="h-4 w-4" />
                  </Button>
                </TooltipTrigger>
                <TooltipContent>清除</TooltipContent>
              </Tooltip>
            </TooltipProvider>
          </div>
        </div>

        <Separator />
        <div className="space-y-2">
          <Label>规划轨迹（JSON/CSV，x,y,z[,t]）</Label>
          <Input type="file" accept=".json,.csv" onChange={async (e) => {
            const f = e.target.files?.[0];
            if (!f) return;
            const text = await f.text();
            try {
              let pts: [number, number, number][] = [];
              if (f.name.endsWith('.json')) {
                const obj = JSON.parse(text);
                const arr = Array.isArray(obj?.points) ? obj.points : (Array.isArray(obj) ? obj : []);
                pts = (arr as unknown[])
                  .map((p) => {
                    const rec = p as Record<string, unknown>;
                    const x = Number(rec.x);
                    const y = Number(rec.y);
                    const z = Number(rec.z);
                    return [x, y, z] as [number, number, number];
                  })
                  .filter((p) => p.every(Number.isFinite));
              } else {
                // very small CSV parser: expects header with x,y,z or time,x,y,z
                const lines = text.split(/\r?\n/).filter(Boolean);
                const header = lines.shift()?.split(/,|\t|;|\s+/) ?? [];
                const xi = header.findIndex((h) => /^(x)$/i.test(h));
                const yi = header.findIndex((h) => /^(y)$/i.test(h));
                const zi = header.findIndex((h) => /^(z)$/i.test(h));
                if (xi < 0 || yi < 0 || zi < 0) throw new Error('CSV 需要包含 x,y,z 列');
                for (const line of lines) {
                  const cols = line.split(/,|\t|;|\s+/);
                  const x = Number(cols[xi]);
                  const y = Number(cols[yi]);
                  const z = Number(cols[zi]);
                  if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) pts.push([x,y,z]);
                }
              }
              setPlannedPath(pts);
            } catch (err) {
              alert('规划轨迹解析失败');
            }
          }} />
          <div className="flex items-center gap-2">
            <Switch checked={showPlanned} onCheckedChange={setShowPlanned} id="showPlanned" />
            <Label htmlFor="showPlanned">显示规划轨迹</Label>
          </div>
        </div>

        <div className="space-y-2">
          <Label>实时轨迹（WebSocket）</Label>
          <div className="flex gap-2">
            <Input value={wsUrl} onChange={(e) => setWsUrl(e.target.value)} disabled={simulateLive} />
            <Button disabled={simulateLive} onClick={() => {
              try {
                wsRef.current?.close();
              } catch {}
              const ws = new WebSocket(wsUrl);
              wsRef.current = ws;
              ws.onopen = () => {/* no-op */};
              ws.onmessage = (ev) => {
                try {
                  const d = JSON.parse(ev.data);
                  const x = Number(d.x), y = Number(d.y), z = Number(d.z);
                  if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
                    setLivePath((prev) => {
                      const next = prev.length >= tailLimit ? prev.slice(prev.length - tailLimit + 1) : prev.slice();
                      next.push([x,y,z]);
                      return next;
                    });
                  }
                } catch {}
              };
              ws.onerror = () => { /* ignore for now */ };
              ws.onclose = () => { /* disconnected */ };
            }}>连接</Button>
            <Button variant="outline" disabled={simulateLive} onClick={() => { wsRef.current?.close(); wsRef.current = null; }}>断开</Button>
          </div>
          <div className="flex items-center gap-2">
            <Switch checked={showLive} onCheckedChange={setShowLive} id="showLive" />
            <Label htmlFor="showLive">显示实时轨迹</Label>
          </div>
          <div className="flex items-center gap-2">
            <Switch checked={simulateLive} onCheckedChange={setSimulateLive} id="simulateLive" />
            <Label htmlFor="simulateLive">前端模拟轨迹</Label>
          </div>
          <div className="space-y-1">
            <Label>尾迹长度（点数）</Label>
            <Slider value={[tailLimit]} min={100} max={20000} step={100} onValueChange={(v) => setTailLimit(v[0] ?? 5000)} />
            <div className="text-xs text-muted-foreground">{tailLimit}</div>
          </div>
        </div>

        <Separator />
        <div className="space-y-2">
          <Label>着色模式</Label>
          <Select
            value={colorMode}
            onValueChange={(v: "original" | "height" | "intensity" | "classification") => setColorMode(v)}
          >
            <SelectTrigger className="w-full">
              <SelectValue placeholder="选择着色模式" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="original">原始</SelectItem>
              <SelectItem value="height">按高度</SelectItem>
              <SelectItem value="intensity">按强度</SelectItem>
              <SelectItem value="classification">按分类</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <div className="space-y-2">
          <Label htmlFor="pcd-url">从 URL 加载</Label>
          <div className="flex gap-2">
            <Input id="pcd-url" placeholder="https://.../example.pcd" value={url} onChange={(e) => setUrl(e.target.value)} />
            <Button onClick={() => url && setSource({ type: "url", url })}>
              <LinkIcon className="h-4 w-4 mr-2" /> 加载
            </Button>
          </div>
        </div>

        <Separator />

        <div className="space-y-2">
          <Label>点大小</Label>
          <div className="px-1">
            <Slider value={[pointSize]} min={0.001} max={0.1} step={0.001} onValueChange={(v) => setPointSize(v[0] ?? 0.01)} />
          </div>
          <div className="text-xs text-muted-foreground">{pointSize.toFixed(3)}</div>
        </div>

        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Switch checked={showGrid} onCheckedChange={setShowGrid} id="grid" />
            <Label htmlFor="grid">显示网格</Label>
          </div>
          <div className="flex items-center gap-2">
            <Switch checked={showAxes} onCheckedChange={setShowAxes} id="axes" />
            <Label htmlFor="axes">显示坐标轴</Label>
          </div>
        </div>

        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Switch checked={sizeAttenuation} onCheckedChange={setSizeAttenuation} id="attenuation" />
            <Label htmlFor="attenuation">点大小距离衰减</Label>
          </div>
          <div className="flex items-center gap-2">
            <Switch checked={autoFit} onCheckedChange={setAutoFit} id="autofit" />
            <Label htmlFor="autofit">加载后自动适配</Label>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <Switch checked={showPerf} onCheckedChange={setShowPerf} id="perf" />
          <Label htmlFor="perf">性能监控</Label>
        </div>

        <Separator />
        <div className="space-y-1 text-xs text-muted-foreground">
          <div>点数：{count}</div>
          <div>包围盒：{bboxText}</div>
        </div>

        <div className="pt-2">
          <Button className="w-full" variant="secondary" onClick={() => canvasRef.current?.fitToView()}>
            <Maximize className="h-4 w-4 mr-2" /> 视图自适应
          </Button>
        </div>

      </aside>
      <section
        className={`bg-card border border-border rounded-lg relative overflow-hidden ${dragOver ? "ring-2 ring-primary/60" : ""}`}
        onDrop={onDropWrapped}
        onDragOver={onDragOver}
        onDragLeave={onDragLeave}
      >
        {!source ? (
          <div className="absolute inset-0 flex items-center justify-center text-muted-foreground">
            <div className="text-center space-y-3">
              <Upload className="w-8 h-8 mx-auto" />
              <div className="text-sm">拖拽或选择 .pcd 文件，或输入 URL 加载</div>
            </div>
          </div>
        ) : null}
        <div className="w-full h-full min-h-[400px] relative">
          {loading && (
            <div className="absolute inset-0 z-10 flex items-center justify-center bg-background/60 backdrop-blur-sm">
              <div className="flex flex-col items-center gap-3 text-muted-foreground">
                <div className="h-8 w-8 animate-spin rounded-full border-2 border-primary border-t-transparent" />
                <div className="text-xs">正在加载 PCD...</div>
              </div>
            </div>
          )}
          <PCDCanvas
            ref={canvasRef}
            source={source}
            pointSize={pointSize}
            showGrid={showGrid}
            showAxes={showAxes}
            sizeAttenuation={sizeAttenuation}
            autoFit={autoFit}
            colorMode={colorMode}
            showPerf={showPerf}
            plannedPath={plannedPath}
            livePath={livePath}
            showPlanned={showPlanned}
            showLive={showLive}
            plannedColor={plannedColor}
            liveColor={liveColor}
            onLoadedAction={({ bbox, count }) => {
              setCount(count);
              setBbox({
                min: [bbox.min.x, bbox.min.y, bbox.min.z],
                max: [bbox.max.x, bbox.max.y, bbox.max.z],
              });
            }}
            onLoadingChange={setLoading}
          />
        </div>
      </section>
    </div>
  );
}
