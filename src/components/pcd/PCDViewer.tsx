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
