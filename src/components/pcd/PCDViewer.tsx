"use client";

import { useRef, useState } from "react";
import { PCDCanvas, PCDCanvasHandle, Source } from "./PCDCanvas";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Separator } from "@/components/ui/separator";
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from "@/components/ui/tooltip";
import { RefreshCw, Maximize, Upload, Link as LinkIcon } from "lucide-react";

export default function PCDViewer() {
  const canvasRef = useRef<PCDCanvasHandle | null>(null);
  const [source, setSource] = useState<Source | null>(null);
  const [url, setUrl] = useState("");
  const [pointSize, setPointSize] = useState(0.01);
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(true);

  return (
    <div className="w-full h-[calc(100vh-2rem)] grid grid-cols-[300px_1fr] gap-4 p-4">
      <aside className="bg-card border border-border rounded-lg p-4 space-y-4">
        <h2 className="text-lg font-semibold">PCD 控制台</h2>
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

        <div className="pt-2">
          <Button className="w-full" variant="secondary" onClick={() => canvasRef.current?.fitToView()}>
            <Maximize className="h-4 w-4 mr-2" /> 视图自适应
          </Button>
        </div>

        <Separator />
        <div className="text-xs text-muted-foreground">
          支持 ASCII PCD（若为二进制 PCD，建议先转换为 ASCII）。
        </div>
      </aside>
      <section className="bg-card border border-border rounded-lg relative overflow-hidden">
        {!source ? (
          <div className="absolute inset-0 flex items-center justify-center text-muted-foreground">
            <div className="text-center space-y-3">
              <Upload className="w-8 h-8 mx-auto" />
              <div className="text-sm">拖拽或选择 .pcd 文件，或输入 URL 加载</div>
            </div>
          </div>
        ) : null}
        <div className="w-full h-full min-h-[400px]">
          <PCDCanvas
            ref={canvasRef}
            source={source}
            pointSize={pointSize}
            showGrid={showGrid}
            showAxes={showAxes}
            onLoadedAction={() => {}}
          />
        </div>
      </section>
    </div>
  );
}
