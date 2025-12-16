import { useRef } from "react";
import { Mission } from "@/types/mission";
import { Button } from "@/components/ui/button";
import { Label } from "@/components/ui/label";
import { Upload, Link as LinkIcon, X, Map, Route, AlertCircle, CheckCircle2 } from "lucide-react";
import { useMissionDatabase } from "@/hooks/useMissionDatabase";

interface TaskConfigPanelProps {
  selectedMission: Mission | null;
  onMissionUpdate: (mission: Mission) => void;
}

export function TaskConfigPanel({ selectedMission, onMissionUpdate }: TaskConfigPanelProps) {
  const sceneFileRef = useRef<HTMLInputElement>(null);
  const trajectoryFileRef = useRef<HTMLInputElement>(null);
  const { updateMission: updateMissionDB } = useMissionDatabase();

  if (!selectedMission) {
    return (
      <div className="flex flex-col items-center justify-center py-8 text-muted-foreground space-y-2">
        <AlertCircle className="h-8 w-8 opacity-50" />
        <div className="text-sm">未选择任务</div>
      </div>
    );
  }

  const persist = async (updates: Partial<Mission>) => {
    const nextMission = { ...selectedMission, ...updates };
    onMissionUpdate(nextMission);
    await updateMissionDB(nextMission.id, updates);
  };

  const fileToDataUrl = (file: File) => {
    return new Promise<string>((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => resolve(reader.result as string);
      reader.onerror = reject;
      reader.readAsDataURL(file);
    });
  };

  const updateSceneFile = async (file: File) => {
    const dataUrl = await fileToDataUrl(file);
    persist({ scene: { type: "url", url: dataUrl } });
  };

  const updateSceneUrl = (url: string) => {
    if (!url.trim()) return;
    persist({ scene: { type: "url", url } });
  };

  const clearScene = () => {
    persist({ scene: undefined });
    if (sceneFileRef.current) {
      sceneFileRef.current.value = "";
    }
  };

  const updateTrajectoryFile = async (file: File) => {
    const dataUrl = await fileToDataUrl(file);
    persist({ trajectory: { type: "url", url: dataUrl } });
  };

  const updateTrajectoryUrl = (url: string) => {
    if (!url.trim()) return;
    persist({ trajectory: { type: "url", url } });
  };

  const clearTrajectory = () => {
    persist({ trajectory: undefined });
    if (trajectoryFileRef.current) {
      trajectoryFileRef.current.value = "";
    }
  };

  return (
    <div className="space-y-8">
      <div className="flex items-center justify-between">
        <Label className="text-xs text-muted-foreground">场景点云 (.pcd/.ply)</Label>
        {selectedMission.scene && <CheckCircle2 className="w-4 h-4 text-green-500" />}
      </div>

      {!selectedMission.scene ? (
        <div className="grid grid-cols-2 gap-3">
          <button
            onClick={() => sceneFileRef.current?.click()}
            className="flex flex-col items-center justify-center gap-2 p-4 rounded-xl border border-dashed border-border hover:border-primary/50 hover:bg-primary/5 transition-all group"
          >
            <div className="p-2 rounded-full bg-muted group-hover:bg-background transition-colors">
              <Upload className="h-4 w-4 text-muted-foreground group-hover:text-primary" />
            </div>
            <span className="text-xs text-muted-foreground group-hover:text-foreground">上传文件</span>
          </button>
          <button
            onClick={() => {
              const url = prompt("输入场景URL (.pcd/.ply):");
              if (url) updateSceneUrl(url);
            }}
            className="flex flex-col items-center justify-center gap-2 p-4 rounded-xl border border-dashed border-border hover:border-primary/50 hover:bg-primary/5 transition-all group"
          >
            <div className="p-2 rounded-full bg-muted group-hover:bg-background transition-colors">
              <LinkIcon className="h-4 w-4 text-muted-foreground group-hover:text-primary" />
            </div>
            <span className="text-xs text-muted-foreground group-hover:text-foreground">输入 URL</span>
          </button>
        </div>
      ) : (
        <div className="relative group overflow-hidden rounded-xl border border-border bg-muted/30 p-3 transition-all hover:border-primary/30">
          <div className="flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 text-primary">
              <Map className="h-4 w-4" />
            </div>
            <div className="flex-1 min-w-0">
              <div className="text-xs font-medium truncate">
                {selectedMission.scene.type === "file" ? selectedMission.scene.file.name : selectedMission.scene.url.split("/").pop()}
              </div>
              <div className="text-[10px] text-muted-foreground">
                {selectedMission.scene.type === "file"
                  ? `${(selectedMission.scene.file.size / 1024 / 1024).toFixed(2)} MB`
                  : "远程资源"}
              </div>
            </div>
            <Button size="icon" variant="ghost" onClick={clearScene} className="h-8 w-8 text-muted-foreground hover:text-destructive">
              <X className="h-4 w-4" />
            </Button>
          </div>
        </div>
      )}

      <input
        ref={sceneFileRef}
        type="file"
        accept=".pcd,.ply"
        className="hidden"
        onChange={(event) => {
          const file = event.target.files?.[0];
          if (file) void updateSceneFile(file);
        }}
      />

      <div className="flex items-center justify-between">
        <Label className="text-xs text-muted-foreground">航线文件 (.json)</Label>
        {selectedMission.trajectory && <CheckCircle2 className="w-4 h-4 text-green-500" />}
      </div>

      {!selectedMission.trajectory ? (
        <div className="grid grid-cols-2 gap-3">
          <button
            onClick={() => trajectoryFileRef.current?.click()}
            className="flex flex-col items-center justify-center gap-2 p-4 rounded-xl border border-dashed border-border hover:border-primary/50 hover:bg-primary/5 transition-all group"
          >
            <div className="p-2 rounded-full bg-muted group-hover:bg-background transition-colors">
              <Upload className="h-4 w-4 text-muted-foreground group-hover:text-primary" />
            </div>
            <span className="text-xs text-muted-foreground group-hover:text-foreground">上传文件</span>
          </button>
          <button
            onClick={() => {
              const url = prompt("输入航线 URL (.json):");
              if (url) updateTrajectoryUrl(url);
            }}
            className="flex flex-col items-center justify-center gap-2 p-4 rounded-xl border border-dashed border-border hover:border-primary/50 hover:bg-primary/5 transition-all group"
          >
            <div className="p-2 rounded-full bg-muted group-hover:bg-background transition-colors">
              <LinkIcon className="h-4 w-4 text-muted-foreground group-hover:text-primary" />
            </div>
            <span className="text-xs text-muted-foreground group-hover:text-foreground">输入 URL</span>
          </button>
        </div>
      ) : (
        <div className="relative group overflow-hidden rounded-xl border border-border bg-muted/30 p-3 transition-all hover:border-primary/30">
          <div className="flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10 text-primary">
              <Route className="h-4 w-4" />
            </div>
            <div className="flex-1 min-w-0">
              <div className="text-xs font-medium truncate">
                {selectedMission.trajectory.type === "file"
                  ? selectedMission.trajectory.file.name
                  : selectedMission.trajectory.url.split("/").pop()}
              </div>
              <div className="text-[10px] text-muted-foreground">
                {selectedMission.trajectory.type === "file"
                  ? `${(selectedMission.trajectory.file.size / 1024 / 1024).toFixed(2)} MB`
                  : "远程资源"}
              </div>
            </div>
            <Button size="icon" variant="ghost" onClick={clearTrajectory} className="h-8 w-8 text-muted-foreground hover:text-destructive">
              <X className="h-4 w-4" />
            </Button>
          </div>
        </div>
      )}

      <input
        ref={trajectoryFileRef}
        type="file"
        accept=".json,.txt"
        className="hidden"
        onChange={(event) => {
          const file = event.target.files?.[0];
          if (file) void updateTrajectoryFile(file);
        }}
      />
    </div>
  );
}
