import { useRef } from "react";
import { Mission } from "@/types/mission";
import { Button } from "@/components/ui/button";
import { Label } from "@/components/ui/label";
import { Upload, Link as LinkIcon, X, Map, Route, AlertCircle, CheckCircle2 } from "lucide-react";
import { useMissionDatabase } from "@/hooks/useMissionDatabase";
import { uploadMissionFile } from "@/lib/upload";

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

  const updateSceneFile = async (file: File) => {
    const uploadedUrl = await uploadMissionFile(file, "scene");
    persist({ scene: { type: "url", url: uploadedUrl } });
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
    const uploadedUrl = await uploadMissionFile(file, "trajectory");
    persist({ trajectory: { type: "url", url: uploadedUrl } });
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
    <div className="space-y-4 text-xs">
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <Label className="text-xs text-muted-foreground">场景点云 (.pcd/.ply)</Label>
          {selectedMission.scene && <CheckCircle2 className="w-3.5 h-3.5 text-green-500" />}
        </div>
        {!selectedMission.scene ? (
          <div className="flex gap-2">
            <Button size="sm" className="flex-1 h-7 text-xs gap-1" variant="outline" onClick={() => sceneFileRef.current?.click()}>
              <Upload className="h-3.5 w-3.5" /> 上传
            </Button>
            <Button
              size="sm"
              className="flex-1 h-7 text-xs gap-1"
              variant="ghost"
              onClick={() => {
                const url = prompt("输入场景URL (.pcd/.ply):");
                if (url) updateSceneUrl(url);
              }}
            >
              <LinkIcon className="h-3.5 w-3.5" /> URL
            </Button>
          </div>
        ) : (
          <div className="flex items-center gap-2 rounded border border-border bg-muted/40 px-2 py-1">
            <Map className="h-4 w-4 text-primary" />
            <div className="flex-1 min-w-0">
              <div className="text-[11px] font-medium truncate">
                {selectedMission.scene.type === "file" ? selectedMission.scene.file.name : selectedMission.scene.url.split("/").pop()}
              </div>
              <div className="text-[10px] text-muted-foreground">
                {selectedMission.scene.type === "file"
                  ? `${(selectedMission.scene.file.size / 1024 / 1024).toFixed(2)} MB`
                  : "远程资源"}
              </div>
            </div>
            <Button size="icon" variant="ghost" onClick={clearScene} className="h-7 w-7 text-muted-foreground hover:text-destructive">
              <X className="h-4 w-4" />
            </Button>
          </div>
        )}
      </div>

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

      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <Label className="text-xs text-muted-foreground">航线文件 (.json)（可选）</Label>
          {selectedMission.trajectory && <CheckCircle2 className="w-3.5 h-3.5 text-green-500" />}
        </div>
        {!selectedMission.trajectory ? (
          <div className="flex gap-2">
            <Button size="sm" className="flex-1 h-7 text-xs gap-1" variant="outline" onClick={() => trajectoryFileRef.current?.click()}>
              <Upload className="h-3.5 w-3.5" /> 上传
            </Button>
            <Button
              size="sm"
              className="flex-1 h-7 text-xs gap-1"
              variant="ghost"
              onClick={() => {
                const url = prompt("输入航线 URL (.json):");
                if (url) updateTrajectoryUrl(url);
              }}
            >
              <LinkIcon className="h-3.5 w-3.5" /> URL
            </Button>
          </div>
        ) : (
          <div className="flex items-center gap-2 rounded border border-border bg-muted/40 px-2 py-1">
            <Route className="h-4 w-4 text-primary" />
            <div className="flex-1 min-w-0">
              <div className="text-[11px] font-medium truncate">
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
            <Button size="icon" variant="ghost" onClick={clearTrajectory} className="h-7 w-7 text-muted-foreground hover:text-destructive">
              <X className="h-4 w-4" />
            </Button>
          </div>
        )}
      </div>

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
