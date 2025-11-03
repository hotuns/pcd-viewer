"use client";

import { useEffect, useRef, useState } from "react";
import { Mission, Source } from "@/types/mission";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { PlusCircle, Trash2 } from "lucide-react";

interface MissionManagerProps {
  onMissionSelect: (mission: Mission | null) => void;
  selectedMission: Mission | null;
}

export default function MissionManager({ onMissionSelect, selectedMission }: MissionManagerProps) {
  const [missions, setMissions] = useState<Mission[]>([
    {
      id: "default",
      name: "默认任务",
      createdAt: new Date(),
      scene: { type: "url", url: "/rosbag_play.pcd" },
      trajectory: { type: "url", url: "/example-planned-path.json" },
    },
  ]);
  const [newMissionName, setNewMissionName] = useState("");
  

  // 首次加载时自动选中默认任务
  useEffect(() => {
    if (!selectedMission && missions.length > 0) {
      onMissionSelect(missions[0]);
    }
  }, [selectedMission, missions, onMissionSelect]);

  

  const createMission = () => {
    if (!newMissionName.trim()) {
      alert("请输入任务名称");
      return;
    }
    const mission: Mission = {
      id: Date.now().toString(),
      name: newMissionName,
      createdAt: new Date(),
    };
    setMissions([...missions, mission]);
    setNewMissionName("");
    onMissionSelect(mission);
  };

  const deleteMission = (id: string) => {
    setMissions(missions.filter(m => m.id !== id));
    if (selectedMission?.id === id) {
      onMissionSelect(null);
    }
  };

  const updateMissionScene = (file: File) => {
    if (!selectedMission) return;
    const updated = { ...selectedMission, scene: { type: "file" as const, file } };
    setMissions(missions.map(m => m.id === selectedMission.id ? updated : m));
    onMissionSelect(updated);
  };

  const updateMissionSceneURL = (url: string) => {
    if (!selectedMission || !url) return;
    const updated = { ...selectedMission, scene: { type: "url" as const, url } };
    setMissions(missions.map(m => m.id === selectedMission.id ? updated : m));
    onMissionSelect(updated);
  };

  const updateMissionTrajectory = (file: File) => {
    if (!selectedMission) return;
    const updated = { ...selectedMission, trajectory: { type: "file" as const, file } };
    setMissions(missions.map(m => m.id === selectedMission.id ? updated : m));
    onMissionSelect(updated);
  };

  const updateMissionTrajectoryURL = (url: string) => {
    if (!selectedMission || !url) return;
    const updated = { ...selectedMission, trajectory: { type: "url" as const, url } };
    setMissions(missions.map(m => m.id === selectedMission.id ? updated : m));
    onMissionSelect(updated);
  };

  return (
    <div className="space-y-4">
      <h2 className="text-lg font-semibold">任务管理</h2>
      
      {/* 创建新任务 */}
      <div className="space-y-2">
        <Label>创建新任务</Label>
        <div className="flex gap-2">
          <Input
            placeholder="任务名称"
            value={newMissionName}
            onChange={(e) => setNewMissionName(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && createMission()}
          />
          <Button onClick={createMission} size="icon">
            <PlusCircle className="h-4 w-4" />
          </Button>
        </div>
      </div>

      <Separator />

      {/* 任务列表 */}
      <div className="space-y-2">
        <Label>任务列表</Label>
        {missions.length === 0 ? (
          <div className="text-sm text-muted-foreground">暂无任务</div>
        ) : (
          <div className="space-y-1">
            {missions.map(mission => (
              <div
                key={mission.id}
                className={`flex items-center justify-between p-2 rounded border cursor-pointer transition-colors ${
                  selectedMission?.id === mission.id
                    ? "border-primary bg-primary/10"
                    : "border-border hover:bg-accent"
                }`}
                onClick={() => onMissionSelect(mission)}
              >
                <div className="flex-1">
                  <div className="text-sm font-medium">{mission.name}</div>
                  <div className="text-xs text-muted-foreground">
                    {mission.scene ? "✓ 场景" : "✗ 场景"} | {mission.trajectory ? "✓ 规划航线" : "✗ 规划航线"}
                  </div>
                </div>
                <Button
                  size="icon"
                  variant="ghost"
                  onClick={(e) => {
                    e.stopPropagation();
                    deleteMission(mission.id);
                  }}
                >
                  <Trash2 className="h-4 w-4" />
                </Button>
              </div>
            ))}
          </div>
        )}
      </div>

      <Separator />

      {/* 当前任务配置（精简版） */}
      {selectedMission && (
        <CompactConfig
          mission={selectedMission as Mission}
          onSceneFile={updateMissionScene}
          onSceneURL={updateMissionSceneURL}
          onSceneClear={() => {
            const { scene: _omit, ...rest } = selectedMission as Mission & { scene?: Mission["scene"] };
            const updated: Mission = { ...rest };
            setMissions(missions.map(m => m.id === selectedMission!.id ? updated : m));
            onMissionSelect(updated);
          }}
          onTrajFile={updateMissionTrajectory}
          onTrajURL={updateMissionTrajectoryURL}
          onTrajClear={() => {
            const { trajectory: _omit, ...rest } = selectedMission as Mission & { trajectory?: Mission["trajectory"] };
            const updated: Mission = { ...rest };
            setMissions(missions.map(m => m.id === selectedMission!.id ? updated : m));
            onMissionSelect(updated);
          }}
        />
      )}
    </div>
  );
}

function CompactConfig({
  mission,
  onSceneFile,
  onSceneURL,
  onSceneClear,
  onTrajFile,
  onTrajURL,
  onTrajClear,
}: {
  mission: Mission;
  onSceneFile: (file: File) => void;
  onSceneURL: (url: string) => void;
  onSceneClear: () => void;
  onTrajFile: (file: File) => void;
  onTrajURL: (url: string) => void;
  onTrajClear: () => void;
}) {
  const sceneInputRef = useRef<HTMLInputElement | null>(null);
  const trajInputRef = useRef<HTMLInputElement | null>(null);

  const summarize = (src: Mission["scene"] | Mission["trajectory"]) => {
    if (!src) return "未设置";
    if (src.type === "file") return src.file.name;
    try {
      const u = new URL(src.url, location.origin);
      return u.pathname.split("/").pop() || src.url;
    } catch {
      return src.url;
    }
  };

  const shortenMiddle = (text: string, max = 28) => {
    if (text.length <= max) return text;
    const keep = max - 3;
    const head = Math.ceil(keep / 2);
    const tail = Math.floor(keep / 2);
    return `${text.slice(0, head)}...${text.slice(-tail)}`;
  };

  const askURL = async (title: string) => {
    const url = prompt(title);
    if (url && url.trim()) return url.trim();
    return undefined;
  };

  return (
    <div className="space-y-3">
      <Label>配置任务: {mission.name}</Label>

      {/* 场景行 */}
      <div className="flex items-center justify-between gap-2 text-sm">
        <div className="min-w-0 flex-1">
          <span className="text-muted-foreground mr-2">场景</span>
          <span
            className="truncate inline-block max-w-[180px] sm:max-w-[220px] align-middle"
            title={summarize(mission.scene)}
          >
            {shortenMiddle(summarize(mission.scene))}
          </span>
        </div>
        <div className="flex gap-2">
          <input ref={sceneInputRef} type="file" accept=".pcd" className="hidden" onChange={(e) => {
            const file = e.target.files?.[0];
            if (file) onSceneFile(file);
            if (sceneInputRef.current) sceneInputRef.current.value = "";
          }} />
          <Button size="sm" variant="outline" onClick={() => sceneInputRef.current?.click()}>文件</Button>
          <Button size="sm" variant="outline" onClick={async () => {
            const url = await askURL("输入点云 URL (.pcd)");
            if (url) onSceneURL(url);
          }}>URL</Button>
          <Button size="sm" variant="ghost" onClick={onSceneClear}>清除</Button>
        </div>
      </div>

      {/* 规划航线路 */}
      <div className="flex items-center justify-between gap-2 text-sm">
        <div className="min-w-0 flex-1">
          <span className="text-muted-foreground mr-2">规划航线</span>
          <span
            className="truncate inline-block max-w-[180px] sm:max-w-[220px] align-middle"
            title={summarize(mission.trajectory)}
          >
            {shortenMiddle(summarize(mission.trajectory))}
          </span>
        </div>
        <div className="flex gap-2">
          <input ref={trajInputRef} type="file" accept=".json,.csv" className="hidden" onChange={(e) => {
            const file = e.target.files?.[0];
            if (file) onTrajFile(file);
            if (trajInputRef.current) trajInputRef.current.value = "";
          }} />
          <Button size="sm" variant="outline" onClick={() => trajInputRef.current?.click()}>文件</Button>
          <Button size="sm" variant="outline" onClick={async () => {
            const url = await askURL("输入航线 URL (.json/.csv)");
            if (url) onTrajURL(url);
          }}>URL</Button>
          <Button size="sm" variant="ghost" onClick={onTrajClear}>清除</Button>
        </div>
      </div>
    </div>
  );
}
