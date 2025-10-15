"use client";

import { useState } from "react";
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
  const [missions, setMissions] = useState<Mission[]>([]);
  const [newMissionName, setNewMissionName] = useState("");

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
                    {mission.scene ? "✓ 场景" : "✗ 场景"} | {mission.trajectory ? "✓ 航线" : "✗ 航线"}
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

      {/* 当前任务配置 */}
      {selectedMission && (
        <div className="space-y-3">
          <Label>配置任务: {selectedMission.name}</Label>
          
          {/* 场景（点云）配置 */}
          <div className="space-y-2">
            <Label className="text-sm">场景（点云文件）</Label>
            <Input
              type="file"
              accept=".pcd"
              onChange={(e) => {
                const file = e.target.files?.[0];
                if (file) updateMissionScene(file);
              }}
            />
            <div className="flex gap-2">
              <Input
                placeholder="或输入 URL"
                onBlur={(e) => updateMissionSceneURL(e.target.value)}
              />
            </div>
            {selectedMission.scene && (
              <div className="text-xs text-muted-foreground">
                ✓ {selectedMission.scene.type === "file" 
                  ? selectedMission.scene.file.name 
                  : selectedMission.scene.url}
              </div>
            )}
          </div>

          {/* 航线轨迹配置 */}
          <div className="space-y-2">
            <Label className="text-sm">航线轨迹（JSON/CSV）</Label>
            <Input
              type="file"
              accept=".json,.csv"
              onChange={(e) => {
                const file = e.target.files?.[0];
                if (file) updateMissionTrajectory(file);
              }}
            />
            <div className="flex gap-2">
              <Input
                placeholder="或输入 URL"
                onBlur={(e) => updateMissionTrajectoryURL(e.target.value)}
              />
            </div>
            {selectedMission.trajectory && (
              <div className="text-xs text-muted-foreground">
                ✓ {selectedMission.trajectory.type === "file" 
                  ? selectedMission.trajectory.file.name 
                  : selectedMission.trajectory.url}
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
