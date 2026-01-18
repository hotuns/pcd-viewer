"use client";

import { useCallback, useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import MissionController from "@/components/pcd/MissionController";
import type { Mission } from "@/types/mission";

const deserializeMission = (mission: Mission): Mission => ({
  ...mission,
  createdAt: new Date(mission.createdAt),
  startedAt: mission.startedAt ? new Date(mission.startedAt) : undefined,
  completedAt: mission.completedAt ? new Date(mission.completedAt) : undefined,
});

type MissionDetailClientProps = {
  missionId: string;
};

export default function MissionDetailClient({ missionId }: MissionDetailClientProps) {
  const router = useRouter();
  const [mission, setMission] = useState<Mission | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const loadMission = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const res = await fetch(`/api/missions/${missionId}`);
      if (!res.ok) {
        throw new Error(`Failed to load mission ${missionId}`);
      }
      const data = await res.json();
      if (!data?.mission) {
        throw new Error("Mission not found");
      }
      setMission(deserializeMission(data.mission));
    } catch (err) {
      console.error(err);
      setError(err instanceof Error ? err.message : "未知错误");
    } finally {
      setLoading(false);
    }
  }, [missionId]);

  useEffect(() => {
    loadMission();
  }, [loadMission]);

  const handleBack = () => {
    router.push("/");
  };

  if (loading) {
    return (
      <div className="h-screen w-full flex items-center justify-center text-sm text-muted-foreground">
        正在加载任务 {missionId} ...
      </div>
    );
  }

  if (error || !mission) {
    return (
      <div className="h-screen w-full flex flex-col items-center justify-center gap-3 text-sm text-muted-foreground">
        <div>无法加载任务：{error || "任务不存在"}</div>
        <button
          onClick={handleBack}
          className="px-3 py-1 rounded border border-border text-xs text-foreground"
        >
          返回主页
        </button>
      </div>
    );
  }

  return (
    <div className="h-screen w-full overflow-hidden">
      <MissionController initialMission={mission} onBack={handleBack} />
    </div>
  );
}
