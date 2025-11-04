"use client";

import { useState } from "react";
import MissionController from "@/components/pcd/MissionController";
import StartupPage from "@/components/StartupPage";
import { Mission } from "@/types/mission";
import { Button } from "@/components/ui/button";
import { ArrowLeft } from "lucide-react";

export default function Home() {
  const [selectedMission, setSelectedMission] = useState<Mission | null>(null);

  const handleBackToStartup = () => {
    setSelectedMission(null);
  };

  if (!selectedMission) {
    return <StartupPage onMissionSelect={setSelectedMission} />;
  }

  return (
    <div className="min-h-screen w-full">
      <header className="h-14 border-b border-border flex items-center px-4">
        <Button 
          variant="ghost" 
          size="sm"
          onClick={handleBackToStartup}
          className="mr-4"
        >
          <ArrowLeft className="h-4 w-4 mr-2" />
          返回任务列表
        </Button>
        <div className="text-sm text-muted-foreground">
          任务: {selectedMission.name}
        </div>
        <div className="ml-auto text-xs text-muted-foreground">
          任务创建 + 场景/规划航线 + ROS 实时位置
        </div>
      </header>
      <MissionController initialMission={selectedMission} />
    </div>
  );
}
