"use client";

import { useState } from "react";
import MissionController from "@/components/pcd/MissionController";
import StartupPage from "@/components/StartupPage";
import { Mission } from "@/types/mission";

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
      <MissionController 
        initialMission={selectedMission} 
        onBack={handleBackToStartup}
      />
    </div>
  );
}
