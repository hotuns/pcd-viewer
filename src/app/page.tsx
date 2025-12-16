"use client";

import { useCallback } from "react";
import { useRouter } from "next/navigation";
import StartupPage from "@/components/StartupPage";
import { Mission } from "@/types/mission";

export default function Home() {
  const router = useRouter();

  const handleMissionSelect = useCallback((mission: Mission) => {
    router.push(`/mission/${mission.id}`);
  }, [router]);

  return <StartupPage onMissionSelect={handleMissionSelect} />;
}
