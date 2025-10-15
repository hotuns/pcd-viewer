import MissionController from "@/components/pcd/MissionController";

export default function Home() {
  return (
    <div className="min-h-screen w-full">
      <header className="h-14 border-b border-border flex items-center px-4">
        <div className="text-sm text-muted-foreground">无人机任务管理工具</div>
        <div className="ml-auto text-xs text-muted-foreground">任务创建 + 场景/航线 + ROS 实时位置</div>
      </header>
      <MissionController />
    </div>
  );
}
