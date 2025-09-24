import PCDViewer from "@/components/pcd/PCDViewer";

export default function Home() {
  return (
    <div className="min-h-screen w-full">
      <header className="h-14 border-b border-border flex items-center px-4">
        <div className="text-sm text-muted-foreground">点云查看工具</div>
        <div className="ml-auto text-xs text-muted-foreground">Next.js + @react-three/fiber + shadcn/ui</div>
      </header>
      <PCDViewer />
    </div>
  );
}
