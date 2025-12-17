import { Mission, MissionPhase } from "@/types/mission";
import { Badge } from "@/components/ui/badge";
import { Map, Route, Calendar, Clock, Activity } from "lucide-react";

interface TaskInfoProps {
  selectedMission: Mission | null;
  phase?: MissionPhase;
}

const phaseText: Record<MissionPhase, string> = {
  idle: "未执行",
  hangar_ready: "机库就绪",
  mission_upload: "上传任务",
  takeoff_ready: "起飞前检查",
  takeoff: "起飞中",
  executing: "任务执行",
  returning: "返航",
  landing: "降落",
  charging: "充电中",
  error: "异常",
};

export function TaskInfo({ selectedMission, phase }: TaskInfoProps) {
  const getStatusText = (status: string) => {
    const statusMap: Record<string, string> = {
      draft: '草稿',
      configured: '已配置',
      planning: '编辑中',
      ready: '就绪',
      running: '执行中',
      paused: '暂停',
      completed: '完成',
      failed: '失败'
    };
    return statusMap[status] || status;
  };

  const getStatusColor = (status: string) => {
    const colorMap: Record<string, "default" | "secondary" | "destructive" | "outline"> = {
      draft: 'secondary',
      configured: 'secondary',
      planning: 'outline',
      ready: 'default',
      running: 'destructive',
      paused: 'secondary',
      completed: 'default',
      failed: 'destructive'
    };
    return colorMap[status] || 'default';
  };

  if (!selectedMission) {
    return (
      <div className="flex flex-col items-center justify-center py-8 text-center space-y-3">
        <div className="w-12 h-12 rounded-full bg-muted flex items-center justify-center">
          <Map className="h-6 w-6 text-muted-foreground" />
        </div>
        <div>
          <h2 className="text-base font-semibold">无任务选中</h2>
          <div className="text-xs text-muted-foreground mt-1">
            请返回首页选择或创建一个新任务
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      <div className="flex items-start justify-between gap-2">
        <div className="space-y-1 overflow-hidden">
          <h2 className="text-lg font-bold truncate leading-tight" title={selectedMission.name}>
            {selectedMission.name}
          </h2>
          <div className="flex items-center gap-2 text-[10px] text-muted-foreground">
            <span className="flex items-center gap-1">
              <Calendar className="h-3 w-3" />
              {(() => {
                try {
                  const d = new Date(selectedMission.createdAt);
                  return isNaN(d.getTime()) ? "未知日期" : d.toLocaleDateString();
                } catch { return "未知日期"; }
              })()}
            </span>
            <span className="flex items-center gap-1">
              <Clock className="h-3 w-3" />
              {(() => {
                try {
                  const d = new Date(selectedMission.createdAt);
                  return isNaN(d.getTime()) ? "--:--" : d.toLocaleTimeString([], {hour: '2-digit', minute:'2-digit'});
                } catch { return "--:--"; }
              })()}
            </span>
          </div>
        </div>
        <div className="flex items-center gap-2">
          <Badge variant={getStatusColor(selectedMission.status)} className="shrink-0 shadow-sm">
            {getStatusText(selectedMission.status)}
          </Badge>
          {phase && (
            <Badge variant={phase === "error" ? "destructive" : "outline"} className="shrink-0 flex items-center gap-1">
              <Activity className="h-3 w-3" />
              {phaseText[phase]}
            </Badge>
          )}
        </div>
      </div>
      
      <div className="grid grid-cols-2 gap-3">
        <div className={`
          flex items-center gap-2 p-2 rounded-lg border text-xs transition-colors
          ${selectedMission.scene 
            ? 'bg-primary/5 border-primary/20 text-primary' 
            : 'bg-muted/30 border-border text-muted-foreground'}
        `}>
          <div className={`p-1.5 rounded-md ${selectedMission.scene ? 'bg-primary/10' : 'bg-muted'}`}>
            <Map className="h-3.5 w-3.5" />
          </div>
          <div className="flex flex-col">
            <span className="font-medium">场景地图</span>
            <span className="text-[10px] opacity-80">{selectedMission.scene ? '已加载' : '未配置'}</span>
          </div>
        </div>

        <div className={`
          flex items-center gap-2 p-2 rounded-lg border text-xs transition-colors
          ${selectedMission.trajectory 
            ? 'bg-primary/5 border-primary/20 text-primary' 
            : 'bg-muted/30 border-border text-muted-foreground'}
        `}>
          <div className={`p-1.5 rounded-md ${selectedMission.trajectory ? 'bg-primary/10' : 'bg-muted'}`}>
            <Route className="h-3.5 w-3.5" />
          </div>
          <div className="flex flex-col">
            <span className="font-medium">飞行航线</span>
            <span className="text-[10px] opacity-80">{selectedMission.trajectory ? '已加载' : '未配置'}</span>
          </div>
        </div>
      </div>
    </div>
  );
}
