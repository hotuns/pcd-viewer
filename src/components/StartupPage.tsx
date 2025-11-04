"use client";

import { useState, useEffect } from "react";
import { Mission, MissionStatus } from "@/types/mission";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { PlusCircle, Trash2, Play, FileText, Map, Clock, CheckCircle2 } from "lucide-react";

// 状态显示辅助函数
const getStatusText = (status: MissionStatus): string => {
  const statusMap = {
    draft: '草稿',
    configured: '已配置',
    planning: '规划中',
    ready: '就绪',
    running: '执行中',
    paused: '暂停',
    completed: '完成',
    failed: '失败'
  };
  return statusMap[status];
};

const getStatusColor = (status: MissionStatus): string => {
  const colorMap = {
    draft: 'secondary',
    configured: 'default',
    planning: 'outline',
    ready: 'default',
    running: 'destructive',
    paused: 'secondary',
    completed: 'default',
    failed: 'destructive'
  };
  return colorMap[status];
};

const getStatusIcon = (status: MissionStatus) => {
  const iconMap = {
    draft: <FileText className="h-4 w-4" />,
    configured: <Map className="h-4 w-4" />,
    planning: <FileText className="h-4 w-4" />,
    ready: <Play className="h-4 w-4" />,
    running: <Clock className="h-4 w-4" />,
    paused: <Clock className="h-4 w-4" />,
    completed: <CheckCircle2 className="h-4 w-4" />,
    failed: <FileText className="h-4 w-4" />
  };
  return iconMap[status];
};

interface StartupPageProps {
  onMissionSelect: (mission: Mission) => void;
}

export default function StartupPage({ onMissionSelect }: StartupPageProps) {
  const [missions, setMissions] = useState<Mission[]>([
    {
      id: "demo_flight",
      name: "演示飞行任务",
      status: "ready",
      createdAt: new Date("2024-11-02"),
      scene: { type: "url", url: "/rosbag_play.pcd" },
      trajectory: { type: "url", url: "/example-planned-path.json" },
    },
    {
      id: "completed_mission",
      name: "已完成的巡检任务",
      status: "completed",
      createdAt: new Date("2024-10-28"),
      completedAt: new Date("2024-10-28T14:30:00"),
      scene: { type: "url", url: "/rosbag_play.pcd" },
      trajectory: { type: "url", url: "/example-planned-path.json" },
      waypoints: [
        { id: "wp1", x: 0, y: 0, z: 1, status: "completed", index: 0, completedAt: new Date() },
        { id: "wp2", x: 5, y: 0, z: 1, status: "completed", index: 1, completedAt: new Date() },
        { id: "wp3", x: 5, y: 5, z: 1, status: "completed", index: 2, completedAt: new Date() },
      ]
    }
  ]);
  
  const [newMissionName, setNewMissionName] = useState("");
  const [selectedMissionId, setSelectedMissionId] = useState<string | null>(null);

  // 从localStorage加载任务
  useEffect(() => {
    const savedMissions = localStorage.getItem('pcd-viewer-missions');
    if (savedMissions) {
      try {
        const parsed = JSON.parse(savedMissions);
        const missionsWithDates = parsed.map((m: Mission & { createdAt: string; completedAt?: string; startedAt?: string }) => ({
          ...m,
          createdAt: new Date(m.createdAt),
          completedAt: m.completedAt ? new Date(m.completedAt) : undefined,
          startedAt: m.startedAt ? new Date(m.startedAt) : undefined,
        }));
        setMissions(missionsWithDates);
      } catch (error) {
        console.error('Failed to load missions from localStorage:', error);
      }
    }
  }, []);

  // 保存任务到localStorage
  const saveMissions = (updatedMissions: Mission[]) => {
    setMissions(updatedMissions);
    localStorage.setItem('pcd-viewer-missions', JSON.stringify(updatedMissions));
  };

  const createMission = () => {
    if (!newMissionName.trim()) {
      alert("请输入任务名称");
      return;
    }
    
    const newMission: Mission = {
      id: Date.now().toString(),
      name: newMissionName,
      status: "draft",
      createdAt: new Date(),
    };
    
    saveMissions([...missions, newMission]);
    setNewMissionName("");
  };

  const deleteMission = (id: string) => {
    if (confirm("确定要删除这个任务吗？")) {
      saveMissions(missions.filter(m => m.id !== id));
      if (selectedMissionId === id) {
        setSelectedMissionId(null);
      }
    }
  };

  const selectMission = (mission: Mission) => {
    onMissionSelect(mission);
  };

  const formatDate = (date: Date) => {
    return date.toLocaleDateString('zh-CN', {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  };

  const getMissionDescription = (mission: Mission) => {
    const parts = [];
    
    if (mission.scene) parts.push("✓ 场景");
    else parts.push("✗ 场景");
    
    if (mission.trajectory) parts.push("✓ 航线");
    else parts.push("✗ 航线");
    
    if (mission.waypoints) {
      const completed = mission.waypoints.filter(w => w.status === 'completed').length;
      parts.push(`${completed}/${mission.waypoints.length} 完成`);
    }
    
    return parts.join(" | ");
  };

  // 按状态和创建时间排序
  const sortedMissions = [...missions].sort((a, b) => {
    // 优先级：running > ready > configured > planning > draft > paused > completed > failed
    const statusPriority = {
      running: 7,
      ready: 6, 
      configured: 5,
      planning: 4,
      draft: 3,
      paused: 2,
      completed: 1,
      failed: 0
    };
    
    const priorityDiff = statusPriority[b.status] - statusPriority[a.status];
    if (priorityDiff !== 0) return priorityDiff;
    
    // 相同状态按创建时间倒序
    return b.createdAt.getTime() - a.createdAt.getTime();
  });

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 p-6">
      <div className="max-w-6xl mx-auto">
        {/* 头部 */}
        <div className="text-center mb-8">
          <h1 className="text-4xl font-bold text-slate-800 mb-2">PCD Viewer</h1>
          <p className="text-slate-600 text-lg">无人机任务规划与执行系统</p>
        </div>

        {/* 创建新任务 */}
        <Card className="mb-6">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <PlusCircle className="h-5 w-5" />
              创建新任务
            </CardTitle>
            <CardDescription>
              创建一个新的无人机任务，设置场景和规划航线
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="flex gap-3">
              <div className="flex-1">
                <Input
                  placeholder="输入任务名称..."
                  value={newMissionName}
                  onChange={(e) => setNewMissionName(e.target.value)}
                  onKeyDown={(e) => e.key === "Enter" && createMission()}
                />
              </div>
              <Button onClick={createMission}>
                <PlusCircle className="h-4 w-4 mr-2" />
                创建任务
              </Button>
            </div>
          </CardContent>
        </Card>

        {/* 任务列表 */}
        <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-6">
          {sortedMissions.map(mission => (
            <Card 
              key={mission.id} 
              className={`cursor-pointer transition-all duration-200 hover:shadow-lg hover:scale-105 ${
                selectedMissionId === mission.id ? 'ring-2 ring-primary' : ''
              }`}
              onClick={() => setSelectedMissionId(mission.id)}
            >
              <CardHeader className="pb-3">
                <div className="flex items-start justify-between">
                  <div className="flex-1">
                    <CardTitle className="text-lg mb-2">{mission.name}</CardTitle>
                    <div className="flex items-center gap-2 mb-2">
                      {getStatusIcon(mission.status)}
                      <Badge variant={getStatusColor(mission.status) as "default" | "secondary" | "destructive" | "outline"}>
                        {getStatusText(mission.status)}
                      </Badge>
                    </div>
                  </div>
                  <Button
                    size="sm"
                    variant="ghost"
                    onClick={(e) => {
                      e.stopPropagation();
                      deleteMission(mission.id);
                    }}
                    className="text-slate-400 hover:text-red-500"
                  >
                    <Trash2 className="h-4 w-4" />
                  </Button>
                </div>
                <CardDescription>
                  {getMissionDescription(mission)}
                </CardDescription>
              </CardHeader>
              
              <CardContent className="pt-0">
                <div className="space-y-2 text-sm text-slate-600">
                  <div>创建: {formatDate(mission.createdAt)}</div>
                  {mission.startedAt && (
                    <div>开始: {formatDate(mission.startedAt)}</div>
                  )}
                  {mission.completedAt && (
                    <div>完成: {formatDate(mission.completedAt)}</div>
                  )}
                </div>
                
                <div className="mt-4 flex gap-2">
                  <Button 
                    size="sm" 
                    className="flex-1"
                    onClick={(e) => {
                      e.stopPropagation();
                      selectMission(mission);
                    }}
                  >
                    <Play className="h-4 w-4 mr-2" />
                    打开任务
                  </Button>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>

        {missions.length === 0 && (
          <Card className="text-center py-12">
            <CardContent>
              <div className="text-slate-400 mb-4">
                <FileText className="h-16 w-16 mx-auto mb-4" />
                <p className="text-lg">暂无任务</p>
                <p className="text-sm">创建你的第一个无人机任务开始使用</p>
              </div>
            </CardContent>
          </Card>
        )}

        {/* 底部信息 */}
        <div className="text-center mt-8 text-slate-500 text-sm">
          <p>选择一个任务进入主界面，或创建新任务开始规划</p>
        </div>
      </div>
    </div>
  );
}
