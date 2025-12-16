"use client";

import { useState, useEffect, useCallback } from "react";
import { Mission, MissionStatus } from "@/types/mission";
import { useMissionDatabase } from "@/hooks/useMissionDatabase";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { PlusCircle, Trash2, Play, FileText, Map, Clock, CheckCircle2, RefreshCw, Database, BarChart, ArrowRight, LayoutGrid } from "lucide-react";

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
    configured: 'secondary',
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
    draft: <FileText className="h-3.5 w-3.5" />,
    configured: <Map className="h-3.5 w-3.5" />,
    planning: <FileText className="h-3.5 w-3.5" />,
    ready: <Play className="h-3.5 w-3.5" />,
    running: <Clock className="h-3.5 w-3.5" />,
    paused: <Clock className="h-3.5 w-3.5" />,
    completed: <CheckCircle2 className="h-3.5 w-3.5" />,
    failed: <FileText className="h-3.5 w-3.5" />
  };
  return iconMap[status];
};

interface MissionRow {
  id: string;
  name: string;
  status: MissionStatus;
  sceneUrl?: string;
  trajectoryUrl?: string;
  createdAt: string;
  startedAt?: string | null;
  completedAt?: string | null;
}

interface StatsRow {
  total: number;
  draft: number;
  configured: number;
  planning: number;
  ready: number;
  running: number;
  paused: number;
  completed: number;
  failed: number;
}

interface StartupPageProps {
  onMissionSelect: (mission: Mission) => void;
}

export default function StartupPage({ onMissionSelect }: StartupPageProps) {
  const [missions, setMissions] = useState<MissionRow[]>([]);
  const [stats, setStats] = useState<StatsRow | null>(null);
  const [newMissionName, setNewMissionName] = useState("");
  const [selectedMissionId, setSelectedMissionId] = useState<string | null>(null);
  const { getAllMissions, getMissionStats, saveMission, deleteMission, loading, error } = useMissionDatabase();

  // 加载数据
  const loadData = useCallback(async () => {
    try {
      const [missionsData, statsData] = await Promise.all([
        getAllMissions(),
        getMissionStats()
      ]);
      setMissions(missionsData as MissionRow[]);
      setStats(statsData as StatsRow);
    } catch (err) {
      console.error("Failed to load missions:", err);
    }
  }, [getAllMissions, getMissionStats]);

  useEffect(() => {
    loadData();
  }, [loadData]);

  const createMission = async () => {
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
    
    await saveMission(newMission);
    setNewMissionName("");
    await loadData();
  };

  const handleDeleteMission = async (id: string) => {
    if (confirm("确定要删除这个任务吗？")) {
      await deleteMission(id);
      if (selectedMissionId === id) {
        setSelectedMissionId(null);
      }
      await loadData();
    }
  };

  const selectMission = (missionRow: MissionRow) => {
    // 转换数据库格式为 Mission 格式
    const mission: Mission = {
      id: missionRow.id,
      name: missionRow.name,
      status: missionRow.status,
      createdAt: (() => {
        const parsed = new Date(missionRow.createdAt);
        return isNaN(parsed.getTime()) ? new Date() : parsed;
      })(),
      startedAt: missionRow.startedAt ? (() => {
        const parsed = new Date(missionRow.startedAt!);
        return isNaN(parsed.getTime()) ? undefined : parsed;
      })() : undefined,
      completedAt: missionRow.completedAt ? (() => {
        const parsed = new Date(missionRow.completedAt!);
        return isNaN(parsed.getTime()) ? undefined : parsed;
      })() : undefined,
      scene: missionRow.sceneUrl ? { type: "url", url: missionRow.sceneUrl } : undefined,
      trajectory: missionRow.trajectoryUrl ? { type: "url", url: missionRow.trajectoryUrl } : undefined,
    };
    onMissionSelect(mission);
  };

  const formatDate = (date: Date) => {
    if (isNaN(date.getTime())) return "未知日期";
    return date.toLocaleDateString('zh-CN', {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  };

  const getMissionDescription = (mission: MissionRow) => {
    return (
      <div className="flex gap-2 text-xs">
        <span className={`px-1.5 py-0.5 rounded border ${mission.sceneUrl ? 'bg-green-500/10 border-green-500/20 text-green-600' : 'bg-muted border-border text-muted-foreground'}`}>
          {mission.sceneUrl ? '✓ 场景' : '✗ 场景'}
        </span>
        <span className={`px-1.5 py-0.5 rounded border ${mission.trajectoryUrl ? 'bg-green-500/10 border-green-500/20 text-green-600' : 'bg-muted border-border text-muted-foreground'}`}>
          {mission.trajectoryUrl ? '✓ 航线' : '✗ 航线'}
        </span>
      </div>
    );
  };

  // 按状态和创建时间排序
  const sortedMissions = [...missions].sort((a, b) => {
    // 优先级：running > ready > configured > planning > draft > paused > completed > failed
    const statusPriority: Record<string, number> = {
      running: 7,
      ready: 6, 
      configured: 5,
      planning: 4,
      draft: 3,
      paused: 2,
      completed: 1,
      failed: 0
    };
    
    const priorityDiff = (statusPriority[b.status] || 0) - (statusPriority[a.status] || 0);
    if (priorityDiff !== 0) return priorityDiff;
    
    // 相同状态按创建时间倒序
    return new Date(b.createdAt).getTime() - new Date(a.createdAt).getTime();
  });

  return (
    <div className="min-h-screen bg-background text-foreground p-6 md:p-12">
      <div className="max-w-7xl mx-auto space-y-8">
        {/* 头部 */}
        <div className="flex flex-col md:flex-row md:items-center justify-between gap-4 animate-in slide-in-from-top-4 duration-500">
          <div className="space-y-1">
            <h1 className="text-4xl font-bold tracking-tight bg-gradient-to-r from-primary to-primary/60 bg-clip-text text-transparent">
              PCD Viewer
            </h1>
            <p className="text-muted-foreground text-lg">无人机任务规划与执行系统</p>
          </div>
          <Button onClick={loadData} variant="outline" disabled={loading} className="shadow-sm">
            <RefreshCw className={`h-4 w-4 mr-2 ${loading ? 'animate-spin' : ''}`} />
            刷新数据
          </Button>
        </div>

        {/* 统计面板 */}
        {stats && (
          <div className="grid grid-cols-2 sm:grid-cols-3 md:grid-cols-5 lg:grid-cols-9 gap-3 animate-in fade-in duration-700 delay-100">
            <Card className="bg-primary/5 border-primary/10 shadow-sm hover:shadow-md transition-all">
              <CardContent className="p-4 flex flex-col items-center justify-center text-center">
                <div className="text-xs font-medium text-muted-foreground mb-1 flex items-center gap-1">
                  <Database className="h-3 w-3" /> 总计
                </div>
                <div className="text-2xl font-bold text-primary">{stats.total}</div>
              </CardContent>
            </Card>
            
            {[
              { label: '草稿', value: stats.draft, color: 'text-muted-foreground' },
              { label: '已配置', value: stats.configured, color: 'text-blue-600' },
              { label: '规划中', value: stats.planning, color: 'text-yellow-600' },
              { label: '就绪', value: stats.ready, color: 'text-green-600' },
              { label: '执行中', value: stats.running, color: 'text-purple-600' },
              { label: '暂停', value: stats.paused, color: 'text-orange-600' },
              { label: '完成', value: stats.completed, color: 'text-teal-600' },
              { label: '失败', value: stats.failed, color: 'text-red-600' },
            ].map((item) => (
              <Card key={item.label} className="shadow-sm hover:shadow-md transition-all">
                <CardContent className="p-4 flex flex-col items-center justify-center text-center">
                  <div className="text-xs font-medium text-muted-foreground mb-1">{item.label}</div>
                  <div className={`text-xl font-bold ${item.color}`}>{item.value}</div>
                </CardContent>
              </Card>
            ))}
          </div>
        )}

        {/* 错误提示 */}
        {error && (
          <div className="p-4 rounded-lg bg-destructive/10 border border-destructive/20 text-destructive flex items-center gap-2 animate-in shake">
            <div className="h-2 w-2 rounded-full bg-destructive animate-pulse" />
            错误: {error}
          </div>
        )}

        <div className="grid lg:grid-cols-[350px_1fr] gap-8 animate-in slide-in-from-bottom-8 duration-700 delay-200">
          {/* 左侧：创建新任务 */}
          <div className="space-y-6">
            <Card className="border-primary/20 shadow-lg shadow-primary/5 overflow-hidden relative">
              <div className="absolute top-0 left-0 w-full h-1 bg-gradient-to-r from-primary to-primary/20" />
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  <PlusCircle className="h-5 w-5 text-primary" />
                  创建新任务
                </CardTitle>
                <CardDescription>
                  开始一个新的无人机飞行任务
                </CardDescription>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="space-y-2">
                  <Input
                    placeholder="输入任务名称..."
                    value={newMissionName}
                    onChange={(e) => setNewMissionName(e.target.value)}
                    onKeyDown={(e) => e.key === "Enter" && createMission()}
                    disabled={loading}
                    className="h-11"
                  />
                </div>
                <Button onClick={createMission} disabled={loading} className="w-full h-11 shadow-lg shadow-primary/20">
                  <PlusCircle className="h-4 w-4 mr-2" />
                  立即创建
                </Button>
              </CardContent>
            </Card>

            <div className="p-6 rounded-xl bg-muted/30 border border-border/50 space-y-4">
              <h3 className="font-semibold flex items-center gap-2">
                <LayoutGrid className="h-4 w-4" />
                快速指南
              </h3>
              <div className="space-y-3 text-sm text-muted-foreground">
                <div className="flex gap-3">
                  <div className="flex h-6 w-6 shrink-0 items-center justify-center rounded-full border bg-background text-xs font-medium shadow-sm">1</div>
                  <p>创建任务并上传场景点云文件 (.pcd)</p>
                </div>
                <div className="flex gap-3">
                  <div className="flex h-6 w-6 shrink-0 items-center justify-center rounded-full border bg-background text-xs font-medium shadow-sm">2</div>
                  <p>上传或在 3D 视图中规划飞行航线</p>
                </div>
                <div className="flex gap-3">
                  <div className="flex h-6 w-6 shrink-0 items-center justify-center rounded-full border bg-background text-xs font-medium shadow-sm">3</div>
                  <p>连接 ROS 并开始执行自动化任务</p>
                </div>
              </div>
            </div>
          </div>

          {/* 右侧：任务列表 */}
          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <h2 className="text-xl font-semibold tracking-tight">最近任务</h2>
              <div className="text-sm text-muted-foreground">
                共 {missions.length} 个任务
              </div>
            </div>

            {/* 加载状态 */}
            {loading && missions.length === 0 && (
              <div className="text-center py-12 border rounded-xl border-dashed">
                <RefreshCw className="h-8 w-8 animate-spin mx-auto mb-4 text-muted-foreground" />
                <p className="text-muted-foreground">加载中...</p>
              </div>
            )}

            {/* 空状态 */}
            {!loading && missions.length === 0 && (
              <div className="text-center py-16 border rounded-xl border-dashed bg-muted/10">
                <BarChart className="h-12 w-12 mx-auto mb-4 text-muted-foreground/50" />
                <h3 className="text-lg font-medium mb-1">暂无任务</h3>
                <p className="text-muted-foreground">创建你的第一个无人机任务开始使用</p>
              </div>
            )}

            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              {sortedMissions.map(mission => (
                <Card 
                  key={mission.id} 
                  className={`group cursor-pointer transition-all duration-300 hover:shadow-lg hover:-translate-y-1 border-border/50 hover:border-primary/50 ${
                    selectedMissionId === mission.id ? 'ring-2 ring-primary shadow-lg' : ''
                  }`}
                  onClick={() => setSelectedMissionId(mission.id)}
                >
                  <CardHeader className="pb-3 space-y-2">
                    <div className="flex items-start justify-between gap-2">
                      <div className="space-y-1 min-w-0">
                        <CardTitle className="text-base truncate leading-tight group-hover:text-primary transition-colors">
                          {mission.name}
                        </CardTitle>
                        <div className="flex items-center gap-2">
                          <Badge variant={getStatusColor(mission.status as MissionStatus) as "default" | "secondary" | "destructive" | "outline"} className="h-5 px-1.5 gap-1 font-normal">
                            {getStatusIcon(mission.status as MissionStatus)}
                            {getStatusText(mission.status as MissionStatus)}
                          </Badge>
                        </div>
                      </div>
                      <Button
                        size="icon"
                        variant="ghost"
                        onClick={(e) => {
                          e.stopPropagation();
                          handleDeleteMission(mission.id);
                        }}
                        className="h-8 w-8 text-muted-foreground hover:text-destructive hover:bg-destructive/10 opacity-0 group-hover:opacity-100 transition-all"
                        disabled={loading}
                      >
                        <Trash2 className="h-4 w-4" />
                      </Button>
                    </div>
                  </CardHeader>
                  
                  <CardContent>
                    <div className="flex items-center justify-between mb-4">
                      {getMissionDescription(mission)}
                    </div>
                    
                    <div className="space-y-1 text-xs text-muted-foreground mb-4">
                      <div className="flex items-center gap-1.5">
                        <Clock className="h-3 w-3" />
                        创建: {formatDate(new Date(mission.createdAt))}
                      </div>
                      {mission.completedAt && (
                        <div className="flex items-center gap-1.5 text-green-600">
                          <CheckCircle2 className="h-3 w-3" />
                          完成: {formatDate(new Date(mission.completedAt))}
                        </div>
                      )}
                    </div>
                    
                    <Button 
                      size="sm" 
                      className="w-full group-hover:bg-primary group-hover:text-primary-foreground transition-colors"
                      variant="secondary"
                      onClick={(e) => {
                        e.stopPropagation();
                        selectMission(mission);
                      }}
                    >
                      打开任务 <ArrowRight className="h-3.5 w-3.5 ml-2 group-hover:translate-x-1 transition-transform" />
                    </Button>
                  </CardContent>
                </Card>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
