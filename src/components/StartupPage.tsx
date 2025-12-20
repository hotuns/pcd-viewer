"use client";

import { useState, useEffect, useCallback } from "react";
import { Mission, MissionStatus } from "@/types/mission";
import { useMissionDatabase } from "@/hooks/useMissionDatabase";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { PlusCircle, Trash2, Play, FileText, Map, Clock, CheckCircle2, RefreshCw, BarChart, ArrowRight, LayoutGrid, List } from "lucide-react";

// 状态显示辅助函数
const getStatusText = (status: MissionStatus): string => {
  const statusMap = {
    draft: '草稿',
    configured: '已配置',
    planning: '编辑中',
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
  const [statusFilter, setStatusFilter] = useState<"all" | MissionStatus>("all");
  const [viewMode, setViewMode] = useState<"card" | "list">("card");
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

  const filteredMissions = sortedMissions.filter((mission) =>
    statusFilter === "all" ? true : mission.status === statusFilter
  );

  const statusOptions: Array<{ value: "all" | MissionStatus; label: string }> = [
    { value: "all", label: "全部状态" },
    { value: "draft", label: "草稿" },
    { value: "configured", label: "已配置" },
    { value: "planning", label: "编辑中" },
    { value: "ready", label: "就绪" },
    { value: "running", label: "执行中" },
    { value: "paused", label: "暂停" },
    { value: "completed", label: "完成" },
    { value: "failed", label: "失败" },
  ];

  return (
    <div className="min-h-screen bg-background text-foreground p-4 md:p-8">
      <div className="max-w-6xl mx-auto space-y-6">
        <div className="flex flex-wrap items-center justify-between gap-3">
          <div>
            <div className="text-sm font-medium text-muted-foreground">任务概览</div>
            <div className="text-xs text-muted-foreground/70">
              {missions.length} 个任务 · {stats?.running ?? 0} 正在执行
            </div>
          </div>
          <div className="flex items-center gap-2">
            <Button onClick={loadData} variant="outline" size="sm" disabled={loading}>
              <RefreshCw className={`h-3.5 w-3.5 mr-2 ${loading ? "animate-spin" : ""}`} />
              刷新
            </Button>
          </div>
        </div>


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
                  <p>上传或在 3D 视图中编辑飞行航线</p>
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
            <div className="flex flex-wrap items-center gap-3">
              <h2 className="text-lg font-semibold tracking-tight">任务列表</h2>
              <div className="text-xs text-muted-foreground">
                显示 {filteredMissions.length} / {missions.length} 个任务
              </div>
              <div className="flex items-center gap-2 ml-auto">
                <select
                  value={statusFilter}
                  onChange={(e) => setStatusFilter(e.target.value as "all" | MissionStatus)}
                  className="h-8 rounded-md border bg-background px-2 text-sm"
                >
                  {statusOptions.map((option) => (
                    <option key={option.value} value={option.value}>
                      {option.label}
                    </option>
                  ))}
                </select>
                <div className="inline-flex rounded-md border bg-background p-0.5">
                  <Button
                    type="button"
                    size="sm"
                    variant={viewMode === "card" ? "secondary" : "ghost"}
                    className="h-8 w-9 p-0"
                    onClick={() => setViewMode("card")}
                  >
                    <LayoutGrid className="h-4 w-4" />
                  </Button>
                  <Button
                    type="button"
                    size="sm"
                    variant={viewMode === "list" ? "secondary" : "ghost"}
                    className="h-8 w-9 p-0"
                    onClick={() => setViewMode("list")}
                  >
                    <List className="h-4 w-4" />
                  </Button>
                </div>
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

            {viewMode === "card" ? (
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                {filteredMissions.map((mission) => (
                  <Card
                    key={mission.id}
                    className={`group cursor-pointer transition-all duration-300 hover:shadow-lg hover:-translate-y-1 border-border/50 hover:border-primary/50 ${
                      selectedMissionId === mission.id ? "ring-2 ring-primary shadow-lg" : ""
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
                            <Badge
                              variant={getStatusColor(mission.status as MissionStatus) as "default" | "secondary" | "destructive" | "outline"}
                              className="h-5 px-1.5 gap-1 font-normal"
                            >
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
                        variant="secondary"
                        className="w-full group-hover:bg-primary group-hover:text-primary-foreground transition-colors"
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
            ) : (
              <div className="rounded-lg border bg-card/70">
                <div className="grid grid-cols-[auto_auto_1fr_auto_auto] gap-3 px-4 py-3 text-xs font-medium text-muted-foreground uppercase tracking-wide">
                  <span>名称</span>
                  <span>状态</span>
                  <span className="hidden sm:block">配置</span>
                  <span>创建时间</span>
                  <span className="text-right">操作</span>
                </div>
                <div className="divide-y divide-border/60 text-sm">
                  {filteredMissions.map((mission) => (
                    <div
                      key={mission.id}
                      className={`grid grid-cols-[auto_auto_1fr_auto_auto] gap-3 px-4 py-3 items-center ${
                        selectedMissionId === mission.id ? "bg-primary/5" : ""
                      }`}
                    >
                      <button
                        type="button"
                        onClick={() => setSelectedMissionId(mission.id)}
                        className="text-left font-medium truncate max-w-[140px] hover:text-primary"
                      >
                        {mission.name}
                      </button>
                      <Badge
                        variant={getStatusColor(mission.status as MissionStatus) as "default" | "secondary" | "destructive" | "outline"}
                        className="justify-center"
                      >
                        {getStatusText(mission.status as MissionStatus)}
                      </Badge>
                      <div className="hidden sm:block">{getMissionDescription(mission)}</div>
                      <div className="text-xs text-muted-foreground">
                        {formatDate(new Date(mission.createdAt))}
                      </div>
                      <div className="flex items-center justify-end gap-2">
                        <Button
                          size="sm"
                          variant="secondary"
                          className="h-8 px-3 text-xs"
                          onClick={() => selectMission(mission)}
                        >
                          打开
                        </Button>
                        <Button
                          size="icon"
                          variant="ghost"
                          className="h-8 w-8 text-muted-foreground hover:text-destructive"
                          onClick={() => handleDeleteMission(mission.id)}
                          disabled={loading}
                        >
                          <Trash2 className="h-4 w-4" />
                        </Button>
                      </div>
                    </div>
                  ))}
                  {filteredMissions.length === 0 && (
                    <div className="px-4 py-6 text-center text-sm text-muted-foreground">
                      当前筛选条件下没有任务
                    </div>
                  )}
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
