"use client";

import { useState, useEffect, useCallback } from "react";
import { Mission, MissionStatus } from "@/types/mission";
import { useMissionDatabase } from "@/hooks/useMissionDatabase";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { PlusCircle, Trash2, Play, FileText, Map, Clock, CheckCircle2, RefreshCw, BarChart, ArrowRight } from "lucide-react";

const getStatusText = (status: MissionStatus): string => {
  const statusMap = {
    draft: "草稿",
    configured: "已配置",
    planning: "编辑中",
    ready: "就绪",
    running: "执行中",
    paused: "暂停",
    completed: "完成",
    failed: "失败",
  };
  return statusMap[status];
};

const getStatusColor = (status: MissionStatus): string => {
  const colorMap = {
    draft: "secondary",
    configured: "secondary",
    planning: "outline",
    ready: "default",
    running: "destructive",
    paused: "secondary",
    completed: "default",
    failed: "destructive",
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
    failed: <FileText className="h-3.5 w-3.5" />,
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
  const { getAllMissions, getMissionStats, saveMission, deleteMission, loading, error } = useMissionDatabase();

  const loadData = useCallback(async () => {
    try {
      const [missionsData, statsData] = await Promise.all([getAllMissions(), getMissionStats()]);
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
    setSelectedMissionId(missionRow.id);
    const mission: Mission = {
      id: missionRow.id,
      name: missionRow.name,
      status: missionRow.status,
      createdAt: (() => {
        const parsed = new Date(missionRow.createdAt);
        return isNaN(parsed.getTime()) ? new Date() : parsed;
      })(),
      startedAt: missionRow.startedAt
        ? (() => {
            const parsed = new Date(missionRow.startedAt!);
            return isNaN(parsed.getTime()) ? undefined : parsed;
          })()
        : undefined,
      completedAt: missionRow.completedAt
        ? (() => {
            const parsed = new Date(missionRow.completedAt!);
            return isNaN(parsed.getTime()) ? undefined : parsed;
          })()
        : undefined,
      scene: missionRow.sceneUrl ? { type: "url", url: missionRow.sceneUrl } : undefined,
      trajectory: missionRow.trajectoryUrl ? { type: "url", url: missionRow.trajectoryUrl } : undefined,
    };
    onMissionSelect(mission);
  };

  const formatDate = (date: Date) => {
    if (isNaN(date.getTime())) return "未知日期";
    return date.toLocaleDateString("zh-CN", {
      year: "numeric",
      month: "short",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    });
  };

  const getMissionDescription = (mission: MissionRow) => {
    return (
      <div className="flex gap-2 text-[11px] text-slate-400">
        <span
          className={`px-1.5 py-0.5 rounded border ${
            mission.sceneUrl ? "bg-emerald-500/10 border-emerald-500/30 text-emerald-400" : "bg-slate-900 border-slate-700"
          }`}
        >
          {mission.sceneUrl ? "✓ 场景" : "✗ 场景"}
        </span>
        <span
          className={`px-1.5 py-0.5 rounded border ${
            mission.trajectoryUrl ? "bg-emerald-500/10 border-emerald-500/30 text-emerald-400" : "bg-slate-900 border-slate-700"
          }`}
        >
          {mission.trajectoryUrl ? "✓ 航线" : "✗ 航线"}
        </span>
      </div>
    );
  };

  const sortedMissions = [...missions].sort((a, b) => {
    const priority: Record<string, number> = {
      running: 7,
      ready: 6,
      configured: 5,
      planning: 4,
      draft: 3,
      paused: 2,
      completed: 1,
      failed: 0,
    };
    const diff = (priority[b.status] ?? 0) - (priority[a.status] ?? 0);
    if (diff !== 0) return diff;
    return new Date(b.createdAt).getTime() - new Date(a.createdAt).getTime();
  });

  const filteredMissions = sortedMissions.filter((mission) => (statusFilter === "all" ? true : mission.status === statusFilter));

  const statusOptions: Array<{ value: "all" | MissionStatus; label: string }> = [
    { value: "all", label: "全部" },
    { value: "running", label: "执行中" },
    { value: "ready", label: "就绪" },
    { value: "planning", label: "编辑中" },
    { value: "completed", label: "完成" },
  ];

  const statsSummary = [
    { label: "任务总数", value: stats?.total ?? 0 },
    { label: "就绪", value: stats?.ready ?? 0 },
    { label: "执行中", value: stats?.running ?? 0 },
    { label: "已完成", value: stats?.completed ?? 0 },
  ];

  return (
    <div className="flex h-full w-full bg-slate-950 text-slate-100">
      <aside className="w-72 border-r border-slate-800 p-4 flex flex-col gap-4">
        <div>
          <div className="text-[11px] uppercase text-slate-400 tracking-widest">Missionlogic</div>
          <div className="flex items-center gap-2 text-sm text-slate-500">
            <BarChart className="h-4 w-4" />
            {stats?.running ?? 0} 个任务执行中
          </div>
          <div className="text-xs text-slate-500">总计 {stats?.total ?? 0} 个任务</div>
        </div>

        <div className="grid grid-cols-2 gap-2">
          {statsSummary.map((item) => (
            <div key={item.label} className="bg-slate-900/60 border border-slate-800 rounded p-3">
              <div className="text-[11px] text-slate-400">{item.label}</div>
              <div className="text-xl font-semibold">{item.value}</div>
            </div>
          ))}
        </div>

        <div className="space-y-2">
          <div className="text-[11px] uppercase text-slate-400 tracking-widest">新建任务</div>
          <div className="flex gap-2">
            <Input
              placeholder="输入名称"
              value={newMissionName}
              onChange={(e) => setNewMissionName(e.target.value)}
              className="bg-slate-900 border-slate-700 text-slate-100"
            />
            <Tooltip>
              <TooltipTrigger asChild>
                <Button size="icon" onClick={createMission}>
                  <PlusCircle className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>创建任务</TooltipContent>
            </Tooltip>
          </div>
        </div>

        <div className="space-y-2">
          <div className="text-[11px] uppercase text-slate-400 tracking-widest">状态筛选</div>
          <div className="flex flex-wrap gap-2">
            {statusOptions.map((option) => (
              <Button
                key={option.value}
                variant={statusFilter === option.value ? "default" : "outline"}
                size="sm"
                className="text-xs"
                onClick={() => setStatusFilter(option.value)}
              >
                {option.label}
              </Button>
            ))}
          </div>
        </div>

        {error && (
          <div className="text-xs text-red-400 bg-red-500/10 border border-red-500/30 px-3 py-2 rounded">
            {error}
          </div>
        )}
      </aside>

      <section className="flex-1 flex flex-col">
        <div className="flex items-center justify-between border-b border-slate-800 px-4 py-3 bg-slate-900/70">
          <div>
            <div className="text-sm font-semibold">任务列表</div>
            <div className="text-[11px] text-slate-400">{filteredMissions.length} 个匹配任务</div>
          </div>
          <div className="flex items-center gap-2">
            <Tooltip>
              <TooltipTrigger asChild>
                <Button variant="outline" size="icon" onClick={loadData} disabled={loading}>
                  <RefreshCw className={`h-4 w-4 ${loading ? "animate-spin" : ""}`} />
                </Button>
              </TooltipTrigger>
              <TooltipContent>刷新列表</TooltipContent>
            </Tooltip>
          </div>
        </div>

        <div className="flex-1 overflow-y-auto p-4 space-y-3">
          {filteredMissions.length === 0 ? (
            <div className="flex flex-col items-center justify-center h-full text-slate-500 border border-dashed border-slate-700 rounded py-16">
              <Map className="h-10 w-10 opacity-40 mb-3" />
              <p className="text-sm">暂无符合条件的任务</p>
              <p className="text-xs text-slate-500">在左侧创建或调整筛选条件</p>
            </div>
          ) : (
            filteredMissions.map((mission) => (
              <div
                key={mission.id}
                className={`border rounded px-4 py-3 bg-slate-900/60 border-slate-800 flex items-center justify-between gap-3 ${
                  selectedMissionId === mission.id ? "border-emerald-500/70 bg-slate-900/80" : ""
                }`}
              >
                <div>
                  <div className="text-sm font-semibold">{mission.name}</div>
                  <div className="text-[11px] text-slate-500">创建于 {formatDate(new Date(mission.createdAt))}</div>
                  <div className="mt-2">{getMissionDescription(mission)}</div>
                </div>
                <div className="flex items-center gap-2">
                  <Badge variant={getStatusColor(mission.status)} className="flex items-center gap-1 text-[11px]">
                    {getStatusIcon(mission.status)}
                    {getStatusText(mission.status)}
                  </Badge>
                  <Tooltip>
                    <TooltipTrigger asChild>
                      <Button variant="outline" size="icon" onClick={() => handleDeleteMission(mission.id)}>
                        <Trash2 className="h-4 w-4" />
                      </Button>
                    </TooltipTrigger>
                    <TooltipContent>删除任务</TooltipContent>
                  </Tooltip>
                  <Tooltip>
                    <TooltipTrigger asChild>
                      <Button size="icon" onClick={() => selectMission(mission)}>
                        <ArrowRight className="h-4 w-4" />
                      </Button>
                    </TooltipTrigger>
                    <TooltipContent>进入任务</TooltipContent>
                  </Tooltip>
                </div>
              </div>
            ))
          )}
        </div>
      </section>
    </div>
  );
}
