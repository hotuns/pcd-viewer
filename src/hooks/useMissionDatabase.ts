"use client";

import { useState, useCallback } from 'react';
import type { Mission } from '@/types/mission';

/**
 * 客户端任务数据库操作 Hook
 */
export function useMissionDatabase() {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * 保存任务到数据库
   */
  const saveMission = useCallback(async (mission: Mission) => {
    setLoading(true);
    setError(null);
    
    try {
      // 将 File 对象转换为 URL（如果需要持久化，需要额外处理）
      const missionData = {
        ...mission,
        scene: mission.scene?.type === 'url' ? mission.scene : undefined,
        trajectory: mission.trajectory?.type === 'url' ? mission.trajectory : undefined,
      };

      const response = await fetch('/api/missions', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(missionData),
      });

      if (!response.ok) {
        throw new Error('Failed to save mission');
      }

      return true;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      return false;
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * 更新任务
   */
  const updateMission = useCallback(async (id: string, updates: Partial<Mission>) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await fetch(`/api/missions/${id}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(updates),
      });

      if (!response.ok) {
        throw new Error('Failed to update mission');
      }

      return true;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      return false;
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * 删除任务
   */
  const deleteMission = useCallback(async (id: string) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await fetch(`/api/missions/${id}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        throw new Error('Failed to delete mission');
      }

      return true;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      return false;
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * 获取任务
   */
  const getMission = useCallback(async (id: string) => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await fetch(`/api/missions/${id}`);

      if (!response.ok) {
        throw new Error('Failed to fetch mission');
      }

      const data = await response.json();
      return data.mission;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      return null;
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * 获取所有任务
   */
  const getAllMissions = useCallback(async () => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await fetch('/api/missions');

      if (!response.ok) {
        throw new Error('Failed to fetch missions');
      }

      const data = await response.json();
      return data.missions;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      return [];
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * 获取任务统计
   */
  const getMissionStats = useCallback(async () => {
    setLoading(true);
    setError(null);
    
    try {
      const response = await fetch('/api/missions?stats=true');

      if (!response.ok) {
        throw new Error('Failed to fetch mission stats');
      }

      const data = await response.json();
      return data.stats;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      return {};
    } finally {
      setLoading(false);
    }
  }, []);

  return {
    loading,
    error,
    saveMission,
    updateMission,
    deleteMission,
    getMission,
    getAllMissions,
    getMissionStats,
  };
}
