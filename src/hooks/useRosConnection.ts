"use client";

import { useCallback, useEffect, useRef, useState } from "react";
// @ts-expect-error - roslib 没有官方类型定义
import ROSLIB from "roslib";

export function useRosConnection(initialUrl: string = "") {
  const STORAGE_KEY = "pcd-viewer.ros-url";
  const [rosUrl, setRosUrlState] = useState(initialUrl);
  const [rosConnected, setRosConnected] = useState(false);
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const rosRef = useRef<typeof ROSLIB.Ros | null>(null);

  useEffect(() => {
    if (typeof window === "undefined") return;
    const saved = window.localStorage.getItem(STORAGE_KEY);
    if (saved) {
      setRosUrlState(saved);
    }
  }, [initialUrl]);

  const persistRosUrl = useCallback((value: string) => {
    setRosUrlState(value);
    if (typeof window !== "undefined") {
      window.localStorage.setItem(STORAGE_KEY, value);
    }
  }, []);

  useEffect(() => {
    if (typeof window === "undefined") return;
    window.localStorage.setItem(STORAGE_KEY, rosUrl);
  }, [rosUrl]);

  const connectROS = useCallback(() => {
    if (!rosUrl) {
      setConnectionError("请先输入正确的 ws 地址");
      return;
    }

    try {
      // 断开旧连接
      if (rosRef.current) {
        rosRef.current.close();
      }

      setConnectionError(null);
      const ros = new ROSLIB.Ros({
        url: rosUrl
      });

      ros.on('connection', () => {
        console.log('Connected to ROS');
        setRosConnected(true);
        setConnectionError(null);
      });

      ros.on('error', (error: unknown) => {
        console.error('ROS connection error:', error);
        setRosConnected(false);
        setConnectionError(`无法连接到 ROS (${rosUrl})`);
      });

      ros.on('close', () => {
        console.log('ROS connection closed');
        setRosConnected(false);
      });

      rosRef.current = ros;
    } catch (error) {
      console.error('Failed to connect to ROS:', error);
      setConnectionError('ROS 连接失败，请检查 ws 地址');
      rosRef.current = null;
    }
  }, [rosUrl]);

  const disconnectROS = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }
    setRosConnected(false);
    setConnectionError(null);
  }, []);

  return {
    rosUrl,
    setRosUrl: persistRosUrl,
    rosConnected,
    rosRef,
    connectROS,
    disconnectROS,
    connectionError,
  };
}
