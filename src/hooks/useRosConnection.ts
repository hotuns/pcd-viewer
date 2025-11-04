"use client";

import { useCallback, useRef, useState } from "react";
// @ts-expect-error - roslib 没有官方类型定义
import ROSLIB from "roslib";

export function useRosConnection(initialUrl: string = "ws://192.168.203.30:9999") {
  const [rosUrl, setRosUrl] = useState(initialUrl);
  const [rosConnected, setRosConnected] = useState(false);
  const rosRef = useRef<typeof ROSLIB.Ros | null>(null);

  const connectROS = useCallback(() => {
    if (!rosUrl) return;
    
    try {
      // 断开旧连接
      if (rosRef.current) {
        rosRef.current.close();
      }

      const ros = new ROSLIB.Ros({
        url: rosUrl
      });

      ros.on('connection', () => {
        console.log('Connected to ROS');
        setRosConnected(true);
      });

      ros.on('error', (error: unknown) => {
        console.error('ROS connection error:', error);
        setRosConnected(false);
      });

      ros.on('close', () => {
        console.log('ROS connection closed');
        setRosConnected(false);
      });

      rosRef.current = ros;
    } catch (error) {
      console.error('Failed to connect to ROS:', error);
      throw new Error('ROS 连接失败');
    }
  }, [rosUrl]);

  const disconnectROS = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }
    setRosConnected(false);
  }, []);

  return {
    rosUrl,
    setRosUrl,
    rosConnected,
    rosRef,
    connectROS,
    disconnectROS,
  };
}
