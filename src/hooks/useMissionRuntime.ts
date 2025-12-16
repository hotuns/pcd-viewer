"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type {
  Mission,
  MissionHomePosition,
  MissionPhase,
  MissionRuntimeEvent,
  HangarChargeTelemetry,
  BatteryTelemetry,
  DronePosition,
} from "@/types/mission";
// @ts-expect-error - roslib 没有类型声明
import ROSLIB from "roslib";

const DEFAULT_ENDPOINTS = {
  missionCommandService: "/mission/command",
  missionListTopic: "/mission/list",
  missionStatusTopic: "/mission/status",
  waypointFeedbackTopic: "/mission/waypoint_feedback",
  controlTopic: "/mission/control",
  hangarChargeTopic: "/hangar/charge_status",
  batteryTopic: "/battery/status",
  poseTopic: "/odom_visualization/pose",
};

const CONTROL_CMDS = {
  TAKEOFF: 1,
  LAND: 2,
  EXECUTE: 3,
  RETURN_HOME: 4,
  ARM_OFF: 5,
};

type Endpoints = typeof DEFAULT_ENDPOINTS;

type UseMissionRuntimeParams = {
  rosConnected: boolean;
  rosRef: React.MutableRefObject<typeof ROSLIB.Ros | null>;
  mission: Mission | null;
  plannedPoints: Array<{ x: number; y: number; z: number }> | null;
  home: MissionHomePosition | null;
  options?: {
    droneId?: number;
    endpoints?: Partial<Endpoints>;
    lowBatteryThreshold?: number;
  };
};

function mapStatusToPhase(statusCode?: number): MissionPhase {
  switch (statusCode) {
    case 0:
      return "hangar_ready";
    case 1:
      return "mission_upload";
    case 2:
      return "takeoff_ready";
    case 3:
      return "takeoff";
    case 4:
      return "executing";
    case 5:
      return "returning";
    case 6:
      return "landing";
    case 7:
      return "charging";
    default:
      return "idle";
  }
}

function yawToQuaternion(yawRad: number) {
  return {
    x: 0,
    y: 0,
    z: Math.sin(yawRad / 2),
    w: Math.cos(yawRad / 2),
  };
}

function makeEvent(level: MissionRuntimeEvent["level"], message: string, details?: Record<string, unknown>): MissionRuntimeEvent {
  return {
    id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
    timestamp: new Date(),
    level,
    message,
    details,
  };
}

export function useMissionRuntime({
  rosConnected,
  rosRef,
  mission,
  plannedPoints,
  home,
  options,
}: UseMissionRuntimeParams) {
  const endpoints = useMemo(() => ({ ...DEFAULT_ENDPOINTS, ...(options?.endpoints ?? {}) }), [options?.endpoints]);
  const [phase, setPhase] = useState<MissionPhase>("idle");
  const [progress, setProgress] = useState<{ completed: number; total: number } | null>(null);
  const [hangar, setHangar] = useState<HangarChargeTelemetry | null>(null);
  const [battery, setBattery] = useState<BatteryTelemetry | null>(null);
  const [events, setEvents] = useState<MissionRuntimeEvent[]>([]);
  const [dronePosition, setDronePosition] = useState<DronePosition | null>(null);
  const [runtimeError, setRuntimeError] = useState<string | null>(null);
  const [busyAction, setBusyAction] = useState<string | null>(null);
  const [lastWaypoint, setLastWaypoint] = useState<{ index: number; info?: string; position?: { x: number; y: number; z: number } } | null>(null);
  const [autoReturnTriggered, setAutoReturnTriggered] = useState(false);
  const [pendingWaypoints, setPendingWaypoints] = useState<Array<{ x: number; y: number; z: number; task_type?: string; info?: string }>>([]);
  const pendingWaypointsRef = useRef<Array<{ x: number; y: number; z: number; task_type?: string; info?: string }>>([]);

  const missionCommandService = useRef<typeof ROSLIB.Service | null>(null);
  const missionListTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const controlTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const missionStatusTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const hangarTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const batteryTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const poseTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const waypointTopic = useRef<typeof ROSLIB.Topic | null>(null);

  const pushEvent = useCallback((level: MissionRuntimeEvent["level"], message: string, details?: Record<string, unknown>) => {
    setEvents((prev) => [makeEvent(level, message, details), ...prev].slice(0, 40));
  }, []);

  const resetRosBindings = useCallback(() => {
    missionCommandService.current = null;
    [missionListTopic, controlTopic, missionStatusTopic, hangarTopic, batteryTopic, poseTopic, waypointTopic].forEach((ref) => {
      ref.current?.unsubscribe?.();
      ref.current = null;
    });
  }, []);

  useEffect(() => {
    if (!rosConnected || !rosRef.current) {
      resetRosBindings();
      setPhase("idle");
      return;
    }

    const ros = rosRef.current;

    missionCommandService.current = new ROSLIB.Service({
      ros,
      name: endpoints.missionCommandService,
      serviceType: "mission_msgs/MissionCommand",
    });

    missionListTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.missionListTopic,
      messageType: "mission_msgs/MissionList",
    });

    controlTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.controlTopic,
      messageType: "mission_msgs/Control",
    });

    missionStatusTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.missionStatusTopic,
      messageType: "mission_msgs/MissionStatus",
    });

    missionStatusTopic.current.subscribe((msg: {
      status?: number;
      PosNum?: number;
      PosList?: Array<{ x: number; y: number; z: number; pass_type?: boolean; task_type?: string; info?: string }>;
    }) => {
      const nextPhase = mapStatusToPhase(msg.status);
      setPhase(nextPhase);
      if (Array.isArray(msg.PosList) && msg.PosList.length > 0) {
        const completed = msg.PosList.filter((p) => !!p.pass_type).length;
        setProgress({
          completed,
          total: msg.PosList.length,
        });
        let lastCompletedIndex = -1;
        msg.PosList.forEach((p, i) => {
          if (p.pass_type) lastCompletedIndex = i;
        });
        if (lastCompletedIndex >= 0) {
          const wp = msg.PosList[lastCompletedIndex];
          setLastWaypoint({ index: lastCompletedIndex, info: wp.info, position: { x: wp.x, y: wp.y, z: wp.z } });
        }
        const remaining = msg.PosList.filter((p) => !p.pass_type).map((p) => ({
          x: p.x,
          y: p.y,
          z: p.z,
          task_type: p.task_type,
          info: p.info,
        }));
        setPendingWaypoints(remaining);
        pendingWaypointsRef.current = remaining;
      } else {
        setPendingWaypoints([]);
        pendingWaypointsRef.current = [];
      }
      pushEvent("info", `任务状态更新为 ${nextPhase}`);
    });

    hangarTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.hangarChargeTopic,
      messageType: "mission_msgs/HangarChargeStatus",
    });

    hangarTopic.current.subscribe((msg: {
      charge_status: number;
      battery_percentage: number;
      charge_power: number;
      charge_duration: number;
      error_message?: string;
    }) => {
      setHangar({
        status: msg.charge_status,
        percentage: msg.battery_percentage,
        power: msg.charge_power,
        duration: msg.charge_duration,
        error: msg.error_message,
        updatedAt: new Date(),
      });
      if (msg.charge_status === 2) {
        setPhase("hangar_ready");
        pushEvent("info", "充电完成，可重新执行任务");
      } else if (msg.charge_status === 3) {
        setPhase("error");
        pushEvent("error", "机库充电错误", { message: msg.error_message });
      } else if (msg.charge_status === 1) {
        setPhase("charging");
      }
    });

    batteryTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.batteryTopic,
      messageType: "sensor_msgs/BatteryState",
    });

    batteryTopic.current.subscribe((msg: { voltage?: number; percentage?: number; charge?: number; capacity?: number; power_supply_status?: number }) => {
      setBattery({
        voltage: msg.voltage ?? undefined,
        percentage: msg.percentage != null ? msg.percentage * 100 : undefined,
        remaining: msg.charge ?? undefined,
        updatedAt: new Date(),
      });
    });

    poseTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.poseTopic,
      messageType: "geometry_msgs/PoseStamped",
    });

    poseTopic.current.subscribe((msg: {
      pose: {
        position: { x: number; y: number; z: number };
        orientation: { x: number; y: number; z: number; w: number };
      };
    }) => {
      setDronePosition({
        x: msg.pose.position.x,
        y: msg.pose.position.y,
        z: msg.pose.position.z,
        orientation: msg.pose.orientation,
      });
    });

    waypointTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.waypointFeedbackTopic,
      messageType: "mission_msgs/WaypointPosition",
    });

    waypointTopic.current.subscribe((msg: { x: number; y: number; z: number; pass_type?: boolean; task_type?: string; info?: string }) => {
      setLastWaypoint((prev) => ({
        index: (prev?.index ?? -1) + 1,
        info: msg.info,
        position: { x: msg.x, y: msg.y, z: msg.z },
      }));
      if (msg.pass_type) {
        pushEvent("info", `航点通过: ${msg.info ?? ""}`.trim());
      }
    });

    return () => {
      resetRosBindings();
    };
  }, [rosConnected, rosRef, endpoints, pushEvent, resetRosBindings]);

  const callMissionCommand = useCallback(async (command: number) => {
    if (!missionCommandService.current) throw new Error("MissionCommand 服务未就绪");
    setBusyAction("mission-command");
    return new Promise<void>((resolve, reject) => {
      missionCommandService.current!.callService(
        new ROSLIB.ServiceRequest({ command }),
        (result: { status?: string }) => {
          setBusyAction(null);
          pushEvent("info", result?.status || `MissionCommand ${command}`);
          resolve();
        },
        (error: unknown) => {
          setBusyAction(null);
          const reason = error instanceof Error ? error.message : String(error);
          pushEvent("error", "MissionCommand 调用失败", { reason });
          reject(error);
        }
      );
    });
  }, [pushEvent]);

  const publishControl = useCallback(async (cmd: number) => {
    if (!controlTopic.current) throw new Error("控制话题未连接");
    setBusyAction(`control-${cmd}`);
    try {
      controlTopic.current!.publish(new ROSLIB.Message({
        cmd,
        drone_id: options?.droneId ?? 1,
      }));
      pushEvent("info", `发送控制指令 ${cmd}`);
    } finally {
      setBusyAction(null);
    }
  }, [options?.droneId, pushEvent]);

  const clampInt32 = (value: number) => {
    if (!Number.isFinite(value)) return 0;
    const truncated = Math.trunc(value);
    const MAX = 2147483647;
    const MIN = -2147483648;
    return Math.min(Math.max(truncated, MIN), MAX);
  };

  const sendMissionList = useCallback(async (pointsToUpload: Array<{ x: number; y: number; z: number; task_type?: string; info?: string }>, label: string) => {
    if (!missionListTopic.current) throw new Error("MissionList 话题未连接");
    if (!mission) throw new Error("请先选择任务");
    if (!home) throw new Error("请先设置 HomePos");
    if (!pointsToUpload || pointsToUpload.length === 0) throw new Error("缺少航线");

    setBusyAction(label);
    const now = Date.now();
    const secs = Math.floor(now / 1000);
    const nsecs = Math.floor((now % 1000) * 1e6);
    const parsedId = Number.parseInt(mission.id, 10);
    const normalized = pointsToUpload.map((p, index) => ({
      x: p.x,
      y: p.y,
      z: p.z,
      task_type: p.task_type ?? "0",
      info: p.info ?? `wp-${index}`,
    }));
    try {
      const message = new ROSLIB.Message({
        id: clampInt32(Number.isFinite(parsedId) ? parsedId : 0),
        HomePos: {
          header: { stamp: { secs, nsecs }, frame_id: home.frameId },
          pose: {
            position: home.position,
            orientation: yawToQuaternion(home.yaw),
          },
        },
        PosNum: normalized.length,
        PosList: normalized.map((p) => ({
          x: p.x,
          y: p.y,
          z: p.z,
          pass_type: false,
          task_type: p.task_type,
          info: p.info,
        })),
      });
      missionListTopic.current.publish(message);
      setPendingWaypoints(normalized);
      pendingWaypointsRef.current = normalized;
      setPhase("mission_upload");
      pushEvent("info", `${label}`, { waypoints: normalized.length });
    } finally {
      setBusyAction(null);
    }
  }, [missionListTopic, mission, home, pushEvent]);

  const uploadMission = useCallback(async () => {
    if (!plannedPoints || plannedPoints.length === 0) throw new Error("缺少航线");
    await sendMissionList(plannedPoints, "任务列表已上传");
  }, [plannedPoints, sendMissionList]);

  const resumeMission = useCallback(async () => {
    const remaining = pendingWaypointsRef.current;
    if (!remaining || remaining.length === 0) {
      throw new Error("没有待执行的航点");
    }
    await sendMissionList(remaining, "续航任务已上传");
    pushEvent("info", "自动开始剩余航点");
    await publishControl(CONTROL_CMDS.EXECUTE);
  }, [publishControl, sendMissionList, pushEvent]);

  const handleTakeoff = useCallback(() => publishControl(CONTROL_CMDS.TAKEOFF), [publishControl]);
  const handleExecute = useCallback(() => publishControl(CONTROL_CMDS.EXECUTE), [publishControl]);
  const handleReturnHome = useCallback(() => publishControl(CONTROL_CMDS.RETURN_HOME), [publishControl]);
  const handleLand = useCallback(() => publishControl(CONTROL_CMDS.LAND), [publishControl]);
  const handleArmOff = useCallback(() => publishControl(CONTROL_CMDS.ARM_OFF), [publishControl]);
  const handleOpenHangar = useCallback(() => callMissionCommand(1), [callMissionCommand]);
  const handleCloseHangar = useCallback(() => callMissionCommand(2), [callMissionCommand]);

  useEffect(() => {
    if (!battery || battery.percentage == null) return;
    const threshold = options?.lowBatteryThreshold ?? 25;
    if (battery.percentage <= threshold && !autoReturnTriggered) {
      setAutoReturnTriggered(true);
      pushEvent("warning", `电量低（${battery.percentage.toFixed(1)}%），触发返航`);
      handleReturnHome().catch((error) => {
        setRuntimeError(error instanceof Error ? error.message : String(error));
      });
    }
    if (battery.percentage > threshold + 10 && autoReturnTriggered) {
      setAutoReturnTriggered(false);
    }
  }, [battery, options?.lowBatteryThreshold, autoReturnTriggered, handleReturnHome, pushEvent]);

  const resumeAvailable = useMemo(() => {
    return phase === "hangar_ready" && (hangar?.status === 2) && pendingWaypoints.length > 0;
  }, [phase, hangar?.status, pendingWaypoints.length]);

  return {
    phase,
    progress,
    hangar,
    battery,
    dronePosition,
    events,
    runtimeError,
    busyAction,
    lastWaypoint,
    pendingCount: pendingWaypoints.length,
    canResume: resumeAvailable,
    actions: {
      openHangar: handleOpenHangar,
      closeHangar: handleCloseHangar,
      uploadMission,
      executeMission: handleExecute,
      takeoff: handleTakeoff,
      returnHome: handleReturnHome,
      land: handleLand,
      armOff: handleArmOff,
      resumeMission,
    },
  };
}
