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
  PlannedPoint,
  TaskType,
} from "@/types/mission";
import type { NestAutoModeTelemetry, NestBatteryTelemetry, NestMotorTelemetry, NestStatusTelemetry, NestTelemetry, NestUavPresenceTelemetry } from "@/types/nest";
import { normalizeTaskType, serializeTaskType } from "@/lib/taskTypes";
import { createGridMapDataFromPointCloud2 } from "@/lib/pointCloudParser";
import { convertBodyOrientationToViewer, convertBodyPositionToViewer } from "@/lib/frameTransforms";
// @ts-expect-error - roslib 没有类型声明
import ROSLIB from "roslib";

const DEFAULT_ENDPOINTS = {
  missionCommandService: "/mission/command",
  missionListTopic: "/mission/list",
  missionStatusTopic: "/mission/status",
  waypointFeedbackTopic: "/mission/waypoint_feedback",
  controlTopic: "/mission/control",
  taskOptTopic: "/mission/task_opt",
  hangarChargeTopic: "/hangar/charge_status",
  batteryTopic: "/battery/status",
  poseTopic: "/odom_visualization/pose",
  pointCloudTopic: "/grid_map/occupancy_inflate",
  nestStatusTopic: "/nest/status",
  nestBatteryTopic: "/nest/battery",
  nestMotorStatusTopic: "/nest/motor_status",
  nestAutoModeTopic: "/nest/auto_mode",
  nestUavOnTopic: "/nest/uav_on",
};

const CONTROL_CMDS = {
  TAKEOFF: 1,
  LAND: 2,
  EXECUTE: 3,
  RETURN_HOME: 4,
  ARM_OFF: 5,
  EMERGENCY_LAND: 6,
};

const TASK_OPT_CMDS = {
  START: 1,
  PAUSE: 2,
  STOP: 3,
  EMERGENCY: 4,
};

const EMERGENCY_WAYPOINT_INFO = "__emergency_waypoint__";

type Endpoints = typeof DEFAULT_ENDPOINTS;

type UseMissionRuntimeParams = {
  rosConnected: boolean;
  rosRef: React.MutableRefObject<typeof ROSLIB.Ros | null>;
  mission: Mission | null;
  plannedPoints: PlannedPoint[] | null;
  home: MissionHomePosition | null;
  emergency?: MissionHomePosition | null;
  options?: {
    droneId?: number;
    endpoints?: Partial<Endpoints>;
    lowBatteryThreshold?: number;
    lowBatteryVoltageThreshold?: number;
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

const shiftPointCloud = (data: Float32Array, origin?: { x: number; y: number; z: number } | null) => {
  if (!origin) return data;
  const shifted = new Float32Array(data.length);
  for (let i = 0; i < data.length; i += 3) {
    shifted[i] = data[i] - origin.x;
    shifted[i + 1] = data[i + 1] - origin.y;
    shifted[i + 2] = data[i + 2] - origin.z;
  }
  return shifted;
};

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
  emergency,
  options,
}: UseMissionRuntimeParams) {
  const endpoints = useMemo(() => ({ ...DEFAULT_ENDPOINTS, ...(options?.endpoints ?? {}) }), [options?.endpoints]);
  const origin = home?.position ?? null;
  const [phase, setPhase] = useState<MissionPhase>("idle");
  const [progress, setProgress] = useState<{ completed: number; total: number } | null>(null);
  const completedHistoryRef = useRef(0);
  const currentCompletedRef = useRef(0);
  const [hangar, setHangar] = useState<HangarChargeTelemetry | null>(null);
  const [battery, setBattery] = useState<BatteryTelemetry | null>(null);
  const [events, setEvents] = useState<MissionRuntimeEvent[]>([]);
  const [dronePosition, setDronePosition] = useState<DronePosition | null>(null);
  const [runtimeError, setRuntimeError] = useState<string | null>(null);
  const [busyAction, setBusyAction] = useState<string | null>(null);
  const [lastWaypoint, setLastWaypoint] = useState<{ index: number; info?: string; position?: { x: number; y: number; z: number } } | null>(null);
  const [autoReturnTriggered, setAutoReturnTriggered] = useState(false);
  const [pendingWaypoints, setPendingWaypoints] = useState<PlannedPoint[]>([]);
  const pendingWaypointsRef = useRef<PlannedPoint[]>([]);
  const [pointCloudStack, setPointCloudStack] = useState<Float32Array[]>([]);
  const [rosDebugLogs, setRosDebugLogs] = useState<Array<{ id: string; timestamp: Date; label: string; payload: string }>>([]);
  const [nestStatus, setNestStatus] = useState<NestStatusTelemetry | null>(null);
  const [nestBattery, setNestBattery] = useState<NestBatteryTelemetry | null>(null);
  const [nestMotorStatus, setNestMotorStatus] = useState<NestMotorTelemetry | null>(null);
  const [nestAutoMode, setNestAutoMode] = useState<NestAutoModeTelemetry | null>(null);
  const [nestUavPresence, setNestUavPresence] = useState<NestUavPresenceTelemetry | null>(null);

  const missionCommandService = useRef<typeof ROSLIB.Service | null>(null);
  const missionListTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const controlTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const taskOptTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const missionStatusTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const hangarTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const batteryTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const poseTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const waypointTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const pointCloudTopicRef = useRef<typeof ROSLIB.Topic | null>(null);
  const nestStatusTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const nestBatteryTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const nestMotorStatusTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const nestAutoModeTopic = useRef<typeof ROSLIB.Topic | null>(null);
  const nestUavOnTopic = useRef<typeof ROSLIB.Topic | null>(null);

  const pushEvent = useCallback((level: MissionRuntimeEvent["level"], message: string, details?: Record<string, unknown>) => {
    setEvents((prev) => [makeEvent(level, message, details), ...prev].slice(0, 40));
  }, []);

  useEffect(() => {
    completedHistoryRef.current = 0;
    currentCompletedRef.current = 0;
    pendingWaypointsRef.current = [];
    setPendingWaypoints([]);
    setProgress(null);
    setLastWaypoint(null);
    setEvents([]);
  }, [mission?.id]);

  const resetRosBindings = useCallback(() => {
    missionCommandService.current = null;
    [
      missionListTopic,
      controlTopic,
      taskOptTopic,
      missionStatusTopic,
      hangarTopic,
      batteryTopic,
      poseTopic,
      waypointTopic,
      pointCloudTopicRef,
      nestStatusTopic,
      nestBatteryTopic,
      nestMotorStatusTopic,
      nestAutoModeTopic,
      nestUavOnTopic,
    ].forEach((ref) => {
      ref.current?.unsubscribe?.();
      ref.current = null;
    });
    setPointCloudStack([]);
    setNestStatus(null);
    setNestBattery(null);
    setNestMotorStatus(null);
    setNestAutoMode(null);
    setNestUavPresence(null);
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
    taskOptTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.taskOptTopic,
      messageType: "mission_msgs/TaskOpt",
    });

    missionStatusTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.missionStatusTopic,
      messageType: "mission_msgs/MissionStatus",
    });

    missionStatusTopic.current.subscribe((msg: {
      status?: number;
      PosNum?: number;
      PosList?: Array<{ x: number; y: number; z: number; pass_type?: boolean; task_type?: string | number; info?: string }>;
    }) => {
      const nextPhase = mapStatusToPhase(msg.status);
      setPhase(nextPhase);
      const filteredList = Array.isArray(msg.PosList)
        ? msg.PosList.filter((p) => (p.info ?? "") !== EMERGENCY_WAYPOINT_INFO)
        : [];
      if (filteredList.length > 0) {
        const completed = filteredList.filter((p) => !!p.pass_type).length;
        currentCompletedRef.current = completed;
        setProgress({
          completed: completedHistoryRef.current + completed,
          total: completedHistoryRef.current + filteredList.length,
        });
        let lastCompletedIndex = -1;
        filteredList.forEach((p, i) => {
          if (p.pass_type) lastCompletedIndex = i;
        });
        if (lastCompletedIndex >= 0) {
          const wp = filteredList[lastCompletedIndex];
          setLastWaypoint({ index: lastCompletedIndex, info: wp.info, position: { x: wp.x, y: wp.y, z: wp.z } });
        }
        const remaining = filteredList.filter((p) => !p.pass_type).map((p) => ({
          x: p.x,
          y: p.y,
          z: p.z,
          task_type: normalizeTaskType(p.task_type),
          info: p.info,
        }));
        setPendingWaypoints(remaining);
        pendingWaypointsRef.current = remaining;
      } else {
        setPendingWaypoints([]);
        pendingWaypointsRef.current = [];
        if (completedHistoryRef.current > 0) {
          setProgress({
            completed: completedHistoryRef.current,
            total: completedHistoryRef.current,
          });
        }
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
      messageType: "nav_msgs/Odometry",
    });

    poseTopic.current.subscribe((msg: {
      pose?: {
        pose?: {
          position: { x: number; y: number; z: number };
          orientation: { x: number; y: number; z: number; w: number };
        };
      };
    }) => {
      const poseData = msg.pose?.pose;
      if (!poseData) return;
      const originShifted = origin
        ? {
            x: poseData.position.x - origin.x,
            y: poseData.position.y - origin.y,
            z: poseData.position.z - origin.z,
          }
        : poseData.position;
      const viewerPosition = convertBodyPositionToViewer(originShifted);
      const viewerOrientation = convertBodyOrientationToViewer(poseData.orientation);
      setDronePosition({
        x: viewerPosition.x,
        y: viewerPosition.y,
        z: viewerPosition.z,
        orientation: viewerOrientation ?? undefined,
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

    pointCloudTopicRef.current = new ROSLIB.Topic({
      ros,
      name: endpoints.pointCloudTopic,
      messageType: "sensor_msgs/PointCloud2",
    });

    pointCloudTopicRef.current.subscribe((msg: {
      header: { frame_id: string };
      data: string;
      point_step: number;
      row_step: number;
      width: number;
      height: number;
    }) => {
      try {
        const parsed = createGridMapDataFromPointCloud2(msg);
        const shifted = shiftPointCloud(parsed.data, origin);
        setPointCloudStack((prev) => {
          const next = [shifted, ...prev];
          return next.slice(0, 10);
        });
      } catch (error) {
        console.warn("Failed to parse point cloud", error);
      }
    });
    nestStatusTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.nestStatusTopic,
      messageType: "nest_msgs/NestStatus",
    });
    nestStatusTopic.current.subscribe((msg: { inter_temp?: number; inter_huminity?: number; Opt_mode?: string }) => {
      setNestStatus({
        temperature: typeof msg.inter_temp === "number" ? msg.inter_temp : null,
        humidity: typeof msg.inter_huminity === "number" ? msg.inter_huminity : null,
        optMode: typeof msg.Opt_mode === "string" ? msg.Opt_mode : null,
      });
    });

    nestBatteryTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.nestBatteryTopic,
      messageType: "nest_msgs/NestBattery",
    });
    nestBatteryTopic.current.subscribe((msg: Record<string, unknown>) => {
      setNestBattery({
        soc: typeof msg.battery_soc === "number" ? msg.battery_soc : null,
        chargingVoltage: typeof msg.charging_volt === "number" ? msg.charging_volt : null,
        chargingCurrent: typeof msg.charging_current === "number" ? msg.charging_current : null,
        batteryTemp1: typeof msg.battery_temp1 === "number" ? msg.battery_temp1 : null,
        batteryTemp2: typeof msg.battery_temp2 === "number" ? msg.battery_temp2 : null,
        pcbTemp1: typeof msg.pcb_temp1 === "number" ? msg.pcb_temp1 : null,
        highestCellVolt: typeof msg.highest_cell_volt === "number" ? msg.highest_cell_volt : null,
        maxCellCount: typeof msg.max_cell_no === "number" ? msg.max_cell_no : null,
        cellVoltages: Array.isArray(msg.cell_volt) ? msg.cell_volt.map((value) => Number(value)).filter((value) => Number.isFinite(value)) : [],
        batteryChargedState: typeof msg.battery_charged === "string" ? msg.battery_charged : null,
        highestCellNo: typeof msg.highest_cell_no === "string" ? msg.highest_cell_no : null,
        lowestCellNo: typeof msg.lowest_cell_no === "string" ? msg.lowest_cell_no : null,
        batterySupplyState: typeof msg.battery_supply === "string" ? msg.battery_supply : null,
        batteryBalanceState: typeof msg.battery_balance === "string" ? msg.battery_balance : null,
        chargingOk: typeof msg.charging_OK === "string" ? msg.charging_OK : null,
      });
    });

    nestMotorStatusTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.nestMotorStatusTopic,
      messageType: "nest_msgs/NestMotorStatus",
    });
    nestMotorStatusTopic.current.subscribe((msg: Record<string, unknown>) => {
      setNestMotorStatus({
        xMotorAlarm: typeof msg.XMotor === "string" ? msg.XMotor : null,
        yMotorAlarm: typeof msg.YMotor === "string" ? msg.YMotor : null,
        xAxisStatus: typeof msg.XAxisStatus === "string" ? msg.XAxisStatus : null,
        yAxisStatus: typeof (msg as Record<string, unknown>)["Y AxisStatus"] === "string" ? (msg as Record<string, unknown>)["Y AxisStatus"] as string : null,
      });
    });

    nestAutoModeTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.nestAutoModeTopic,
      messageType: "nest_msgs/NestAutoMode",
    });
    nestAutoModeTopic.current.subscribe((msg: { Auto_mode?: string }) => {
      setNestAutoMode({ mode: typeof msg.Auto_mode === "string" ? msg.Auto_mode : null });
    });

    nestUavOnTopic.current = new ROSLIB.Topic({
      ros,
      name: endpoints.nestUavOnTopic,
      messageType: "nest_msgs/UavOn",
    });
    nestUavOnTopic.current.subscribe((msg: { isUAVOn?: string }) => {
      setNestUavPresence({ state: typeof msg.isUAVOn === "string" ? msg.isUAVOn : null });
    });

    return () => {
      resetRosBindings();
    };
  }, [rosConnected, rosRef, endpoints, pushEvent, resetRosBindings, origin]);

  const appendDebugLog = useCallback((label: string, payload: unknown) => {
    const entry = {
      id: `${label}-${Date.now()}-${Math.random().toString(16).slice(2)}`,
      timestamp: new Date(),
      label,
      payload: (() => {
        try { return JSON.stringify(payload); } catch { return String(payload); }
      })(),
    };
    setRosDebugLogs((prev) => [entry, ...prev].slice(0, 50));
    try {
      console.info(`[ROS][${label}]`, payload);
    } catch {
      // ignore console issues
    }
  }, []);
  const clearDebugLogs = useCallback(() => setRosDebugLogs([]), []);

  const callMissionCommand = useCallback(async (command: number) => {
    if (!missionCommandService.current) throw new Error("MissionCommand 服务未就绪");
    setBusyAction("mission-command");
    appendDebugLog("MissionCommand.request", { command });
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
  }, [appendDebugLog, pushEvent]);

  const publishControl = useCallback(async (cmd: number) => {
    if (!controlTopic.current) throw new Error("控制话题未连接");
    setBusyAction(`control-${cmd}`);
    try {
      const msg = new ROSLIB.Message({
        cmd,
        drone_id: options?.droneId ?? 1,
      });
      appendDebugLog("Control.publish", msg);
      controlTopic.current!.publish(msg);
      pushEvent("info", `发送控制指令 ${cmd}`);
    } finally {
      setBusyAction(null);
    }
  }, [appendDebugLog, options?.droneId, pushEvent]);

  const clampInt32 = (value: number) => {
    if (!Number.isFinite(value)) return 0;
    const truncated = Math.trunc(value);
    const MAX = 2147483647;
    const MIN = -2147483648;
    return Math.min(Math.max(truncated, MIN), MAX);
  };

  const publishTaskOpt = useCallback(async (opt: number, label: string) => {
    if (!taskOptTopic.current) throw new Error("TaskOpt 话题未连接");
    if (!mission) throw new Error("请先选择任务");
    setBusyAction(`taskopt-${opt}`);
    try {
      const parsedId = clampInt32(Number.parseInt(mission.id, 10));
      const message = new ROSLIB.Message({
        opt,
        id: parsedId,
      });
      appendDebugLog("TaskOpt.publish", message);
      taskOptTopic.current.publish(message);
      pushEvent("info", label, { opt, mission: mission.id });
    } finally {
      setBusyAction(null);
    }
  }, [appendDebugLog, mission, pushEvent]);

  const sendMissionList = useCallback(async (pointsToUpload: PlannedPoint[], label: string, options?: { resetProgress?: boolean }) => {
    if (!missionListTopic.current) throw new Error("MissionList 话题未连接");
    if (!mission) throw new Error("请先选择任务");
    if (!home) throw new Error("请先设置 HomePos");
    if (!pointsToUpload || pointsToUpload.length === 0) throw new Error("缺少航线");

    if (options?.resetProgress) {
      completedHistoryRef.current = 0;
      currentCompletedRef.current = 0;
    } else {
      completedHistoryRef.current += currentCompletedRef.current;
      currentCompletedRef.current = 0;
    }

    setBusyAction(label);
    const now = Date.now();
    const secs = Math.floor(now / 1000);
    const nsecs = Math.floor((now % 1000) * 1e6);
    const parsedId = Number.parseInt(mission.id, 10);
    const normalizedBase = pointsToUpload.map((p, index) => ({
      x: p.x,
      y: p.y,
      z: p.z,
      w: p.w,
      task_type: normalizeTaskType(p.task_type) ?? 0,
      info: p.info ?? `wp-${index}`,
    }));
    const emergencyNormalized = emergency ? {
      x: emergency.position.x,
      y: emergency.position.y,
      z: emergency.position.z,
      w: emergency.yaw,
      task_type: 0,
      info: EMERGENCY_WAYPOINT_INFO,
    } : null;
    const rosWaypoints = emergencyNormalized ? [...normalizedBase, emergencyNormalized] : normalizedBase;
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
        PosNum: rosWaypoints.length,
        PosList: rosWaypoints.map((p) => ({
          x: p.x,
          y: p.y,
          z: p.z,
          w: p.w ?? 0,
          pass_type: false,
          task_type: serializeTaskType(p.task_type as TaskType | undefined),
          info: p.info,
        })),
      });
      appendDebugLog("MissionList.publish", message);
      missionListTopic.current.publish(message);
      setPendingWaypoints(normalizedBase);
      pendingWaypointsRef.current = normalizedBase;
      setPhase("mission_upload");
      pushEvent("info", `${label}`, { waypoints: normalizedBase.length, emergencyAppended: emergencyNormalized != null });
    } finally {
      setBusyAction(null);
    }
  }, [appendDebugLog, missionListTopic, mission, home, emergency, pushEvent]);

  const uploadMission = useCallback(async () => {
    if (!plannedPoints || plannedPoints.length === 0) throw new Error("缺少航线");
    await sendMissionList(plannedPoints, "任务列表已上传", { resetProgress: true });
  }, [plannedPoints, sendMissionList]);


  const resumeMission = useCallback(async () => {
    const remaining = pendingWaypointsRef.current;
    if (!remaining || remaining.length === 0) {
      throw new Error("没有待执行的航点");
    }
    await sendMissionList(remaining, "续航任务已上传", { resetProgress: false });
    pushEvent("info", "自动开始剩余航点");
    await publishTaskOpt(TASK_OPT_CMDS.START, "TaskOpt 启动剩余任务");
    await publishControl(CONTROL_CMDS.EXECUTE);
  }, [publishControl, publishTaskOpt, sendMissionList, pushEvent]);

  const uploadEmergencyMission = useCallback(async () => {
    pushEvent("warning", "执行迫降指令");
    await publishTaskOpt(TASK_OPT_CMDS.EMERGENCY, "TaskOpt 迫降");
    await publishControl(CONTROL_CMDS.EMERGENCY_LAND);
  }, [publishControl, publishTaskOpt, pushEvent]);

  const handleTakeoff = useCallback(() => publishControl(CONTROL_CMDS.TAKEOFF), [publishControl]);
  const handleExecute = useCallback(async () => {
    await publishTaskOpt(TASK_OPT_CMDS.START, "TaskOpt 启动任务");
    await publishControl(CONTROL_CMDS.EXECUTE);
  }, [publishControl, publishTaskOpt]);
  const handleReturnHome = useCallback(async () => {
    await publishTaskOpt(TASK_OPT_CMDS.STOP, "TaskOpt 停止任务");
    await publishControl(CONTROL_CMDS.RETURN_HOME);
  }, [publishControl, publishTaskOpt]);
  const handleLand = useCallback(() => publishControl(CONTROL_CMDS.LAND), [publishControl]);
  const handleArmOff = useCallback(() => publishControl(CONTROL_CMDS.ARM_OFF), [publishControl]);
  const handleOpenHangar = useCallback(() => callMissionCommand(1), [callMissionCommand]);
  const handleCloseHangar = useCallback(() => callMissionCommand(2), [callMissionCommand]);

  useEffect(() => {
    if (!battery) return;
    const voltage = typeof battery.voltage === "number" ? battery.voltage : null;
    const voltageThreshold = options?.lowBatteryVoltageThreshold ?? 21;
    const voltageRelease = voltageThreshold + 1;
    if (voltage != null) {
      if (voltage <= voltageThreshold && !autoReturnTriggered) {
        setAutoReturnTriggered(true);
        pushEvent("warning", `电压低（${voltage.toFixed(2)}V），触发返航`);
        handleReturnHome().catch((error) => {
          setRuntimeError(error instanceof Error ? error.message : String(error));
        });
      }
      if (voltage >= voltageRelease && autoReturnTriggered) {
        setAutoReturnTriggered(false);
      }
      return;
    }
    if (battery.percentage == null) return;
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
  }, [battery, options?.lowBatteryThreshold, options?.lowBatteryVoltageThreshold, autoReturnTriggered, handleReturnHome, pushEvent]);

  const resumeAvailable = useMemo(() => {
    const inHangarReady = phase === "hangar_ready" || phase === "mission_upload";
    const hangarReady = (hangar?.status === 2) || hangar?.status == null;
    return inHangarReady && hangarReady && pendingWaypoints.length > 0;
  }, [phase, hangar?.status, pendingWaypoints.length]);

  const nestTelemetry: NestTelemetry = useMemo(() => ({
    status: nestStatus,
    battery: nestBattery,
    motor: nestMotorStatus,
    autoMode: nestAutoMode,
    uavPresence: nestUavPresence,
  }), [nestStatus, nestBattery, nestMotorStatus, nestAutoMode, nestUavPresence]);

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
    pointClouds: pointCloudStack,
    debugLogs: rosDebugLogs,
    clearDebugLogs,
    nest: nestTelemetry,
    actions: {
      openHangar: handleOpenHangar,
      closeHangar: handleCloseHangar,
      uploadMission,
      uploadEmergencyMission,
      executeMission: handleExecute,
      takeoff: handleTakeoff,
      returnHome: handleReturnHome,
      land: handleLand,
      armOff: handleArmOff,
      resumeMission,
    },
  };
}
