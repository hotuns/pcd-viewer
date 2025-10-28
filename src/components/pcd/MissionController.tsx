"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { PCDCanvas, PCDCanvasHandle } from "./PCDCanvas";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Separator } from "@/components/ui/separator";
import { Maximize, PlayCircle, StopCircle } from "lucide-react";
import MissionManager from "@/components/mission/MissionManager";
import { Mission, DronePosition } from "@/types/mission";
// @ts-expect-error - roslib 没有官方类型定义
import ROSLIB from "roslib";

export default function MissionController() {
  const canvasRef = useRef<PCDCanvasHandle | null>(null);
  const [selectedMission, setSelectedMission] = useState<Mission | null>(null);
  const [pointSize, setPointSize] = useState(0.01);
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(true);
  const [showDrone, setShowDrone] = useState(true);
  const [showVelocityVector, setShowVelocityVector] = useState(true);
  const [showLivePointCloud, setShowLivePointCloud] = useState(true);
  const [showGridMap, setShowGridMap] = useState(true);
  const [loading, setLoading] = useState(false);

  // ROS 连接
  const [rosUrl, setRosUrl] = useState("ws://101.132.193.179:7001");
  const [rosConnected, setRosConnected] = useState(false);
  const [missionRunning, setMissionRunning] = useState(false);
  const [dronePosition, setDronePosition] = useState<DronePosition | null>(
    null
  );
  const [testMode, setTestMode] = useState(false);
  const rosRef = useRef<typeof ROSLIB.Ros | null>(null);
  const odomTopicRef = useRef<typeof ROSLIB.Topic | null>(null);
  const pointCloudTopicRef = useRef<typeof ROSLIB.Topic | null>(null);
  const gridMapTopicRef = useRef<typeof ROSLIB.Topic | null>(null);
  const testIntervalRef = useRef<NodeJS.Timeout | null>(null);

  // 实时点云数据
  const [livePointCloud, setLivePointCloud] = useState<{
    points: Float32Array;
    colors?: Float32Array;
    count: number;
  } | null>(null);

  // 栅格地图数据
  const [gridMapData, setGridMapData] = useState<{
    points: Float32Array;
    colors?: Float32Array;
    count: number;
    width: number;
    height: number;
  } | null>(null);

  // 点云和包围盒信息
  const [count, setCount] = useState<number>(0);
  const [bbox, setBbox] = useState<{
    min: [number, number, number];
    max: [number, number, number];
  } | null>(null);

  // ROS 连接管理
  const connectROS = useCallback(() => {
    if (!rosUrl) return;

    try {
      // 断开旧连接
      if (rosRef.current) {
        rosRef.current.close();
      }

      const ros = new ROSLIB.Ros({
        url: rosUrl,
      });

      ros.on("connection", () => {
        console.log("Connected to ROS");
        setRosConnected(true);
      });

      ros.on("error", (error: unknown) => {
        console.error("ROS connection error:", error);
        setRosConnected(false);
      });

      ros.on("close", () => {
        console.log("ROS connection closed");
        setRosConnected(false);
      });

      rosRef.current = ros;
    } catch (error) {
      console.error("Failed to connect to ROS:", error);
      alert("ROS 连接失败");
    }
  }, [rosUrl]);

  const disconnectROS = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }
    if (pointCloudTopicRef.current) {
      pointCloudTopicRef.current.unsubscribe();
      pointCloudTopicRef.current = null;
    }
    setRosConnected(false);
  }, []);

  // 开始任务
  const startMission = useCallback(() => {
    if (!rosConnected && !testMode) {
      alert("请先连接 ROS 或启用测试模式");
      return;
    }

    if (testMode) {
      // 测试模式：生成假数据
      let time = 0;
      testIntervalRef.current = setInterval(() => {
        time += 0.1;
        const radius = 5;
        const height = 2 + Math.sin(time * 0.5) * 1;

        setDronePosition({
          x: Math.cos(time * 0.3) * radius,
          y: Math.sin(time * 0.3) * radius,
          z: height,
          orientation: {
            x: 0,
            y: 0,
            z: Math.sin(time * 0.2) * 0.3,
            w: Math.cos(time * 0.2) * 0.3 + 0.7,
          },
          velocity: {
            x: -Math.sin(time * 0.3) * radius * 0.3,
            y: Math.cos(time * 0.3) * radius * 0.3,
            z: Math.cos(time * 0.5) * 0.5,
          },
        });

        // 生成测试点云数据
        const testPointCount = 1000;
        const testPoints = new Float32Array(testPointCount * 3);
        const testColors = new Float32Array(testPointCount * 3);

        for (let i = 0; i < testPointCount; i++) {
          // 生成随机点云
          const angle = (i / testPointCount) * Math.PI * 2;
          const r = Math.random() * 3 + 1;
          testPoints[i * 3] = Math.cos(angle) * r + Math.cos(time) * 2;
          testPoints[i * 3 + 1] = Math.sin(angle) * r + Math.sin(time) * 2;
          testPoints[i * 3 + 2] = Math.random() * 2;

          // 设置颜色
          testColors[i * 3] = Math.random() * 0.5;
          testColors[i * 3 + 1] = 1.0;
          testColors[i * 3 + 2] = Math.random() * 0.5;
        }

        setLivePointCloud({
          points: testPoints,
          colors: testColors,
          count: testPointCount,
        });
      }, 100);
      setMissionRunning(true);
      return;
    }

    if (!rosRef.current) return;

    // 订阅无人机位置话题
    const odomTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/mavros/local_position/odom",
      messageType: "nav_msgs/msg/Odometry",
    });

    odomTopic.subscribe(
      (msg: {
        pose: {
          pose: {
            position: { x: number; y: number; z: number };
            orientation: { x: number; y: number; z: number; w: number };
          };
        };
        twist: { twist: { linear: { x: number; y: number; z: number } } };
      }) => {
        const pos = msg.pose.pose.position;
        const orient = msg.pose.pose.orientation;
        const vel = msg.twist.twist.linear;

        setDronePosition({
          x: pos.x,
          y: pos.y,
          z: pos.z,
          orientation: {
            x: orient.x,
            y: orient.y,
            z: orient.z,
            w: orient.w,
          },
          velocity: {
            x: vel.x,
            y: vel.y,
            z: vel.z,
          },
        });
      }
    );

    odomTopicRef.current = odomTopic;

    // 订阅点云话题
    const pointCloudTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/lidar/point_cloud",
      messageType: "sensor_msgs/PointCloud2",
    });

    pointCloudTopic.subscribe(
      (msg: {
        width: number;
        height: number;
        fields: Array<{
          name: string;
          offset: number;
          datatype: number;
          count: number;
        }>;
        point_step: number;
        row_step: number;
        data: string | Uint8Array | number[];
        is_dense: boolean;
        is_bigendian?: boolean;
      }) => {
        try {
          // 解析PointCloud2数据
          const pointCount = msg.width * msg.height;
          if (pointCount === 0) {
            console.warn("Point cloud is empty");
            return;
          }

          // 创建点位置数组
          const points = new Float32Array(pointCount * 3);
          const colors = new Float32Array(pointCount * 3);

          // 查找XYZ字段的偏移量
          const xField = msg.fields.find((f) => f.name === "x");
          const yField = msg.fields.find((f) => f.name === "y");
          const zField = msg.fields.find((f) => f.name === "z");
          const intensityField = msg.fields.find((f) => f.name === "intensity");

          if (!xField || !yField || !zField) {
            console.warn("Point cloud missing XYZ fields");
            return;
          }

          // 处理不同的数据格式
          let dataBuffer: ArrayBuffer;

          if (typeof msg.data === "string") {
            // Base64编码的字符串，需要解码
            try {
              const binaryString = atob(msg.data);
              const bytes = new Uint8Array(binaryString.length);
              for (let i = 0; i < binaryString.length; i++) {
                bytes[i] = binaryString.charCodeAt(i);
              }
              dataBuffer = bytes.buffer;
            } catch (e) {
              console.error("Failed to decode base64 data:", e);
              return;
            }
          } else {
            console.error("Unsupported data format:", typeof msg.data);
            return;
          }

          // 验证数据长度
          const expectedDataLength = pointCount * msg.point_step;
          if (dataBuffer.byteLength < expectedDataLength) {
            console.error(
              `Data length mismatch: expected ${expectedDataLength}, got ${dataBuffer.byteLength}`
            );
            return;
          }

          // 解析点云数据
          const dataView = new DataView(dataBuffer);
          const isLittleEndian = !msg.is_bigendian;

          let validPointCount = 0;
          for (let i = 0; i < pointCount; i++) {
            const pointOffset = i * msg.point_step;

            // 检查偏移量是否在有效范围内
            if (
              pointOffset +
                Math.max(
                  xField.offset + 4,
                  yField.offset + 4,
                  zField.offset + 4
                ) >
              dataBuffer.byteLength
            ) {
              console.warn(`Point ${i}: offset out of bounds, skipping`);
              continue;
            }

            try {
              // 读取XYZ坐标 (假设为float32)
              const x = dataView.getFloat32(
                pointOffset + xField.offset,
                isLittleEndian
              );
              const y = dataView.getFloat32(
                pointOffset + yField.offset,
                isLittleEndian
              );
              const z = dataView.getFloat32(
                pointOffset + zField.offset,
                isLittleEndian
              );

              // 检查坐标值是否有效
              if (!isFinite(x) || !isFinite(y) || !isFinite(z)) {
                continue;
              }

              points[validPointCount * 3] = x;
              points[validPointCount * 3 + 1] = y;
              points[validPointCount * 3 + 2] = z;

              // 设置颜色 (基于强度或默认绿色)
              if (
                intensityField &&
                pointOffset + intensityField.offset + 4 <= dataBuffer.byteLength
              ) {
                try {
                  const intensity = dataView.getFloat32(
                    pointOffset + intensityField.offset,
                    isLittleEndian
                  );
                  if (isFinite(intensity)) {
                    const normalizedIntensity = Math.min(
                      Math.abs(intensity) / 255.0,
                      1.0
                    );
                    colors[validPointCount * 3] = normalizedIntensity;
                    colors[validPointCount * 3 + 1] = 1.0;
                    colors[validPointCount * 3 + 2] = normalizedIntensity;
                  } else {
                    colors[validPointCount * 3] = 0.0;
                    colors[validPointCount * 3 + 1] = 1.0;
                    colors[validPointCount * 3 + 2] = 0.0;
                  }
                } catch (e) {
                  colors[validPointCount * 3] = 0.0;
                  colors[validPointCount * 3 + 1] = 1.0;
                  colors[validPointCount * 3 + 2] = 0.0;
                }
              } else {
                colors[validPointCount * 3] = 0.0;
                colors[validPointCount * 3 + 1] = 1.0;
                colors[validPointCount * 3 + 2] = 0.0;
              }

              validPointCount++;
            } catch (e) {
              console.warn(`Error reading point ${i}:`, e);
              continue;
            }
          }

          if (validPointCount > 0) {
            // 创建正确大小的数组
            const finalPoints = new Float32Array(validPointCount * 3);
            const finalColors = new Float32Array(validPointCount * 3);

            finalPoints.set(points.subarray(0, validPointCount * 3));
            finalColors.set(colors.subarray(0, validPointCount * 3));

            setLivePointCloud({
              points: finalPoints,
              colors: finalColors,
              count: validPointCount,
            });
          } else {
            console.warn("No valid points found in point cloud data");
          }
        } catch (error) {
          console.error("Error parsing point cloud data:", error);
        }
      }
    );

    pointCloudTopicRef.current = pointCloudTopic;

    // 订阅栅格地图话题
    const gridMapTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/drone_0_grid/grid_map/occupancy_inflate",
      messageType: "sensor_msgs/PointCloud2",
    });

    gridMapTopic.subscribe(
      (msg: {
        width: number;
        height: number;
        fields: Array<{
          name: string;
          offset: number;
          datatype: number;
          count: number;
        }>;
        point_step: number;
        row_step: number;
        data: string | Uint8Array | number[];
        is_dense: boolean;
        is_bigendian?: boolean;
      }) => {
        try {
          // 解析栅格地图数
          const pointCount = msg.width * msg.height;
          if (pointCount === 0) {
            console.warn("Grid map is empty");
            return;
          }

          // 创建点位置数组
          const points = new Float32Array(pointCount * 3);
          const colors = new Float32Array(pointCount * 3);

          // 查找XYZ字段的偏移量
          const xField = msg.fields.find((f) => f.name === "x");
          const yField = msg.fields.find((f) => f.name === "y");
          const zField = msg.fields.find((f) => f.name === "z");

          if (!xField || !yField || !zField) {
            console.warn("Grid map missing XYZ fields");
            return;
          }

          // 处理不同的数据格式
          let dataBuffer: ArrayBuffer;

          if (typeof msg.data === "string") {
            // Base64编码的字符串，需要解码
            try {
              const binaryString = atob(msg.data);
              const bytes = new Uint8Array(binaryString.length);
              for (let i = 0; i < binaryString.length; i++) {
                bytes[i] = binaryString.charCodeAt(i);
              }
              dataBuffer = bytes.buffer;
            } catch (e) {
              console.error("Failed to decode base64 grid map data:", e);
              return;
            }
          } else {
            console.error("Unsupported grid map data format:", typeof msg.data);
            return;
          }

          // 验证数据长度
          const expectedDataLength = pointCount * msg.point_step;
          if (dataBuffer.byteLength < expectedDataLength) {
            console.error(
              `Grid map data length mismatch: expected ${expectedDataLength}, got ${dataBuffer.byteLength}`
            );
            return;
          }

          // 解析栅格地图数据
          const dataView = new DataView(dataBuffer);
          const isLittleEndian = !msg.is_bigendian;

          let validPointCount = 0;
          for (let i = 0; i < pointCount; i++) {
            const pointOffset = i * msg.point_step;

            // 检查偏移量是否在有效范围内
            if (
              pointOffset +
                Math.max(
                  xField.offset + 4,
                  yField.offset + 4,
                  zField.offset + 4
                ) >
              dataBuffer.byteLength
            ) {
              console.warn(
                `Grid map point ${i}: offset out of bounds, skipping`
              );
              continue;
            }

            try {
              // 读取XYZ坐标 (假设为float32)
              const x = dataView.getFloat32(
                pointOffset + xField.offset,
                isLittleEndian
              );
              const y = dataView.getFloat32(
                pointOffset + yField.offset,
                isLittleEndian
              );
              const z = dataView.getFloat32(
                pointOffset + zField.offset,
                isLittleEndian
              );

              // 检查坐标值是否有效
              if (!isFinite(x) || !isFinite(y) || !isFinite(z)) {
                continue;
              }

              points[validPointCount * 3] = x;
              points[validPointCount * 3 + 1] = y;
              points[validPointCount * 3 + 2] = z;

              // 栅格地图使用红色显示
              colors[validPointCount * 3] = 1.0; // R
              colors[validPointCount * 3 + 1] = 0.2; // G
              colors[validPointCount * 3 + 2] = 0.2; // B

              validPointCount++;
            } catch (e) {
              console.warn(`Error reading grid map point ${i}:`, e);
              continue;
            }
          }

          if (validPointCount > 0) {
            // 创建正确大小的数组
            const finalPoints = new Float32Array(validPointCount * 3);
            const finalColors = new Float32Array(validPointCount * 3);

            finalPoints.set(points.subarray(0, validPointCount * 3));
            finalColors.set(colors.subarray(0, validPointCount * 3));

            setGridMapData({
              points: finalPoints,
              colors: finalColors,
              count: validPointCount,
              width: msg.width,
              height: msg.height,
            });
          } else {
            console.warn("No valid points found in grid map data");
          }
        } catch (error) {
          console.error("Error parsing grid map data:", error);
        }
      }
    );

    gridMapTopicRef.current = gridMapTopic;
    setMissionRunning(true);
  }, [rosConnected, testMode]);

  // 停止任务
  const stopMission = useCallback(() => {
    if (testIntervalRef.current) {
      clearInterval(testIntervalRef.current);
      testIntervalRef.current = null;
    }
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }
    if (pointCloudTopicRef.current) {
      pointCloudTopicRef.current.unsubscribe();
      pointCloudTopicRef.current = null;
    }
    if (gridMapTopicRef.current) {
      gridMapTopicRef.current.unsubscribe();
      gridMapTopicRef.current = null;
    }
    setMissionRunning(false);
    setDronePosition(null);
    setLivePointCloud(null);
    setGridMapData(null);
  }, []);

  // 组件卸载时清理
  useEffect(() => {
    return () => {
      disconnectROS();
    };
  }, [disconnectROS]);

  const bboxText = bbox
    ? `min(${bbox.min[0].toFixed(3)}, ${bbox.min[1].toFixed(
        3
      )}, ${bbox.min[2].toFixed(3)})  max(${bbox.max[0].toFixed(
        3
      )}, ${bbox.max[1].toFixed(3)}, ${bbox.max[2].toFixed(3)})`
    : "-";

  return (
    <div className="w-full h-[calc(100vh-2rem)] grid grid-cols-[320px_1fr] gap-4 p-4">
      <aside className="bg-card border border-border rounded-lg p-4 space-y-4 overflow-y-auto">
        {/* 任务管理 */}
        <MissionManager
          selectedMission={selectedMission}
          onMissionSelect={setSelectedMission}
        />

        <Separator />

        {/* ROS 连接 */}
        <div className="space-y-2">
          <Label>ROS 连接</Label>
          <Input
            placeholder="ws://host:port"
            value={rosUrl}
            onChange={(e) => setRosUrl(e.target.value)}
            disabled={rosConnected}
          />
          <div className="flex gap-2">
            <Button
              onClick={connectROS}
              disabled={rosConnected}
              className="flex-1"
            >
              连接
            </Button>
            <Button
              onClick={disconnectROS}
              disabled={!rosConnected}
              variant="outline"
              className="flex-1"
            >
              断开
            </Button>
          </div>
          <div className="flex items-center justify-between">
            <div className="text-xs text-muted-foreground">
              状态: {rosConnected ? "✓ 已连接" : "✗ 未连接"}
            </div>
            <div className="flex items-center gap-2">
              <Switch
                checked={testMode}
                onCheckedChange={setTestMode}
                id="testMode"
                disabled={missionRunning}
              />
              <Label htmlFor="testMode" className="text-xs">
                测试模式
              </Label>
            </div>
          </div>
        </div>

        <Separator />

        {/* 任务控制 */}
        <div className="space-y-2">
          <Label>任务控制</Label>
          <Button
            onClick={missionRunning ? stopMission : startMission}
            disabled={!rosConnected && !testMode}
            className="w-full"
            variant={missionRunning ? "destructive" : "default"}
          >
            {missionRunning ? (
              <>
                <StopCircle className="h-4 w-4 mr-2" /> 停止任务
              </>
            ) : (
              <>
                <PlayCircle className="h-4 w-4 mr-2" />
                {testMode ? "开始测试" : "开始任务"}
              </>
            )}
          </Button>
        </div>

        {/* 无人机状态 */}
        {dronePosition && (
          <>
            <Separator />
            <div className="space-y-2">
              <Label>无人机位置</Label>
              <div className="text-xs font-mono space-y-1">
                <div>X: {dronePosition.x.toFixed(3)} m</div>
                <div>Y: {dronePosition.y.toFixed(3)} m</div>
                <div>Z: {dronePosition.z.toFixed(3)} m</div>
                {dronePosition.velocity && (
                  <>
                    <Separator className="my-1" />
                    <div>
                      速度:{" "}
                      {Math.hypot(
                        dronePosition.velocity.x,
                        dronePosition.velocity.y,
                        dronePosition.velocity.z
                      ).toFixed(2)}{" "}
                      m/s
                    </div>
                  </>
                )}
              </div>
            </div>
          </>
        )}

        <Separator />

        {/* 显示设置 */}
        <div className="space-y-3">
          <Label>显示设置</Label>

          <div className="space-y-2">
            <Label className="text-sm">点大小</Label>
            <Slider
              value={[pointSize]}
              min={0.001}
              max={0.1}
              step={0.001}
              onValueChange={(v) => setPointSize(v[0] ?? 0.01)}
            />
            <div className="text-xs text-muted-foreground">
              {pointSize.toFixed(3)}
            </div>
          </div>

          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Switch
                checked={showAxes}
                onCheckedChange={setShowAxes}
                id="axes"
              />
              <Label htmlFor="drone">坐标轴</Label>
            </div>
            <div className="flex items-center gap-2">
              <Switch
                checked={showGrid}
                onCheckedChange={setShowGrid}
                id="drone"
              />
              <Label htmlFor="grid">网格</Label>
            </div>
          </div>
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Switch
                checked={showDrone}
                onCheckedChange={setShowDrone}
                id="drone"
              />
              <Label htmlFor="drone">无人机</Label>
            </div>
            <div className="flex items-center gap-2">
              <Switch
                checked={showVelocityVector}
                onCheckedChange={setShowVelocityVector}
                id="velocity"
              />
              <Label htmlFor="velocity">速度矢量</Label>
            </div>
          </div>

          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Switch
                checked={showLivePointCloud}
                onCheckedChange={setShowLivePointCloud}
                id="livePointCloud"
              />
              <Label htmlFor="livePointCloud">实时点云</Label>
            </div>
            <div className="flex items-center gap-2">
              <Switch
                checked={showGridMap}
                onCheckedChange={setShowGridMap}
                id="gridMap"
              />
              <Label htmlFor="gridMap">栅格地图</Label>
            </div>
          </div>
        </div>

        <Separator />

        {/* 点云信息 */}
        <div className="space-y-1 text-xs text-muted-foreground">
          <div>点数: {count.toLocaleString()}</div>
          <div className="break-all">包围盒: {bboxText}</div>
          {livePointCloud && (
            <div>实时点云: {livePointCloud.count.toLocaleString()} 点</div>
          )}
          {gridMapData && (
            <div>
              栅格地图: {gridMapData.count.toLocaleString()} 点 (
              {gridMapData.width}×{gridMapData.height})
            </div>
          )}
        </div>

        <Button
          className="w-full"
          variant="secondary"
          onClick={() => canvasRef.current?.fitToView()}
        >
          <Maximize className="h-4 w-4 mr-2" /> 视图适配
        </Button>
      </aside>

      <section className="bg-card border border-border rounded-lg relative overflow-hidden">
        <div className="w-full h-full min-h-[400px] relative">
          {loading && (
            <div className="absolute inset-0 z-10 flex items-center justify-center bg-background/60 backdrop-blur-sm">
              <div className="flex flex-col items-center gap-3 text-muted-foreground">
                <div className="h-8 w-8 animate-spin rounded-full border-2 border-primary border-t-transparent" />
                <div className="text-xs">加载中...</div>
              </div>
            </div>
          )}
          <PCDCanvas
            ref={canvasRef}
            source={selectedMission?.scene ?? null}
            pointSize={pointSize}
            showGrid={showGrid}
            showAxes={showAxes}
            dronePosition={dronePosition}
            showDrone={showDrone}
            showVelocityVector={showVelocityVector}
            livePointCloud={livePointCloud}
            showLivePointCloud={showLivePointCloud}
            gridMapData={gridMapData}
            showGridMap={showGridMap}
            onLoadedAction={({ bbox, count }) => {
              setCount(count);
              setBbox({
                min: [bbox.min.x, bbox.min.y, bbox.min.z],
                max: [bbox.max.x, bbox.max.y, bbox.max.z],
              });
            }}
            onLoadingChange={setLoading}
          />
        </div>
      </section>
    </div>
  );
}
