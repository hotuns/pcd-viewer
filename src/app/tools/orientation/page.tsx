"use client";

import { Suspense, useMemo, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import * as THREE from "three";
import { DroneModel } from "@/components/pcd/DroneModel";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { convertBodyOrientationToViewer, convertBodyPositionToViewer } from "@/lib/frameTransforms";
import { AxisLabels } from "@/components/pcd/AxisLabels";

const degreeToRad = (deg: number) => (deg * Math.PI) / 180;

export default function DroneOrientationTool() {
  const [position, setPosition] = useState({ x: 0, y: 0, z: 0 });
  const [angles, setAngles] = useState({ roll: 0, pitch: 0, yaw: 0 });

  const quaternion = useMemo(() => {
    const euler = new THREE.Euler(
      degreeToRad(angles.roll),
      degreeToRad(angles.pitch),
      degreeToRad(angles.yaw),
      "XYZ"
    );
    const quat = new THREE.Quaternion().setFromEuler(euler);
    return { x: quat.x, y: quat.y, z: quat.z, w: quat.w };
  }, [angles]);

  const viewerPosition = useMemo(() => convertBodyPositionToViewer(position), [position]);
  const viewerOrientation = useMemo(() => convertBodyOrientationToViewer(quaternion) ?? quaternion, [quaternion]);

  const axesHelper = useMemo(() => new THREE.AxesHelper(2), []);

  const handlePositionChange = (axis: "x" | "y" | "z", value: number) => {
    setPosition((prev) => ({ ...prev, [axis]: value }));
  };

  const handleAngleChange = (axis: "roll" | "pitch" | "yaw", value: number) => {
    setAngles((prev) => ({ ...prev, [axis]: value }));
  };

  return (
    <div className="flex flex-col gap-4 p-4 h-full">
      <div className="grid grid-cols-3 gap-3 text-sm">
        {(["x", "y", "z"] as const).map((axis) => {
          const axisLabel = axis === "x" ? "X (前后)" : axis === "y" ? "Y (左右)" : "Z (上下)";
          return (
            <div key={axis} className="space-y-1">
              <Label className="text-xs uppercase tracking-wide text-slate-500">{axisLabel}</Label>
              <Input
                type="number"
                value={position[axis]}
                step="0.1"
                onChange={(e) => handlePositionChange(axis, Number(e.target.value))}
              />
            </div>
          );
        })}
        {(["roll", "pitch", "yaw"] as const).map((axis) => (
          <div key={axis} className="space-y-1">
            <Label className="text-xs uppercase tracking-wide text-slate-500">{axis} (°)</Label>
            <Input
              type="number"
              value={angles[axis]}
              step="1"
              onChange={(e) => handleAngleChange(axis, Number(e.target.value))}
            />
          </div>
        ))}
      </div>

      <div className="flex-1 min-h-[500px] rounded-xl border border-slate-800 overflow-hidden bg-black">
        <Canvas camera={{ position: [-4, 3, 4], fov: 55 }}>
          <ambientLight intensity={0.6} />
          <directionalLight position={[3, 5, 2]} intensity={0.8} />
          <Suspense fallback={null}>
            <DroneModel
              position={[viewerPosition.x, viewerPosition.y, viewerPosition.z]}
              orientation={viewerOrientation}
            />
          </Suspense>
          <Grid args={[10, 10]} />
          <primitive object={axesHelper} />
          <AxisLabels length={2.5} labelOffset={0.1} mode="ros" />
          <OrbitControls />
        </Canvas>
        <div className="mt-2 text-[11px] text-slate-400 flex flex-col gap-1 leading-relaxed">
          <span>输入遵循 ROS 机体系：X 前进、Y 向左、Z 高度。</span>
          <span>在 Three.js 视图中：红 X 对应 ROS X（向右显示前进）、绿 Y 对应 ROS Z（垂直向上）、蓝 Z 对应 ROS -Y（朝屏幕内/外）。</span>
        </div>
      </div>
    </div>
  );
}
