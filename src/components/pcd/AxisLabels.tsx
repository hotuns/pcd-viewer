"use client";

import { useMemo } from "react";
import { Html, Line } from "@react-three/drei";
import * as THREE from "three";

type AxisLabelsProps = {
  origin?: THREE.Vector3;
  length?: number;
  labelOffset?: number;
  mode?: "viewer" | "ros";
  labels?: {
    x?: string;
    y?: string;
    z?: string;
  };
};

export function AxisLabels({
  origin = new THREE.Vector3(0, 0, 0),
  length = 2,
  labelOffset = 0.2,
  mode = "viewer",
  labels,
}: AxisLabelsProps) {
  const points = useMemo(() => {
    const viewerVec = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z);
    const basePoints = [
      {
        key: "x" as const,
        axis: labels?.x ?? "X",
        color: "text-red-400",
        description: "Three.js X 轴：右(+)/左(-)",
        rosHint: "对应 ROS Y 轴（左为正）",
        position: viewerVec(length, 0, 0),
      },
      {
        key: "y" as const,
        axis: labels?.y ?? "Y",
        color: "text-green-400",
        description: "Three.js Y 轴：上(+)/下(-)",
        rosHint: "对应 ROS Z 轴（上为正）",
        position: viewerVec(0, length, 0),
      },
      {
        key: "z" as const,
        axis: labels?.z ?? "Z",
        color: "text-blue-400",
        description: "Three.js Z 轴：朝前(+)/朝后(-)",
        rosHint: "对应 ROS -X 轴（后为正）",
        position: viewerVec(0, 0, length),
      },
    ];

    if (mode === "ros") {
      return basePoints.map((p) => ({
        axis:
          p.key === "x"
            ? labels?.x ?? "ROS -Y (右)"
            : p.key === "y"
            ? labels?.y ?? "ROS Z (上)"
            : labels?.z ?? "ROS -X (后)",
        color: p.color,
        description: `${p.description} · ${p.rosHint}`,
        position: p.position,
      }));
    }

    return basePoints.map((p) => ({
      axis: p.axis,
      color: p.color,
      description: p.description,
      position: p.position,
    }));
  }, [length, mode, labels]);

  return (
    <group position={origin}>
      {points.map(({ axis, color, position, description }) => {
        const labelPosition = position.clone().setLength(Math.max(position.length() + labelOffset, labelOffset));
        return (
          <Html
            key={`${axis}-${description}`}
            position={labelPosition}
            style={{
              fontSize: "11px",
              color: "white",
              textTransform: "uppercase",
              letterSpacing: "0.05em",
              userSelect: "none",
              whiteSpace: "nowrap",
            }}
            center
            title={description}
          >
            <span className={`${color} font-semibold`}>{axis}</span>
          </Html>
        );
      })}
    </group>
  );
}

export function RosAxes({ length = 2 }: { length?: number }) {
  const axes = useMemo(
    () => [
      { color: "#ef4444", dir: new THREE.Vector3(length, 0, 0) },
      { color: "#22c55e", dir: new THREE.Vector3(0, length, 0) },
      { color: "#60a5fa", dir: new THREE.Vector3(0, 0, length) },
    ],
    [length]
  );

  return (
    <group>
      {axes.map((axis, idx) => (
        <Line
          key={`viewer-axis-${idx}`}
          points={[[0, 0, 0], [axis.dir.x, axis.dir.y, axis.dir.z]]}
          color={axis.color}
          lineWidth={2}
          dashed={false}
        />
      ))}
    </group>
  );
}
