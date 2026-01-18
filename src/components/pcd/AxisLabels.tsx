"use client";

import { useMemo } from "react";
import { Html, Line } from "@react-three/drei";
import * as THREE from "three";
import { convertBodyPositionToViewer } from "@/lib/frameTransforms";

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
    const toViewerVec = (vec: { x: number; y: number; z: number }) => {
      const converted = convertBodyPositionToViewer(vec);
      return new THREE.Vector3(converted.x, converted.y, converted.z);
    };
    const viewerVec = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z);

    if (mode === "ros") {
      return [
        {
          axis: labels?.x ?? "X (前)",
          color: "text-red-400",
          description: "ROS X 轴：前/后",
          position: toViewerVec({ x: length, y: 0, z: 0 }),
        },
        {
          axis: labels?.z ?? "Z (上)",
          color: "text-green-400",
          description: "ROS Z 轴：上/下",
          position: toViewerVec({ x: 0, y: 0, z: length }),
        },
        {
          axis: labels?.y ?? "Y (左)",
          color: "text-blue-400",
          description: "ROS Y 轴：左/右",
          position: toViewerVec({ x: 0, y: length, z: 0 }),
        },
      ];
    }
    return [
      { axis: labels?.x ?? "X", color: "text-red-400", description: "Three.js X", position: viewerVec(length, 0, 0) },
      { axis: labels?.y ?? "Y", color: "text-green-400", description: "Three.js Y", position: viewerVec(0, length, 0) },
      { axis: labels?.z ?? "Z", color: "text-blue-400", description: "Three.js Z", position: viewerVec(0, 0, length) },
    ];
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
  const axes = useMemo(() => {
    const toViewerVec = (vec: { x: number; y: number; z: number }) => {
      const converted = convertBodyPositionToViewer(vec);
      return new THREE.Vector3(converted.x, converted.y, converted.z);
    };
    return [
      { color: '#ef4444', dir: toViewerVec({ x: 1, y: 0, z: 0 }) },
      { color: '#60a5fa', dir: toViewerVec({ x: 0, y: 1, z: 0 }) },
      { color: '#22c55e', dir: toViewerVec({ x: 0, y: 0, z: 1 }) },
    ].map(axis => ({ ...axis, dir: axis.dir.normalize().multiplyScalar(length) }));
  }, [length]);

  return (
    <group>
      {axes.map((axis, idx) => (
        <Line
          key={`ros-axis-${idx}`}
          points={[[0, 0, 0], [axis.dir.x, axis.dir.y, axis.dir.z]]}
          color={axis.color}
          lineWidth={2}
          dashed={false}
        />
      ))}
    </group>
  );
}
