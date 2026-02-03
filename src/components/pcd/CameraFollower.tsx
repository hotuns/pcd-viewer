"use client";

import { useEffect } from "react";
import { useThree } from "@react-three/fiber";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import type { DronePosition } from "@/types/mission";

interface CameraFollowerProps {
  dronePosition: DronePosition | null;
  followDrone: boolean;
  controlsRef: React.RefObject<OrbitControlsImpl | null>;
}

export function CameraFollower({
  dronePosition,
  followDrone,
  controlsRef,
}: CameraFollowerProps) {
  const { camera } = useThree();

  useEffect(() => {
    if (!followDrone || !dronePosition || !controlsRef.current) return;

    const controls = controlsRef.current;
    const dronePos = new THREE.Vector3(dronePosition.x, dronePosition.y, dronePosition.z);
    const newTarget = controls.target.clone().lerp(dronePos, 0.4);
    controls.target.copy(newTarget);
    controls.update();
  }, [dronePosition, followDrone, controlsRef, camera]);

  return null;
}
