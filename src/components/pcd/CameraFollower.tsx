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
    
    // 第三人称跟随视角：相机在无人机后上方
    const followHeight = 2.5; // 相机高度偏移（米）
    const followBack = -4; // 相机后方偏移（米）
    
    // 计算目标位置和相机位置
    const targetPos = dronePos.clone();
    const cameraOffset = new THREE.Vector3(followBack, 0, followHeight);
    const desiredCameraPos = dronePos.clone().add(cameraOffset);
    
    // 平滑插值
    const currentTarget = controls.target.clone();
    const currentCameraPos = camera.position.clone();
    
    const newTarget = currentTarget.lerp(targetPos, 0.15);
    const newCameraPos = currentCameraPos.lerp(desiredCameraPos, 0.15);
    
    controls.target.copy(newTarget);
    camera.position.copy(newCameraPos);
    
    controls.update();
  }, [dronePosition, followDrone, controlsRef, camera]);

  return null;
}
