"use client";

import { useEffect, useRef } from "react";
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
  const previousPositionRef = useRef<THREE.Vector3 | null>(null);
  const headingRef = useRef<THREE.Vector3>(new THREE.Vector3(1, 0, 0));

  useEffect(() => {
    if (!followDrone) {
      previousPositionRef.current = null;
      headingRef.current.set(1, 0, 0);
    }
  }, [followDrone]);

  useEffect(() => {
    if (!followDrone || !dronePosition || !controlsRef.current) return;

    const controls = controlsRef.current;
    const dronePos = new THREE.Vector3(dronePosition.x, dronePosition.y, dronePosition.z);

    const orientationHeading = (() => {
      const ori = dronePosition.orientation;
      if (!ori) return null;
      const quat = new THREE.Quaternion(ori.x ?? 0, ori.y ?? 0, ori.z ?? 0, ori.w ?? 1).normalize();
      const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(quat);
      forward.z = 0;
      return forward.lengthSq() > 1e-6 ? forward.normalize() : null;
    })();

    let desiredHeading = headingRef.current.clone();

    if (previousPositionRef.current) {
      const movement = dronePos.clone().sub(previousPositionRef.current);
      if (movement.lengthSq() > 1e-4) {
        desiredHeading = movement.normalize();
      } else if (orientationHeading) {
        desiredHeading = orientationHeading;
      }
    } else if (orientationHeading) {
      desiredHeading = orientationHeading;
    }

    previousPositionRef.current = dronePos.clone();
    headingRef.current.lerp(desiredHeading, 0.35);

    const smoothedHeading = headingRef.current.clone();
    smoothedHeading.z = 0;
    if (smoothedHeading.lengthSq() < 1e-6) {
      smoothedHeading.set(1, 0, 0);
    }
    smoothedHeading.normalize();

    const followDistance = 6;
    const followHeight = 3;

    const cameraOffset = smoothedHeading.clone().multiplyScalar(-followDistance);
    cameraOffset.z += followHeight;

    const focusPoint = dronePos.clone().add(smoothedHeading.clone().multiplyScalar(2));
    const desiredCameraPos = dronePos.clone().add(cameraOffset);

    const currentTarget = controls.target.clone();
    const currentCameraPos = camera.position.clone();

    const newTarget = currentTarget.lerp(focusPoint, 0.3);
    const newCameraPos = currentCameraPos.lerp(desiredCameraPos, 0.3);

    controls.target.copy(newTarget);
    camera.position.copy(newCameraPos);

    controls.update();
  }, [dronePosition, followDrone, controlsRef, camera]);

  return null;
}
