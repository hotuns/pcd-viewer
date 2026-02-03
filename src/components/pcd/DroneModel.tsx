"use client";

import { useMemo } from "react";
import { useGLTF } from "@react-three/drei";
import * as THREE from "three";

interface DroneModelProps {
  position: [number, number, number];
  orientation?: { x: number; y: number; z: number; w: number };
}

export function DroneModel({ position, orientation }: DroneModelProps) {
  const { scene } = useGLTF('/drone.glb');
  // const { scene } = useGLTF('/1drone.glb');

  const finalQuaternion = useMemo(() => {
    if (orientation) {
      const rosQuaternion = new THREE.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
      return rosQuaternion;
    }
    return new THREE.Quaternion();
  }, [orientation]);
  
  // 克隆场景并确保材质正确
  const clonedScene = scene.clone();
  
  // 遍历所有材质，确保它们能正确渲染
  clonedScene.traverse((obj) => {
    const child = obj as THREE.Mesh;
    if (child.isMesh && child.material) {
      // 如果是标准材质
      if (child.material instanceof THREE.MeshStandardMaterial) {
        child.material.needsUpdate = true;
        
        // 提升材质亮度，确保可见
        child.material.toneMapped = false;
        
        // 增加金属感和光泽度，让模型更醒目
        child.material.roughness = Math.min(child.material.roughness, 0.5);
        child.material.metalness = Math.max(child.material.metalness, 0.3);
        
        // 如果有发光贴图，增强发光强度
        if (child.material.emissiveMap) {
          child.material.emissiveIntensity = 3.0;
        } else if (child.material.emissive) {
          // 如果没有发光贴图但有发光颜色，增强一下
          child.material.emissiveIntensity = 0.5;
        }
      }
      
      // 确保网格可以投射和接收阴影
      child.castShadow = true;
      child.receiveShadow = true;
    }
  });
  
  return (
    <group 
      position={position}
      quaternion={finalQuaternion}
      scale={[0.2, 0.2, 0.2]}
      rotation={[0, 0, 0]}
    >
      <primitive object={clonedScene} />
    </group>
  );
}

// 预加载GLTF模型
useGLTF.preload('/drone.glb');
// useGLTF.preload('/1drone.glb');
