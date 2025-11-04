"use client";

import { useGLTF } from "@react-three/drei";
import * as THREE from "three";

interface DroneModelProps {
  position: [number, number, number];
  orientation?: { x: number; y: number; z: number; w: number };
}

export function DroneModel({ position, orientation }: DroneModelProps) {
  const { scene } = useGLTF('/drone/scene.gltf');
  
  // 克隆场景并确保材质正确
  const clonedScene = scene.clone();
  
  // 遍历所有材质，确保它们能正确渲染
  clonedScene.traverse((child) => {
    if (child instanceof THREE.Mesh && child.material) {
      // 如果是标准材质
      if (child.material instanceof THREE.MeshStandardMaterial) {
        child.material.needsUpdate = true;
        
        // 提升材质亮度，确保可见
        child.material.toneMapped = false;
        
        // 如果有发光贴图，增强发光强度
        if (child.material.emissiveMap) {
          child.material.emissiveIntensity = 2.5;
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
      quaternion={orientation ? [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ] : undefined}
      scale={[0.2, 0.2, 0.2]}
      rotation={[0, 0, 0]}
    >
      <primitive object={clonedScene} />
    </group>
  );
}

// 预加载GLTF模型
useGLTF.preload('/drone/scene.gltf');
