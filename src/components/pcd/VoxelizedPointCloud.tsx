"use client";

import { useMemo } from "react";
import * as THREE from "three";
import { voxelizePointCloud } from "@/lib/voxelizer";

interface VoxelizedPointCloudProps {
  geometry: THREE.BufferGeometry;
  voxelSize: number;
}

/**
 * 体素化点云渲染组件
 * 将点云渲染为密集的小立方体，形成实体化效果
 * 使用固定的浅青色，在深色背景下清晰可见
 */
export function VoxelizedPointCloud({ 
  geometry, 
  voxelSize
}: VoxelizedPointCloudProps) {
  const voxels = useMemo(() => {
    console.log('Voxelizing point cloud with voxel size:', voxelSize);
    const voxelData = voxelizePointCloud(geometry, voxelSize);
    console.log(`Created ${voxelData.length} voxels from point cloud`);
    return voxelData;
  }, [geometry, voxelSize]);

  // 创建实例化网格
  const instancedMesh = useMemo(() => {
    if (voxels.length === 0) return null;

    // 创建立方体几何体
    const boxGeometry = new THREE.BoxGeometry(voxelSize, voxelSize, voxelSize);
    
    // 使用深青色，在深色背景下清晰但不刺眼
    const voxelColor = new THREE.Color(0x2d8a7f); // 深青色
    
    // 创建实例化网格 - 使用固定颜色，不使用 vertexColors
    const mesh = new THREE.InstancedMesh(
      boxGeometry,
      new THREE.MeshStandardMaterial({
        color: voxelColor,
        roughness: 0.6,
        metalness: 0.15,
        emissive: new THREE.Color(0x0a3030),
        emissiveIntensity: 0.15,
        flatShading: false,
      }),
      voxels.length
    );
    
    // 启用阴影
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    // 设置每个实例的位置
    const matrix = new THREE.Matrix4();
    
    for (let i = 0; i < voxels.length; i++) {
      const voxel = voxels[i];
      matrix.setPosition(voxel.position.x, voxel.position.y, voxel.position.z);
      mesh.setMatrixAt(i, matrix);
    }

    mesh.instanceMatrix.needsUpdate = true;

    return mesh;
  }, [voxels, voxelSize]);

  if (!instancedMesh) return null;

  return <primitive object={instancedMesh} />;
}
