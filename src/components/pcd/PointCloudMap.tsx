"use client";

import { useMemo } from "react";
import * as THREE from "three";
import type { GridMapData } from "@/types/mission";

interface PointCloudMapProps {
  gridMapData: GridMapData;
}

export function PointCloudMap({ gridMapData }: PointCloudMapProps) {
  const geometry = useMemo(() => {
    if (!gridMapData.data || gridMapData.data.length === 0) return null;
    
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(gridMapData.data);
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    
    return geom;
  }, [gridMapData]);

  if (!geometry) return null;

  return (
    <points geometry={geometry} frustumCulled={false}>
      <pointsMaterial
        size={0.05}
        sizeAttenuation
        color="#00ff00"
        transparent
        opacity={0.6}
      />
    </points>
  );
}
