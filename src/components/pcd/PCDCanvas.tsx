"use client";

import { forwardRef, useEffect, useImperativeHandle, useMemo, useRef, useState } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import * as THREE from "three";
import { PCDLoader, OrbitControls as OrbitControlsImpl } from "three-stdlib";

export type Source = { type: "file"; file: File } | { type: "url"; url: string };

export type PCDCanvasHandle = {
  fitToView: () => void;
};

export type PCDCanvasProps = {
  source?: Source | null;
  pointSize?: number;
  showGrid?: boolean;
  showAxes?: boolean;
  onLoadedAction?: (bbox: THREE.Box3) => void;
};

function SceneFitter({
  bbox,
  controlsRef,
  registerFit,
}: {
  bbox: THREE.Box3 | null;
  controlsRef: React.RefObject<OrbitControlsImpl | null>;
  registerFit: (fn: () => void) => void;
}) {
  const { camera } = useThree();
  useEffect(() => {
    const fit = () => {
      if (!bbox) return;
      const size = bbox.getSize(new THREE.Vector3());
      const center = bbox.getCenter(new THREE.Vector3());
      const radius = Math.max(size.x, size.y, size.z) * 0.75 || 1;
      const fov = (camera as THREE.PerspectiveCamera).fov * (Math.PI / 180);
      const dist = radius / Math.tan(fov / 2);
      const newPos = new THREE.Vector3(center.x + dist, center.y + dist, center.z + dist);
      camera.position.copy(newPos);
      camera.near = Math.max(0.01, dist / 100);
      camera.far = dist * 1000;
      camera.updateProjectionMatrix();
      const controls = controlsRef.current;
      if (controls) {
        controls.target.copy(center);
        controls.update();
      }
    };
    registerFit(fit);
  }, [bbox, camera, controlsRef, registerFit]);
  return null;
}

function Points({ geometry, size }: { geometry: THREE.BufferGeometry; size: number }) {
  const material = useMemo(
    () => new THREE.PointsMaterial({ size, vertexColors: !!geometry.getAttribute("color"), sizeAttenuation: true }),
    [size, geometry]
  );
  useEffect(() => () => material.dispose(), [material]);
  return <points geometry={geometry} material={material} frustumCulled={false} />;
}

export const PCDCanvas = forwardRef<PCDCanvasHandle, PCDCanvasProps>(function PCDCanvas(
  { source, pointSize = 0.01, showGrid = true, showAxes = true, onLoadedAction },
  ref
) {
  const [geom, setGeom] = useState<THREE.BufferGeometry | null>(null);
  const [bbox, setBbox] = useState<THREE.Box3 | null>(null);
  const controlsRef = useRef<OrbitControlsImpl | null>(null);
  const fitRef = useRef<(() => void) | null>(null);
  const registerFit = (fn: () => void) => {
    fitRef.current = fn;
  };
  useImperativeHandle(ref, () => ({ fitToView: () => fitRef.current?.() }), []);

  useEffect(() => {
    let cancelled = false;
    async function load() {
      if (!source) return;
      const loader = new PCDLoader();
      try {
        let points: THREE.Points | null = null;
        if (source.type === "url") {
          points = (await loader.loadAsync(source.url)) as THREE.Points;
        } else {
          const text = await source.file.text();
          const blobUrl = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
          points = (await loader.loadAsync(blobUrl)) as THREE.Points;
          URL.revokeObjectURL(blobUrl);
        }
        if (cancelled) return;
        const geometry = points.geometry as THREE.BufferGeometry;
        geometry.computeBoundingBox();
        setGeom(geometry);
        const b = geometry.boundingBox ?? new THREE.Box3().setFromObject(points);
  const clone = b.clone();
  setBbox(clone);
  onLoadedAction?.(clone);
  // auto fit after load
  setTimeout(() => fitRef.current?.(), 0);
      } catch (e) {
        console.error("Failed to load PCD:", e);
      }
    }
    load();
    return () => {
      cancelled = true;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [source]);

  return (
    <Canvas camera={{ position: [2, 2, 2], fov: 60 }} dpr={[1, 2]}>
      <color attach="background" args={["#111214"]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[5, 5, 5]} intensity={0.8} />
      {showGrid && <Grid args={[10, 10]} cellColor="#2a2a2a" sectionColor="#3a3a3a" infiniteGrid />}
      {showAxes && <axesHelper args={[1]} />}
      {geom && <Points geometry={geom} size={pointSize} />}
      <OrbitControls ref={controlsRef} makeDefault />
      <SceneFitter bbox={bbox} controlsRef={controlsRef} registerFit={registerFit} />
    </Canvas>
  );
});

export default PCDCanvas;
