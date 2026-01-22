"use client";

import { Suspense, useMemo, useState, useCallback, useEffect, useRef } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import * as THREE from "three";
import { DroneModel } from "@/components/pcd/DroneModel";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { convertBodyOrientationToViewer, convertBodyPositionToViewer } from "@/lib/frameTransforms";
import { AxisLabels } from "@/components/pcd/AxisLabels";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader.js";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";

const degreeToRad = (deg: number) => (deg * Math.PI) / 180;

export default function DroneOrientationTool() {
  const [position, setPosition] = useState({ x: 0, y: 0, z: 0 });
  const [angles, setAngles] = useState({ roll: 0, pitch: 0, yaw: 0 });
  const [pointGeometry, setPointGeometry] = useState<THREE.BufferGeometry | null>(null);
  const [pointMesh, setPointMesh] = useState<THREE.Mesh | null>(null);
  const [cloudInfo, setCloudInfo] = useState("");
  const [loadingCloud, setLoadingCloud] = useState(false);
  const [cloudError, setCloudError] = useState<string | null>(null);
  const [viewPreset, setViewPreset] = useState<'xy' | 'xz' | 'yz' | 'iso'>('iso');

  const quaternion = useMemo(() => {
    const euler = new THREE.Euler(
      degreeToRad(angles.roll),
      degreeToRad(angles.pitch),
      degreeToRad(angles.yaw),
      "XYZ"
    );
    const quat = new THREE.Quaternion().setFromEuler(euler);
    return { x: quat.x, y: quat.y, z: quat.z, w: quat.w };
  }, [angles]);

  const viewerPosition = useMemo(() => convertBodyPositionToViewer(position), [position]);
  const viewerOrientation = useMemo(() => convertBodyOrientationToViewer(quaternion) ?? quaternion, [quaternion]);

  const pointMaterialRef = useRef<THREE.PointsMaterial | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControlsImpl | null>(null);
  const axesHelper = useMemo(() => new THREE.AxesHelper(2), []);

  const applyViewPreset = useCallback((preset: 'xy' | 'xz' | 'yz' | 'iso') => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) return;

    let normal = new THREE.Vector3(0, 0, 1);
    if (preset === 'xz') normal = new THREE.Vector3(0, 1, 0);
    if (preset === 'yz') normal = new THREE.Vector3(1, 0, 0);
    if (preset === 'iso') normal = new THREE.Vector3(1, 1, 1);

    const distance = 6;
    const target = new THREE.Vector3(0, 0, 0);
    const nextPos = target.clone().add(normal.normalize().multiplyScalar(distance));
    camera.position.copy(nextPos);
    camera.up.set(0, 1, 0);
    camera.lookAt(target);
    controls.target.copy(target);
    controls.update();
  }, []);

  useEffect(() => {
    const geometry = pointGeometry;
    const mesh = pointMesh;
    const material = pointMaterialRef.current;
    return () => {
      geometry?.dispose?.();
      if (mesh) {
        mesh.geometry?.dispose?.();
        if (Array.isArray(mesh.material)) {
          mesh.material.forEach((mat) => mat.dispose?.());
        } else {
          mesh.material?.dispose?.();
        }
      }
      material?.dispose?.();
    };
  }, [pointGeometry, pointMesh]);

  const handlePositionChange = (axis: "x" | "y" | "z", value: number) => {
    setPosition((prev) => ({ ...prev, [axis]: value }));
  };

  const handleAngleChange = (axis: "roll" | "pitch" | "yaw", value: number) => {
    setAngles((prev) => ({ ...prev, [axis]: value }));
  };

  const handlePointCloudUpload = useCallback(async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;
    setLoadingCloud(true);
    setCloudError(null);
    try {
      const ext = file.name.split(".").pop()?.toLowerCase();
      if (!ext || !["pcd", "ply"].includes(ext)) {
        throw new Error("仅支持 PCD / PLY 文件");
      }
      pointGeometry?.dispose?.();
      if (pointMesh) {
        pointMesh.geometry?.dispose?.();
        if (Array.isArray(pointMesh.material)) {
          pointMesh.material.forEach((mat) => mat.dispose?.());
        } else {
          pointMesh.material?.dispose?.();
        }
        setPointMesh(null);
      }
      const convertGeometryToViewer = (geometry: THREE.BufferGeometry) => {
        const position = geometry.getAttribute("position") as THREE.BufferAttribute | undefined;
        if (position) {
          for (let i = 0; i < position.count; i++) {
            const x = position.getX(i);
            const y = position.getY(i);
            const z = position.getZ(i);
            const converted = convertBodyPositionToViewer({ x, y, z });
            position.setXYZ(i, converted.x, converted.y, converted.z);
          }
          position.needsUpdate = true;
          geometry.computeBoundingBox();
          geometry.computeBoundingSphere();
        }
        return geometry;
      };

      if (ext === "pcd") {
        const loader = new PCDLoader();
        const buffer = await file.arrayBuffer();
        const points = loader.parse(buffer) as THREE.Points;
        const geometry = convertGeometryToViewer(points.geometry as THREE.BufferGeometry);
        setPointGeometry(geometry);
        setPointMesh(null);
        setCloudInfo(`${file.name} · ${(geometry.getAttribute("position")?.count ?? 0)} 点`);
      } else {
        const loader = new PLYLoader();
        const blobUrl = URL.createObjectURL(file);
        const geometry = await loader.loadAsync(blobUrl);
        URL.revokeObjectURL(blobUrl);
        convertGeometryToViewer(geometry);
        geometry.computeVertexNormals();
        const material = new THREE.MeshStandardMaterial({
          color: geometry.getAttribute("color") ? 0xffffff : 0xdddddd,
          vertexColors: !!geometry.getAttribute("color"),
          roughness: 0.6,
          metalness: 0.1,
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        setPointMesh(mesh);
        setPointGeometry(geometry);
        setCloudInfo(`${file.name} · ${(geometry.getAttribute("position")?.count ?? 0)} 顶点`);
      }
      setViewPreset('iso');
      applyViewPreset('iso');
    } catch (error) {
      console.error("Failed to load point cloud", error);
      setPointGeometry(null);
      setPointMesh(null);
      setCloudError(error instanceof Error ? error.message : String(error));
      setCloudInfo("");
    } finally {
      setLoadingCloud(false);
      event.target.value = "";
    }
  }, [applyViewPreset, pointGeometry, pointMesh]);

  return (
    <div className="flex flex-col gap-4 p-4 h-full">
      <div className="grid grid-cols-3 gap-3 text-sm">
        {(["x", "y", "z"] as const).map((axis) => {
          const axisLabel = axis === "x" ? "X (前后)" : axis === "y" ? "Y (左右)" : "Z (上下)";
          return (
            <div key={axis} className="space-y-1">
              <Label className="text-xs uppercase tracking-wide text-slate-500">{axisLabel}</Label>
              <Input
                type="number"
                value={position[axis]}
                step="0.1"
                onChange={(e) => handlePositionChange(axis, Number(e.target.value))}
              />
            </div>
          );
        })}
        {(["roll", "pitch", "yaw"] as const).map((axis) => (
          <div key={axis} className="space-y-1">
            <Label className="text-xs uppercase tracking-wide text-slate-500">{axis} (°)</Label>
            <Input
              type="number"
              value={angles[axis]}
              step="1"
              onChange={(e) => handleAngleChange(axis, Number(e.target.value))}
            />
          </div>
        ))}
      </div>

      <div className="flex items-center gap-3 text-xs text-slate-300">
        <label className="flex items-center gap-2">
          <span>加载点云</span>
          <Input type="file" accept=".pcd,.ply" onChange={handlePointCloudUpload} className="h-8" />
        </label>
        {loadingCloud && <span className="text-amber-400">加载中...</span>}
        {cloudInfo && <span className="text-slate-500">{cloudInfo}</span>}
        {cloudError && <span className="text-red-400">{cloudError}</span>}
      </div>

      <div className="flex-1 min-h-[500px] rounded-xl border border-slate-800 overflow-hidden bg-black">
        <div className="flex justify-end gap-2 px-4 py-2 text-[11px] text-slate-200 border-b border-slate-800 bg-slate-950/60">
          {([
            { key: 'xy', label: 'XY' },
            { key: 'xz', label: 'XZ' },
            { key: 'yz', label: 'YZ' },
            { key: 'iso', label: '45°' },
          ] as const).map((preset) => (
            <button
              key={preset.key}
              onClick={() => {
                setViewPreset(preset.key);
                applyViewPreset(preset.key);
              }}
              className={`px-2 py-1 rounded border transition-colors ${viewPreset === preset.key ? "bg-white text-slate-900 border-white" : "border-slate-600 text-slate-200 hover:bg-slate-700"}`}
            >
              {preset.label}
            </button>
          ))}
        </div>
        <Canvas
          camera={{ position: [-4, 3, 4], fov: 55 }}
          onCreated={({ camera }) => {
            cameraRef.current = camera as THREE.PerspectiveCamera;
            requestAnimationFrame(() => applyViewPreset(viewPreset));
          }}
        >
          <ambientLight intensity={0.6} />
          <directionalLight position={[3, 5, 2]} intensity={0.8} />
          <Suspense fallback={null}>
            <DroneModel
              position={[viewerPosition.x, viewerPosition.y, viewerPosition.z]}
              orientation={viewerOrientation}
            />
          </Suspense>
          <Grid args={[10, 10]} />
          <primitive object={axesHelper} />
          <AxisLabels length={2.5} labelOffset={0.1} mode="ros" />
          {pointMesh ? (
            <primitive object={pointMesh} />
          ) : pointGeometry ? (
            <points geometry={pointGeometry} frustumCulled={false}>
              <pointsMaterial ref={pointMaterialRef} size={0.03} sizeAttenuation color="#60a5fa" />
            </points>
          ) : null}
          <OrbitControls
            ref={(controls) => {
              controlsRef.current = controls;
              if (controls) {
                requestAnimationFrame(() => applyViewPreset(viewPreset));
              }
            }}
          />
        </Canvas>
      </div>
    </div>
  );
}
