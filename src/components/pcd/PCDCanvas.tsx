"use client";

import { forwardRef, useEffect, useImperativeHandle, useRef, useState, Suspense } from "react";
import { Canvas, useThree, ThreeEvent } from "@react-three/fiber";
import { Grid, OrbitControls, Line } from "@react-three/drei";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader.js";
import type { Source, DronePosition, Waypoint, PointCloudFrame, GridMapData } from "@/types/mission";
import { DroneModel } from "./DroneModel";
import { PointCloudMap } from "./PointCloudMap";
import { CameraFollower } from "./CameraFollower";

export type PCDCanvasHandle = {
  fitToView: () => void;
  orientToPlane: (plane: 'xy'|'xz'|'yz') => void;
};

export type PCDCanvasProps = {
  source?: Source | null;
  pointSize?: number;
  showGrid?: boolean;
  showAxes?: boolean;
  showSceneCloud?: boolean; // 场景点云显示开关
  colorMode?: "none" | "rgb" | "intensity" | "height"; // none 表示禁用伪着色
  roundPoints?: boolean; // 使用圆盘精灵渲染点
  onLoadedAction?: (info: { bbox: THREE.Box3; count: number }) => void;
  onLoadingChange?: (loading: boolean) => void;
  plannedPathPoints?: Array<{ x: number; y: number; z: number }>; // 用于渲染规划航线
  plannedPathVisible?: boolean;
  plannedPointSize?: number; // 航点球体大小（半径）
  plannedPathEditable?: boolean; // 是否支持在 3D 中拖拽编辑
  onPlannedPointsChange?: (points: Array<{ x: number; y: number; z: number }>) => void; // 拖拽时回传
  plannedDragPlane?: 'xy' | 'xz' | 'yz'; // 拖拽所用平面
  dronePosition?: DronePosition | null; // 无人机位置和姿态
  followDrone?: boolean; // 视角跟随飞机
  waypoints?: Waypoint[]; // 航点状态信息
  currentWaypointIndex?: number; // 当前目标航点
  realtimeFrames?: PointCloudFrame[]; // 实时点云帧数据
  showRealtimeCloud?: boolean; // 是否显示实时点云
  gridMapData?: GridMapData | null; // 网格地图数据
  showGridMap?: boolean; // 是否显示网格地图
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

function CameraOrienter({
  bbox,
  plannedPoints,
  controlsRef,
  registerOrient,
}: {
  bbox: THREE.Box3 | null;
  plannedPoints?: Array<{ x: number; y: number; z: number }>;
  controlsRef: React.RefObject<OrbitControlsImpl | null>;
  registerOrient: (fn: (plane: 'xy'|'xz'|'yz') => void) => void;
}) {
  const { camera } = useThree();
  useEffect(() => {
    const orient = (plane: 'xy'|'xz'|'yz') => {
      const controls = controlsRef.current;
      const target = controls ? controls.target.clone() : new THREE.Vector3(0,0,0);
      // 优先使用航线点包围盒
      let center = target.clone();
      let size = new THREE.Vector3(1,1,1);
      if (plannedPoints && plannedPoints.length > 0) {
        const pb = new THREE.Box3();
        for (const p of plannedPoints) {
          pb.expandByPoint(new THREE.Vector3(p.x, p.y, p.z));
        }
        if (pb.isEmpty()) {
          center = target.clone();
          size.set(1,1,1);
        } else {
          center = pb.getCenter(new THREE.Vector3());
          size = pb.getSize(new THREE.Vector3());
        }
      } else if (bbox) {
        center = bbox.getCenter(new THREE.Vector3());
        size = bbox.getSize(new THREE.Vector3());
      }
      const radius = Math.max(size.x, size.y, size.z) * 0.75 || 1;
      const fov = (camera as THREE.PerspectiveCamera).fov * (Math.PI / 180);
      const dist = radius / Math.tan(fov / 2);
      let normal = new THREE.Vector3(0,0,1); // XY -> +Z
      if (plane === 'xz') normal = new THREE.Vector3(0,1,0); // look along +Y
      if (plane === 'yz') normal = new THREE.Vector3(1,0,0); // look along +X

      // Use Z-up for intuitive top/side views
      camera.up.set(0,0,1);
      const newPos = new THREE.Vector3().copy(center).addScaledVector(normal.normalize(), dist);
      camera.position.copy(newPos);
      camera.near = Math.max(0.01, dist / 100);
      camera.far = dist * 1000;
      camera.updateProjectionMatrix();
      if (controls) {
        controls.target.copy(center);
        controls.update();
      }
    };
    registerOrient(orient);
  }, [bbox, plannedPoints, camera, controlsRef, registerOrient]);
  return null;
}

export const PCDCanvas = forwardRef<PCDCanvasHandle, PCDCanvasProps>(function PCDCanvas(
  { source, pointSize = 0.01, showGrid = true, showAxes = true, showSceneCloud = true, colorMode = "none", roundPoints = true, onLoadedAction, onLoadingChange, plannedPathPoints, plannedPathVisible = true, plannedPointSize = 0.05, plannedPathEditable = false, onPlannedPointsChange, plannedDragPlane = 'xy', dronePosition, followDrone = false, waypoints, currentWaypointIndex, realtimeFrames, showRealtimeCloud = true, gridMapData, showGridMap = false },
  ref
) {
  const [geom, setGeom] = useState<THREE.BufferGeometry | null>(null);
  const [bbox, setBbox] = useState<THREE.Box3 | null>(null);
  const controlsRef = useRef<OrbitControlsImpl | null>(null);
  const fitRef = useRef<(() => void) | null>(null);
  const orientRef = useRef<((plane: 'xy'|'xz'|'yz') => void) | null>(null);
  const [colorVersion, setColorVersion] = useState(0);
  // 生成一个圆形纹理用于点精灵（圆盘效果，边缘柔和）
  const circleTexture = useRef<THREE.Texture | null>(null);
  if (roundPoints && !circleTexture.current && typeof document !== 'undefined') {
    const size = 64;
    const canvas = document.createElement('canvas');
    canvas.width = size; canvas.height = size;
    const ctx = canvas.getContext('2d');
    if (ctx) {
      ctx.clearRect(0, 0, size, size);
      const grad = ctx.createRadialGradient(size/2, size/2, 0, size/2, size/2, size/2);
      grad.addColorStop(0.0, 'rgba(255,255,255,1)');
      grad.addColorStop(0.8, 'rgba(255,255,255,1)');
      grad.addColorStop(1.0, 'rgba(255,255,255,0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(size/2, size/2, size/2, 0, Math.PI * 2);
      ctx.fill();
    }
    const tex = new THREE.CanvasTexture(canvas);
    tex.needsUpdate = true;
    tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
    tex.magFilter = THREE.LinearFilter;
    tex.minFilter = THREE.LinearMipMapLinearFilter;
    circleTexture.current = tex;
  }
  // 使用 ref 保存回调，避免因父组件函数身份变化导致重复加载
  const onLoadedRef = useRef<typeof onLoadedAction>(onLoadedAction);
  const onLoadingChangeRef = useRef<typeof onLoadingChange>(onLoadingChange);
  const onPlannedPointsChangeRef = useRef<typeof onPlannedPointsChange>(onPlannedPointsChange);
  const dragPlaneTypeRef = useRef<typeof plannedDragPlane>(plannedDragPlane);
  useEffect(() => { onLoadedRef.current = onLoadedAction; }, [onLoadedAction]);
  useEffect(() => { onLoadingChangeRef.current = onLoadingChange; }, [onLoadingChange]);
  useEffect(() => { onPlannedPointsChangeRef.current = onPlannedPointsChange; }, [onPlannedPointsChange]);
  useEffect(() => { dragPlaneTypeRef.current = plannedDragPlane; }, [plannedDragPlane]);
  const draggingIdx = useRef<number | null>(null);
  const dragPlane = useRef<THREE.Plane | null>(null);
  const fixedVal = useRef<number>(0);
  const [selectedIdx, setSelectedIdx] = useState<number | null>(null);

  // Esc 取消选中（并终止拖拽）
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        // 取消选中和拖拽
        draggingIdx.current = null;
        dragPlane.current = null;
        fixedVal.current = 0;
        setSelectedIdx(null);
        if (controlsRef.current) controlsRef.current.enabled = true;
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);

  const registerFit = (fn: () => void) => {
    fitRef.current = fn;
  };
  const registerOrient = (fn: (plane: 'xy'|'xz'|'yz') => void) => {
    orientRef.current = fn;
  };
  useImperativeHandle(ref, () => ({
    fitToView: () => fitRef.current?.(),
    orientToPlane: (plane: 'xy'|'xz'|'yz') => orientRef.current?.(plane),
  }), []);

  useEffect(() => {
    let cancelled = false;
    async function load() {
      if (!source) {
        setGeom(null);
        setBbox(null);
        return;
      }
      const loader = new PCDLoader();
      try {
        onLoadingChangeRef.current?.(true);
        let points: THREE.Points | null = null;
        if (source.type === "url") {
          points = (await loader.loadAsync(source.url)) as THREE.Points;
        } else {
          const blobUrl = URL.createObjectURL(source.file);
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
        const pos = geometry.getAttribute("position");
        const count = pos?.count ?? 0;
        onLoadedRef.current?.({ bbox: clone, count });
        // auto fit
        setTimeout(() => fitRef.current?.(), 0);
      } catch (e) {
        console.error("Failed to load PCD:", e);
      } finally {
        onLoadingChangeRef.current?.(false);
      }
    }
    load();
    return () => {
      cancelled = true;
    };
  }, [source]);

  // 根据模式为点云生成颜色（禁用时恢复原有颜色或移除自定义颜色）
  useEffect(() => {
    if (!geom) return;
    const g = geom;

    // 禁用：恢复原始颜色或移除我们生成的颜色
    if (colorMode === "none") {
      const saved = (g.userData && g.userData._origColorSaved) as boolean | undefined;
      if (saved) {
        const orig = g.userData._origColorAttr as THREE.BufferAttribute | null | undefined;
        if (orig) {
          g.setAttribute("color", orig);
        } else {
          g.deleteAttribute("color");
        }
        g.userData._coloredBy = undefined;
        setColorVersion(v => v + 1);
      }
      return;
    }

    // 着色模式：intensity / height / rgb
    const mode: "rgb" | "intensity" | "height" = colorMode === "rgb" ? "rgb" : colorMode === "intensity" ? "intensity" : "height";

    const count = (g.getAttribute("position") as THREE.BufferAttribute | undefined)?.count ?? 0;
    if (count === 0) return;

    const colors = new Float32Array(count * 3);
    const setRGB = (i: number, r: number, g: number, b: number) => {
      const o = i * 3; colors[o] = r; colors[o + 1] = g; colors[o + 2] = b;
    };

    const pos = g.getAttribute("position") as THREE.BufferAttribute | undefined;

    // 简单分段渐变：蓝-青-绿-黄-红（0..1）
    const mapScalarToRGB = (v01: number) => {
      const v = Math.min(1, Math.max(0, v01));
      if (v < 0.25) { // blue -> cyan
        const t = v / 0.25; return [0, t, 1];
      } else if (v < 0.5) { // cyan -> green
        const t = (v - 0.25) / 0.25; return [0, 1, 1 - t];
      } else if (v < 0.75) { // green -> yellow
        const t = (v - 0.5) / 0.25; return [t, 1, 0];
      } else { // yellow -> red
        const t = (v - 0.75) / 0.25; return [1, 1 - t, 0];
      }
    };

    // 保存一次原始颜色，便于禁用时恢复
    if (!g.userData._origColorSaved) {
      g.userData._origColorAttr = (g.getAttribute("color") as THREE.BufferAttribute | undefined) ?? null;
      g.userData._origColorSaved = true;
    }

    if (mode === "intensity") {
      const inten = g.getAttribute("intensity") as THREE.BufferAttribute | undefined;
      if (!inten) return; // 没有强度则不改色
      // 计算 min/max
      let min = Infinity, max = -Infinity;
      for (let i = 0; i < inten.count; i++) {
        const v = inten.getX(i); if (v < min) min = v; if (v > max) max = v;
      }
      const span = max - min || 1;
      for (let i = 0; i < count; i++) {
        const v = (inten.getX(i) - min) / span;
        const [r, gC, b] = mapScalarToRGB(v);
        setRGB(i, r, gC, b);
      }
      g.setAttribute("color", new THREE.BufferAttribute(colors, 3));
      g.attributes.color.needsUpdate = true;
      g.userData._coloredBy = "intensity";
      // 触发一次刷新，使 pointsMaterial.vertexColors 生效
      setColorVersion(v => v + 1);
    } else if (mode === "height" && pos) {
      // 以 z 高度着色
      let minZ = Infinity, maxZ = -Infinity;
      for (let i = 0; i < count; i++) {
        const z = pos.getZ(i); if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
      }
      const spanZ = maxZ - minZ || 1;
      for (let i = 0; i < count; i++) {
        const z = (pos.getZ(i) - minZ) / spanZ;
        const [r, gC, b] = mapScalarToRGB(z);
        setRGB(i, r, gC, b);
      }
      g.setAttribute("color", new THREE.BufferAttribute(colors, 3));
      g.attributes.color.needsUpdate = true;
      g.userData._coloredBy = "height";
      setColorVersion(v => v + 1);
    }
  }, [geom, colorMode]);

  return (
    <div style={{ position: "relative", width: "100%", height: "100%" }}>
      <Canvas 
        camera={{ position: [2, 2, 2], fov: 60 }} 
        dpr={[1, 2]}
        gl={{ 
          toneMapping: THREE.ACESFilmicToneMapping,
          toneMappingExposure: 1.5,
          outputColorSpace: THREE.SRGBColorSpace
        }}
      >
        <color attach="background" args={["#111214"]} />
        <ambientLight intensity={1.0} />
        <directionalLight position={[5, 5, 5]} intensity={1.5} />
        <directionalLight position={[-5, -5, 5]} intensity={0.8} />
        <pointLight position={[0, 0, 10]} intensity={0.8} />
        <hemisphereLight args={["#ffffff", "#444444", 0.6]} />
        {showGrid && (
          <Grid args={[10, 10]} cellColor="#2a2a2a" sectionColor="#3a3a3a" infiniteGrid />
        )}
        {showAxes && <axesHelper args={[50]} />}
        {showSceneCloud && geom && (
          <points key={colorVersion} geometry={geom} frustumCulled={false}>
            <pointsMaterial
              size={pointSize}
              sizeAttenuation
              vertexColors={!!geom.getAttribute("color")}
              map={roundPoints ? circleTexture.current ?? undefined : undefined}
              transparent={roundPoints}
              alphaTest={roundPoints ? 0.5 : 0}
              depthWrite={!roundPoints}
            />
          </points>
        )}

        {/* 规划航线渲染：折线 + 航点球（可选编辑）*/}
        {plannedPathVisible && Array.isArray(plannedPathPoints) && plannedPathPoints.length > 0 && (
          <group
            onPointerMove={(e: ThreeEvent<PointerEvent>) => {
              if (draggingIdx.current === null || !plannedPathEditable) return;
              const plane = dragPlane.current;
              const pts = plannedPathPoints;
              if (!plane || !Array.isArray(pts) || pts.length === 0) return;
              if (!e.ray) return;
              const out = new THREE.Vector3();
              if (!e.ray.intersectPlane(plane, out)) return;
              const idx = draggingIdx.current;
              if (idx === null || idx < 0 || idx >= pts.length) return;
              const next = pts.slice();
              const planeType = dragPlaneTypeRef.current;
              if (planeType === 'xy') {
                next[idx] = { x: out.x, y: out.y, z: fixedVal.current };
              } else if (planeType === 'xz') {
                next[idx] = { x: out.x, y: fixedVal.current, z: out.z };
              } else { // yz
                next[idx] = { x: fixedVal.current, y: out.y, z: out.z };
              }
              onPlannedPointsChangeRef.current?.(next);
              e.stopPropagation();
            }}
            onPointerUp={(e: ThreeEvent<PointerEvent>) => {
              if (draggingIdx.current !== null) {
                draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                if (controlsRef.current) controlsRef.current.enabled = true;
                e.stopPropagation();
              }
            }}
          >
            {/* 折线（全路径）*/}
            <Line
              points={plannedPathPoints.map(p => [p.x, p.y, p.z]) as [number, number, number][]}
              color="#2dd4bf"
              lineWidth={2}
              dashed={false}
              depthTest
              opacity={0.9}
              transparent
            />
            {/* 已完成路径段：起点 -> 最后一个 completed 航点（叠加更醒目的绿色）*/}
            {(() => {
              if (!Array.isArray(waypoints) || waypoints.length === 0) return null;
              // 取最大 completed 索引
              const lastCompleted = waypoints.reduce((m, w) => (w.status === 'completed' ? Math.max(m, w.index ?? 0) : m), -1);
              const endIdx = Math.min(lastCompleted, plannedPathPoints.length - 1);
              if (endIdx >= 1) {
                const pts = plannedPathPoints.slice(0, endIdx + 1).map(p => [p.x, p.y, p.z]) as [number, number, number][];
                return (
                  <Line
                    points={pts}
                    color="#22c55e"
                    lineWidth={3}
                    dashed={false}
                    depthTest
                    opacity={1}
                    transparent={false}
                  />
                );
              }
              return null;
            })()}
            {/* 航点 */}
            {plannedPathPoints.map((p, i) => {
              // 获取航点状态和颜色
              const waypoint = waypoints?.find(w => w.index === i);
              const getWaypointColor = () => {
                if (selectedIdx === i) return "#f59e0b"; // 选中时为橙色
                
                if (waypoint) {
                  switch (waypoint.status) {
                    case 'completed': return "#22c55e"; // 绿色 - 已完成
                    case 'active': return "#f59e0b"; // 橙色 - 当前目标
                    case 'pending': return "#60a5fa"; // 蓝色 - 待执行
                    case 'skipped': return "#6b7280"; // 灰色 - 已跳过
                  }
                }
                
                // 默认颜色（编辑模式）
                if (i === 0) return "#22c55e"; // 起点绿色
                if (i === plannedPathPoints.length - 1) return "#ef4444"; // 终点红色
                return "#60a5fa"; // 中间点蓝色
              };
              
              const waypointColor = getWaypointColor();
              const isActive = waypoint?.status === 'active' || currentWaypointIndex === i;
              
              return (
                <mesh
                  key={i}
                  position={[p.x, p.y, p.z] as [number, number, number]}
                  onDoubleClick={(e: ThreeEvent<MouseEvent>) => {
                    if (!plannedPathEditable) return;
                    e.stopPropagation();
                    // 双击当前小球：若已选中则取消选中；否则选中
                    setSelectedIdx(prev => (prev === i ? null : i));
                    // 结束可能的拖拽
                    draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                    if (controlsRef.current) controlsRef.current.enabled = true;
                  }}
                  onPointerDown={(e: ThreeEvent<PointerEvent>) => {
                  if (!plannedPathEditable) return;
                    // 仅左键参与拖拽/删除；右键留给相机
                    if (e.button !== 0) return;
                  if (e.altKey) {
                    // Alt+Click 删除该点
                    e.stopPropagation();
                    const next = plannedPathPoints.slice();
                    next.splice(i, 1);
                    onPlannedPointsChangeRef.current?.(next);
                    return;
                  }
                  // 只有已双击选中的点，才能开始拖拽
                  if (selectedIdx !== i) return;
                  // 开始拖拽
                  e.stopPropagation();
                  draggingIdx.current = i;
                  const planeType = dragPlaneTypeRef.current;
                  if (planeType === 'xy') {
                    fixedVal.current = p.z;
                    dragPlane.current = new THREE.Plane(new THREE.Vector3(0, 0, 1), -p.z);
                  } else if (planeType === 'xz') {
                    fixedVal.current = p.y;
                    dragPlane.current = new THREE.Plane(new THREE.Vector3(0, 1, 0), -p.y);
                  } else { // yz
                    fixedVal.current = p.x;
                    dragPlane.current = new THREE.Plane(new THREE.Vector3(1, 0, 0), -p.x);
                  }
                  if (controlsRef.current) controlsRef.current.enabled = false;
                  }}
                >
                <sphereGeometry args={[Math.max(0.001, plannedPointSize * (isActive ? 1.3 : 1)), 16, 16]} />
                <meshStandardMaterial
                  color={waypointColor}
                  emissive={selectedIdx === i || isActive ? new THREE.Color(waypointColor) : new THREE.Color('black')}
                  emissiveIntensity={selectedIdx === i ? 0.3 : (isActive ? 0.2 : 0)}
                />
                </mesh>
              );
            })}
          </group>
        )}

        {/* 编辑模式下：在参考平面上 Shift+点击添加新点（参考平面按拖拽平面选择，固定坐标取末点或 0） */}
        {plannedPathVisible && plannedPathEditable && draggingIdx.current === null && (
          (() => {
            const last = plannedPathPoints && plannedPathPoints.length > 0 ? plannedPathPoints[plannedPathPoints.length - 1] : { x: 0, y: 0, z: 0 };
            const planeType = dragPlaneTypeRef.current;
            if (planeType === 'xy') {
              return (
                <mesh
                  position={[0, 0, last.z] as [number, number, number]}
                  onPointerDown={(e: ThreeEvent<PointerEvent>) => {
                    if (!e.shiftKey) return; e.stopPropagation();
                    const p = e.point; const next = (plannedPathPoints ?? []).slice();
                    next.push({ x: p.x, y: p.y, z: last.z });
                    onPlannedPointsChangeRef.current?.(next);
                  }}
                >
                  <planeGeometry args={[10000, 10000]} />
                  <meshBasicMaterial color={"#ffffff"} transparent opacity={0.0} depthWrite={false} />
                </mesh>
              );
            } else if (planeType === 'xz') {
              return (
                <mesh
                  position={[0, last.y, 0] as [number, number, number]}
                  rotation={[-Math.PI / 2, 0, 0]}
                  onPointerDown={(e: ThreeEvent<PointerEvent>) => {
                    if (!e.shiftKey) return; e.stopPropagation();
                    const p = e.point; const next = (plannedPathPoints ?? []).slice();
                    next.push({ x: p.x, y: last.y, z: p.z });
                    onPlannedPointsChangeRef.current?.(next);
                  }}
                >
                  <planeGeometry args={[10000, 10000]} />
                  <meshBasicMaterial color={"#ffffff"} transparent opacity={0.0} depthWrite={false} />
                </mesh>
              );
            } else {
              return (
                <mesh
                  position={[last.x, 0, 0] as [number, number, number]}
                  rotation={[0, Math.PI / 2, 0]}
                  onPointerDown={(e: ThreeEvent<PointerEvent>) => {
                    if (!e.shiftKey) return; e.stopPropagation();
                    const p = e.point; const next = (plannedPathPoints ?? []).slice();
                    next.push({ x: last.x, y: p.y, z: p.z });
                    onPlannedPointsChangeRef.current?.(next);
                  }}
                >
                  <planeGeometry args={[10000, 10000]} />
                  <meshBasicMaterial color={"#ffffff"} transparent opacity={0.0} depthWrite={false} />
                </mesh>
              );
            }
          })()
        )}

        {/* 可视化当前拖拽平面（仅网格可见，用于拖拽捕获）*/}
        {plannedPathVisible && plannedPathEditable && (
          (() => {
            const last = plannedPathPoints && plannedPathPoints.length > 0 ? plannedPathPoints[plannedPathPoints.length - 1] : { x: 0, y: 0, z: 0 };
            const planeType = dragPlaneTypeRef.current;
            const size = 2000;
            if (planeType === 'xy') {
              const z = draggingIdx.current !== null ? fixedVal.current : last.z;
              return (
                <mesh
                  position={[0, 0, z] as [number, number, number]}
                  onPointerDown={(e: ThreeEvent<PointerEvent>) => {
                    if (!plannedPathEditable) return;
                    if (e.button !== 0) return; // 右键给相机
                    if (selectedIdx === null) return; // 未选中则不进入拖拽
                    const idx = selectedIdx;
                    if (!plannedPathPoints || idx < 0 || idx >= plannedPathPoints.length) return;
                    e.stopPropagation();
                    draggingIdx.current = idx;
                    const pt = plannedPathPoints[idx];
                    fixedVal.current = pt.z;
                    dragPlane.current = new THREE.Plane(new THREE.Vector3(0, 0, 1), -pt.z);
                    if (controlsRef.current) controlsRef.current.enabled = false;
                  }}
                  onPointerMove={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current === null || !plannedPathEditable) return;
                    const plane = dragPlane.current;
                    const pts = plannedPathPoints;
                    if (!plane || !Array.isArray(pts) || pts.length === 0) return;
                    if (!e.ray) return;
                    const out = new THREE.Vector3();
                    if (!e.ray.intersectPlane(plane, out)) return;
                    const idx = draggingIdx.current;
                    if (idx === null || idx < 0 || idx >= pts.length) return;
                    const next = pts.slice();
                    const pt = { x: out.x, y: out.y, z: fixedVal.current };
                    next[idx] = pt;
                    onPlannedPointsChangeRef.current?.(next);
                    e.stopPropagation();
                  }}
                  onPointerUp={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current !== null) {
                      draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                      if (controlsRef.current) controlsRef.current.enabled = true;
                      e.stopPropagation();
                    }
                  }}
                  onPointerLeave={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current !== null) {
                      draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                      if (controlsRef.current) controlsRef.current.enabled = true;
                      e.stopPropagation();
                    }
                  }}
                >
                  <planeGeometry args={[size, size]} />
                  {/* 仅保留网格可视，平面本体透明 */}
                  <meshBasicMaterial color={"#22d3ee"} transparent opacity={0.0} depthWrite={false} side={THREE.DoubleSide} />
                  {/* 网格线（通过 gridHelper 旋转到 XY） */}
                  <primitive
                    object={new THREE.GridHelper(size, 50, new THREE.Color('#06b6d4'), new THREE.Color('#67e8f9'))}
                    rotation={[Math.PI / 2, 0, 0]}
                    position={[0, 0, 0]}
                  />
                </mesh>
              );
            } else if (planeType === 'xz') {
              const y = draggingIdx.current !== null ? fixedVal.current : last.y;
              return (
                <mesh
                  position={[0, y, 0] as [number, number, number]}
                  rotation={[-Math.PI/2,0,0]}
                  onPointerDown={(e: ThreeEvent<PointerEvent>) => {
                    if (!plannedPathEditable) return;
                    if (e.button !== 0) return;
                    if (selectedIdx === null) return;
                    const idx = selectedIdx;
                    if (!plannedPathPoints || idx < 0 || idx >= plannedPathPoints.length) return;
                    e.stopPropagation();
                    draggingIdx.current = idx;
                    const pt = plannedPathPoints[idx];
                    fixedVal.current = pt.y;
                    dragPlane.current = new THREE.Plane(new THREE.Vector3(0, 1, 0), -pt.y);
                    if (controlsRef.current) controlsRef.current.enabled = false;
                  }}
                  onPointerMove={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current === null || !plannedPathEditable) return;
                    const plane = dragPlane.current;
                    const pts = plannedPathPoints;
                    if (!plane || !Array.isArray(pts) || pts.length === 0) return;
                    if (!e.ray) return;
                    const out = new THREE.Vector3();
                    if (!e.ray.intersectPlane(plane, out)) return;
                    const idx = draggingIdx.current;
                    if (idx === null || idx < 0 || idx >= pts.length) return;
                    const next = pts.slice();
                    const pt = { x: out.x, y: fixedVal.current, z: out.z };
                    next[idx] = pt;
                    onPlannedPointsChangeRef.current?.(next);
                    e.stopPropagation();
                  }}
                  onPointerUp={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current !== null) {
                      draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                      if (controlsRef.current) controlsRef.current.enabled = true;
                      e.stopPropagation();
                    }
                  }}
                  onPointerLeave={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current !== null) {
                      draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                      if (controlsRef.current) controlsRef.current.enabled = true;
                      e.stopPropagation();
                    }
                  }}
                >
                  <planeGeometry args={[size, size]} />
                  {/* 仅保留网格可视，平面本体透明 */}
                  <meshBasicMaterial color={"#22d3ee"} transparent opacity={0.0} depthWrite={false} side={THREE.DoubleSide} />
                  <primitive
                    object={new THREE.GridHelper(size, 50, new THREE.Color('#06b6d4'), new THREE.Color('#67e8f9'))}
                  />
                </mesh>
              );
            } else {
              const x = draggingIdx.current !== null ? fixedVal.current : last.x;
              return (
                <mesh
                  position={[x, 0, 0] as [number, number, number]}
                  rotation={[0,Math.PI/2,0]}
                  onPointerDown={(e: ThreeEvent<PointerEvent>) => {
                    if (!plannedPathEditable) return;
                    if (e.button !== 0) return;
                    if (selectedIdx === null) return;
                    const idx = selectedIdx;
                    if (!plannedPathPoints || idx < 0 || idx >= plannedPathPoints.length) return;
                    e.stopPropagation();
                    draggingIdx.current = idx;
                    const pt = plannedPathPoints[idx];
                    fixedVal.current = pt.x;
                    dragPlane.current = new THREE.Plane(new THREE.Vector3(1, 0, 0), -pt.x);
                    if (controlsRef.current) controlsRef.current.enabled = false;
                  }}
                  onPointerMove={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current === null || !plannedPathEditable) return;
                    const plane = dragPlane.current;
                    const pts = plannedPathPoints;
                    if (!plane || !Array.isArray(pts) || pts.length === 0) return;
                    if (!e.ray) return;
                    const out = new THREE.Vector3();
                    if (!e.ray.intersectPlane(plane, out)) return;
                    const idx = draggingIdx.current;
                    if (idx === null || idx < 0 || idx >= pts.length) return;
                    const next = pts.slice();
                    const pt = { x: fixedVal.current, y: out.y, z: out.z };
                    next[idx] = pt;
                    onPlannedPointsChangeRef.current?.(next);
                    e.stopPropagation();
                  }}
                  onPointerUp={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current !== null) {
                      draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                      if (controlsRef.current) controlsRef.current.enabled = true;
                      e.stopPropagation();
                    }
                  }}
                  onPointerLeave={(e: ThreeEvent<PointerEvent>) => {
                    if (draggingIdx.current !== null) {
                      draggingIdx.current = null; dragPlane.current = null; fixedVal.current = 0;
                      if (controlsRef.current) controlsRef.current.enabled = true;
                      e.stopPropagation();
                    }
                  }}
                >
                  <planeGeometry args={[size, size]} />
                  {/* 仅保留网格可视，平面本体透明 */}
                  <meshBasicMaterial color={"#22d3ee"} transparent opacity={0.0} depthWrite={false} side={THREE.DoubleSide} />
                  <primitive
                    object={new THREE.GridHelper(size, 50, new THREE.Color('#06b6d4'), new THREE.Color('#67e8f9'))}
                  />
                </mesh>
              );
            }
          })()
        )}

        {/* 点云地图渲染（来自ROS PointCloud2消息） */}
        {showGridMap && gridMapData && gridMapData.data && gridMapData.data.length > 0 && (
          <PointCloudMap gridMapData={gridMapData} />
        )}

        {/* 无人机渲染 */}
        {dronePosition && (
          <Suspense fallback={
            <mesh position={[dronePosition.x, dronePosition.y, dronePosition.z]}>
              <boxGeometry args={[0.2, 0.2, 0.1]} />
              <meshStandardMaterial color="#ff6b6b" />
            </mesh>
          }>
            <DroneModel 
              position={[dronePosition.x, dronePosition.y, dronePosition.z]}
              orientation={dronePosition.orientation}
            />
          </Suspense>
        )}

        <OrbitControls ref={controlsRef} makeDefault />
        <SceneFitter bbox={bbox} controlsRef={controlsRef} registerFit={registerFit} />
        <CameraOrienter bbox={bbox} plannedPoints={plannedPathPoints} controlsRef={controlsRef} registerOrient={registerOrient} />
        <CameraFollower dronePosition={dronePosition ?? null} followDrone={followDrone} controlsRef={controlsRef} />
      </Canvas>
      {plannedPathVisible && plannedPathEditable && (
        (() => {
          const last = plannedPathPoints && plannedPathPoints.length > 0 ? plannedPathPoints[plannedPathPoints.length - 1] : { x: 0, y: 0, z: 0 };
          const planeType = dragPlaneTypeRef.current;
          let planeLabel = "";
          if (planeType === 'xy') {
            const z = draggingIdx.current !== null ? fixedVal.current : last.z;
            planeLabel = `XY, Z=${z.toFixed(2)}`;
          } else if (planeType === 'xz') {
            const y = draggingIdx.current !== null ? fixedVal.current : last.y;
            planeLabel = `XZ, Y=${y.toFixed(2)}`;
          } else {
            const x = draggingIdx.current !== null ? fixedVal.current : last.x;
            planeLabel = `YZ, X=${x.toFixed(2)}`;
          }
          // 选中点坐标（若有选中）
          let pointLabel: string | null = null;
          if (typeof selectedIdx === 'number' && plannedPathPoints && selectedIdx >= 0 && selectedIdx < plannedPathPoints.length) {
            const pt = plannedPathPoints[selectedIdx];
            pointLabel = `P${selectedIdx}: X=${pt.x.toFixed(3)}, Y=${pt.y.toFixed(3)}, Z=${pt.z.toFixed(3)}`;
          }
          return (
            <div style={{ position: 'absolute', right: 12, bottom: 12, pointerEvents: 'none', display: 'flex', flexDirection: 'column', alignItems: 'flex-end', gap: 6 }}>
              <div style={{ background: 'rgba(34,211,238,0.85)', color: '#0b1324', padding: '4px 8px', borderRadius: 8, fontSize: 12, fontFamily: 'monospace', boxShadow: '0 2px 6px rgba(0,0,0,0.25)' }}>
                {planeLabel}
              </div>
              {pointLabel && (
                <div style={{ background: 'rgba(17,24,39,0.85)', color: '#e5e7eb', padding: '4px 8px', borderRadius: 8, fontSize: 12, fontFamily: 'monospace', boxShadow: '0 2px 6px rgba(0,0,0,0.25)' }}>
                  {pointLabel}
                </div>
              )}
            </div>
          );
        })()
      )}
    </div>
  );
});

export default PCDCanvas;
 
