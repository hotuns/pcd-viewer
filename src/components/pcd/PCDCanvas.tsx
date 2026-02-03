"use client";

import { forwardRef, useCallback, useEffect, useImperativeHandle, useMemo, useRef, useState, Suspense } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { Grid, OrbitControls, Line, TransformControls } from "@react-three/drei";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl, TransformControls as TransformControlsImpl } from "three-stdlib";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader.js";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import type { Source, DronePosition, Waypoint, PlannedPoint } from "@/types/mission";
import { DroneModel } from "./DroneModel";
import { CameraFollower } from "./CameraFollower";
import { VoxelizedPointCloud } from "./VoxelizedPointCloud";
import { AxisLabels, RosAxes } from "./AxisLabels";
import { convertBodyPositionToViewer } from "@/lib/frameTransforms";

const convertGeometryPositionsToViewer = (geometry: THREE.BufferGeometry) => {
  const positionAttr = geometry.getAttribute("position") as THREE.BufferAttribute | undefined;
  if (positionAttr) {
    for (let i = 0; i < positionAttr.count; i++) {
      const converted = convertBodyPositionToViewer({
        x: positionAttr.getX(i),
        y: positionAttr.getY(i),
        z: positionAttr.getZ(i),
      });
      positionAttr.setXYZ(i, converted.x, converted.y, converted.z);
    }
    positionAttr.needsUpdate = true;
  }

  const normalAttr = geometry.getAttribute("normal") as THREE.BufferAttribute | undefined;
  if (normalAttr) {
    for (let i = 0; i < normalAttr.count; i++) {
      const converted = convertBodyPositionToViewer({
        x: normalAttr.getX(i),
        y: normalAttr.getY(i),
        z: normalAttr.getZ(i),
      });
      const len = Math.hypot(converted.x, converted.y, converted.z) || 1;
      normalAttr.setXYZ(i, converted.x / len, converted.y / len, converted.z / len);
    }
    normalAttr.needsUpdate = true;
  }

  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  return geometry;
};

const downsampleGeometry = (geometry: THREE.BufferGeometry, voxelSize = 0.2) => {
  const positionAttr = geometry.getAttribute("position") as THREE.BufferAttribute | undefined;
  if (!positionAttr) return geometry;
  const colorAttr = geometry.getAttribute("color") as THREE.BufferAttribute | undefined;
  const normalAttr = geometry.getAttribute("normal") as THREE.BufferAttribute | undefined;
  const buckets = new Map<string, { position: [number, number, number]; color?: [number, number, number]; normal?: [number, number, number] }>();

  for (let i = 0; i < positionAttr.count; i++) {
    const x = positionAttr.getX(i);
    const y = positionAttr.getY(i);
    const z = positionAttr.getZ(i);
    const key = `${Math.round(x / voxelSize)}_${Math.round(y / voxelSize)}_${Math.round(z / voxelSize)}`;
    if (buckets.has(key)) continue;
    const color = colorAttr ? [colorAttr.getX(i), colorAttr.getY(i), colorAttr.getZ(i)] as [number, number, number] : undefined;
    const normal = normalAttr ? [normalAttr.getX(i), normalAttr.getY(i), normalAttr.getZ(i)] as [number, number, number] : undefined;
    buckets.set(key, { position: [x, y, z], color, normal });
  }

  if (buckets.size === positionAttr.count) return geometry;

  const positions = new Float32Array(buckets.size * 3);
  const colors = colorAttr ? new Float32Array(buckets.size * 3) : null;
  const normals = normalAttr ? new Float32Array(buckets.size * 3) : null;
  let idx = 0;
  buckets.forEach(({ position, color, normal }) => {
    positions[idx * 3] = position[0];
    positions[idx * 3 + 1] = position[1];
    positions[idx * 3 + 2] = position[2];
    if (colors && color) {
      colors[idx * 3] = color[0];
      colors[idx * 3 + 1] = color[1];
      colors[idx * 3 + 2] = color[2];
    }
    if (normals && normal) {
      normals[idx * 3] = normal[0];
      normals[idx * 3 + 1] = normal[1];
      normals[idx * 3 + 2] = normal[2];
    }
    idx++;
  });

  const simplified = new THREE.BufferGeometry();
  simplified.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  if (colors) simplified.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  if (normals) simplified.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
  simplified.computeBoundingBox();
  simplified.computeBoundingSphere();
  geometry.dispose();
  return simplified;
};

export type PCDCanvasHandle = {
  fitToView: () => void;
  zoomToCenter: () => void;
  orientToPlane: (plane: 'xy'|'xz'|'yz'|'iso') => void;
};

export type PCDCanvasProps = {
  source?: Source | null;
  livePointClouds?: Float32Array[];
  pointSize?: number;
  performanceMode?: boolean;
  showGrid?: boolean;
  showAxes?: boolean;
  showSceneCloud?: boolean; // 场景点云显示开关
  colorMode?: "none" | "rgb" | "intensity" | "height"; // none 表示禁用伪着色
  roundPoints?: boolean; // 使用圆盘精灵渲染点
  voxelSize?: number; // 体素大小（米）
  onLoadedAction?: (info: { bbox: THREE.Box3; count: number }) => void;
  onLoadingChange?: (loading: boolean) => void;
  sceneRenderMode?: 'points' | 'mesh' | 'voxel';
  plannedPathPoints?: PlannedPoint[]; // 用于渲染编辑航线
  plannedPathVisible?: boolean;
  plannedPointSize?: number; // 航点球体大小（半径）
  plannedPathLineWidth?: number;
  plannedPathEditable?: boolean; // 是否支持在 3D 中拖拽编辑
  onPlannedPointsChange?: (points: PlannedPoint[]) => void; // 拖拽时回传
  selectedPointIndex?: number | null;
  onSelectPoint?: (index: number | null) => void;
  predictedTrajectoryPoints?: PlannedPoint[];
  dronePosition?: DronePosition | null; // 无人机位置和姿态
  followDrone?: boolean; // 视角跟随飞机
  waypoints?: Waypoint[]; // 航点状态信息
  currentWaypointIndex?: number; // 当前目标航点
};

function SceneFitter({
  bbox,
  plannedPoints,
  controlsRef,
  registerFit,
}: {
  bbox: THREE.Box3 | null;
  plannedPoints?: PlannedPoint[];
  controlsRef: React.RefObject<OrbitControlsImpl | null>;
  registerFit: (fn: () => void) => void;
}) {
  const { camera } = useThree();
  useEffect(() => {
    const fit = () => {
      const bounds = new THREE.Box3();
      if (plannedPoints && plannedPoints.length > 0) {
        plannedPoints.forEach((p) => bounds.expandByPoint(new THREE.Vector3(p.x, p.y, p.z)));
      } else if (bbox) {
        bounds.copy(bbox);
      } else {
        bounds.setFromCenterAndSize(new THREE.Vector3(0, 0, 0), new THREE.Vector3(4, 4, 4));
      }
      const center = bounds.getCenter(new THREE.Vector3());
      const size = bounds.getSize(new THREE.Vector3());
      const radius = Math.max(size.x, size.y, size.z) * 0.75 || 1;
      const fov = (camera as THREE.PerspectiveCamera).fov * (Math.PI / 180);
      const distance = radius / Math.tan(fov / 2);
      const direction = new THREE.Vector3(1, 1, 1).normalize();
      camera.up.set(0, 1, 0);
      const newPos = center.clone().add(direction.multiplyScalar(distance));
      camera.position.copy(newPos);
      camera.lookAt(center);
      camera.updateProjectionMatrix();
      const controls = controlsRef.current;
      if (controls) {
        controls.target.copy(center);
        controls.update();
      }
    };
    registerFit(fit);
  }, [bbox, plannedPoints, camera, controlsRef, registerFit]);
  return null;
}

function CameraOrienter({
  controlsRef,
  registerOrient,
}: {
  controlsRef: React.RefObject<OrbitControlsImpl | null>;
  registerOrient: (fn: (plane: 'xy'|'xz'|'yz'|'iso') => void) => void;
}) {
  const { camera } = useThree();
  useEffect(() => {
    const orient = (plane: 'xy'|'xz'|'yz'|'iso') => {
      const center = new THREE.Vector3(0, 0, 0);
      const controls = controlsRef.current;
      const distance = 6;

      let normal = new THREE.Vector3(0, 0, 1);
      if (plane === 'xz') normal = new THREE.Vector3(0, 1, 0);
      if (plane === 'yz') normal = new THREE.Vector3(1, 0, 0);
      if (plane === 'iso') normal = new THREE.Vector3(1, 1, 1);

      camera.up.set(0, 1, 0);
      const newPos = center.clone().add(normal.normalize().multiplyScalar(distance));
      camera.position.copy(newPos);
      camera.lookAt(center);
      camera.updateProjectionMatrix();
      if (controls) {
        controls.target.copy(center);
        controls.update();
      }
    };
    registerOrient(orient);
  }, [camera, controlsRef, registerOrient]);
  return null;
}

function SceneResetter({
  controlsRef,
  registerReset,
}: {
  controlsRef: React.RefObject<OrbitControlsImpl | null>;
  registerReset: (fn: () => void) => void;
}) {
  const { camera } = useThree();
  useEffect(() => {
    const reset = () => {
      // Reset to default view (looking at 0,0,0 from a distance)
      const defaultPos = new THREE.Vector3(5, 5, 5); // Default position
      const defaultTarget = new THREE.Vector3(0, 0, 0);
      
      camera.position.copy(defaultPos);
      camera.up.set(0, 0, 1);
      camera.lookAt(defaultTarget);
      camera.updateProjectionMatrix();
      
      const controls = controlsRef.current;
      if (controls) {
        controls.target.copy(defaultTarget);
        controls.update();
      }
    };
    registerReset(reset);
  }, [camera, controlsRef, registerReset]);
  return null;
}

export const PCDCanvas = forwardRef<PCDCanvasHandle, PCDCanvasProps>(function PCDCanvas(
  { source, livePointClouds = [], pointSize = 0.01, performanceMode = false, showGrid = true, showAxes = true, showSceneCloud = true, colorMode = "none", roundPoints = true, voxelSize = 0.2, onLoadedAction, onLoadingChange, sceneRenderMode = 'points', plannedPathPoints, plannedPathVisible = true, plannedPointSize = 0.05, plannedPathLineWidth = 2, plannedPathEditable = false, onPlannedPointsChange, selectedPointIndex, onSelectPoint, predictedTrajectoryPoints, dronePosition, followDrone = false, waypoints, currentWaypointIndex },
  ref
) {
  const [geom, setGeom] = useState<THREE.BufferGeometry | null>(null);
  const [mesh, setMesh] = useState<THREE.Mesh | null>(null); // 存储网格对象
  const [bbox, setBbox] = useState<THREE.Box3 | null>(null);
  const controlsRef = useRef<OrbitControlsImpl | null>(null);
  const fitRef = useRef<(() => void) | null>(null);
  const resetRef = useRef<(() => void) | null>(null);
  const orientRef = useRef<((plane: 'xy'|'xz'|'yz'|'iso') => void) | null>(null);
  const [colorVersion, setColorVersion] = useState(0);
  const liveCloudGeometries = useMemo(() => {
    return livePointClouds.map((cloud, index) => {
      const pointCount = Math.floor(cloud.length / 3);
      const step = performanceMode ? 8 : 3;
      const samples = Math.max(1, Math.floor(pointCount / step));
      const positions = new Float32Array(samples * 3);
      let write = 0;
      for (let i = 0; i < pointCount && write < samples * 3; i += step) {
        const body = {
          x: cloud[i * 3],
          y: cloud[i * 3 + 1],
          z: cloud[i * 3 + 2],
        };
        const viewer = convertBodyPositionToViewer(body);
        positions[write++] = viewer.x;
        positions[write++] = viewer.y;
        positions[write++] = viewer.z;
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      return { geometry, opacity: Math.max(0.15, 0.75 - index * 0.08) };
    });
  }, [livePointClouds, performanceMode]);

  const effectiveRenderPreference = sceneRenderMode;
  const effectiveRenderMode: 'points' | 'mesh' | 'voxel' = useMemo(() => {
    if (effectiveRenderPreference === 'mesh') {
      if (mesh) return 'mesh';
      return geom ? 'points' : 'mesh';
    }
    if (effectiveRenderPreference === 'voxel') {
      if (geom) return 'voxel';
      return mesh ? 'mesh' : 'points';
    }
    return geom ? 'points' : (mesh ? 'mesh' : 'points');
  }, [effectiveRenderPreference, mesh, geom]);

  const appliedPointSize = performanceMode ? Math.max(pointSize * 0.7, 0.005) : pointSize;
  const fadeNear = performanceMode ? 1.5 : 2.0;
  const fadeFar = performanceMode ? 5.5 : 8.0;

  useEffect(() => {
    return () => {
      liveCloudGeometries.forEach(({ geometry }) => geometry.dispose());
    };
  }, [liveCloudGeometries]);
  // 生成一个圆形纹理用于点精灵（圆盘效果，边缘柔和）
  const circleTexture = useRef<THREE.Texture | null>(null);
  const pointMaterialRef = useRef<THREE.PointsMaterial | null>(null);
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
  useEffect(() => {
    const material = pointMaterialRef.current;
    if (!material) return;
    if (!material.userData._fadePatched) {
      material.onBeforeCompile = (shader) => {
        shader.uniforms.uFadeNear = { value: fadeNear };
        shader.uniforms.uFadeFar = { value: fadeFar };
        material.userData._fadeUniforms = shader.uniforms;
        shader.vertexShader = shader.vertexShader.replace(
          '#include <common>',
          '#include <common>\nvarying float vViewDistance;'
        ).replace(
          '#include <project_vertex>',
          '#include <project_vertex>\nvViewDistance = length(mvPosition.xyz);'
        );
        shader.fragmentShader = shader.fragmentShader.replace(
          '#include <common>',
          '#include <common>\nuniform float uFadeNear;\nuniform float uFadeFar;\nvarying float vViewDistance;'
        ).replace(
          '#include <color_fragment>',
          '#include <color_fragment>\nfloat fadeFactor = smoothstep(uFadeNear, uFadeFar, vViewDistance);\nif (fadeFactor <= 0.01) discard;\ndiffuseColor.a *= fadeFactor;'
        );
      };
      material.userData._fadePatched = true;
      material.needsUpdate = true;
    }
    const uniforms = material.userData._fadeUniforms;
    if (uniforms) {
      uniforms.uFadeNear.value = fadeNear;
      uniforms.uFadeFar.value = fadeFar;
    }
  }, [fadeNear, fadeFar, effectiveRenderMode, geom, colorVersion]);
  // 使用 ref 保存回调，避免因父组件函数身份变化导致重复加载
  const onLoadedRef = useRef<typeof onLoadedAction>(onLoadedAction);
  const onLoadingChangeRef = useRef<typeof onLoadingChange>(onLoadingChange);
  const onPlannedPointsChangeRef = useRef<typeof onPlannedPointsChange>(onPlannedPointsChange);
  const pointRefs = useRef<Array<THREE.Mesh | null>>([]);
  const plannedPointsRef = useRef<typeof plannedPathPoints>(plannedPathPoints);
  const transformControlsRef = useRef<TransformControlsImpl | null>(null);
  const [internalSelectedIdx, setInternalSelectedIdx] = useState<number | null>(null);
  const selectedIdxRef = useRef<number | null>(null);
  const isControlledSelection = typeof selectedPointIndex === "number" || selectedPointIndex === null;
  const selectedIdx = isControlledSelection ? (selectedPointIndex ?? null) : internalSelectedIdx;
  const setSelectedIdx = useCallback((value: number | null) => {
    if (typeof onSelectPoint === "function") {
      onSelectPoint(value);
    } else {
      setInternalSelectedIdx(value);
    }
  }, [onSelectPoint]);
  const [pathVersion, setPathVersion] = useState(0);
  useEffect(() => {
    setPathVersion((v) => {
      const next = v + 1;
      console.debug("[PCDCanvas] path changed", plannedPathPoints?.length, "version", next);
      return next;
    });
  }, [plannedPathPoints]);
  useEffect(() => { onLoadedRef.current = onLoadedAction; }, [onLoadedAction]);
  useEffect(() => { onLoadingChangeRef.current = onLoadingChange; }, [onLoadingChange]);
  useEffect(() => { onPlannedPointsChangeRef.current = onPlannedPointsChange; }, [onPlannedPointsChange]);
  useEffect(() => { plannedPointsRef.current = plannedPathPoints; }, [plannedPathPoints]);
  useEffect(() => { selectedIdxRef.current = selectedIdx; }, [selectedIdx]);
  useEffect(() => {
    const length = plannedPathPoints?.length ?? 0;
    pointRefs.current = pointRefs.current.slice(0, length);
  }, [plannedPathPoints?.length]);

  // Esc 取消选中
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        setSelectedIdx(null);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [setSelectedIdx]);

  useEffect(() => {
    if (!plannedPathEditable) {
      setSelectedIdx(null);
    }
  }, [plannedPathEditable, setSelectedIdx]);

  useEffect(() => {
    if (selectedIdx != null && (!plannedPathPoints || selectedIdx >= plannedPathPoints.length)) {
      setSelectedIdx(null);
    }
  }, [plannedPathPoints, selectedIdx, setSelectedIdx]);

  const transformControls = transformControlsRef.current;

  useEffect(() => {
    if (!transformControls) return;
    console.debug("[GIZMO] register events");
    const handleObjectChange = () => {
      const idx = selectedIdxRef.current;
      const pts = plannedPointsRef.current;
      const pointObject = idx != null ? pointRefs.current[idx] : null;
      if (!pointObject || !pts || idx == null || idx < 0 || idx >= pts.length) return;
      const pos = pointObject.position;
      const next = pts.map((pt, i) =>
        i === idx ? { ...pt, x: pos.x, y: pos.y, z: pos.z } : pt
      );
      plannedPointsRef.current = next;
      console.debug("[GIZMO] updating point", idx, { x: pos.x, y: pos.y, z: pos.z });
    };
    const handleDraggingChanged = (event: { value: boolean }) => {
      if (controlsRef.current) {
        controlsRef.current.enabled = !event.value;
      }
      if (!event.value) {
        const next = plannedPointsRef.current;
        if (next) {
          onPlannedPointsChangeRef.current?.(next);
        }
      }
    };
    const controlsEvents = transformControls as unknown as {
      addEventListener: (type: string, listener: (...args: unknown[]) => void) => void;
      removeEventListener: (type: string, listener: (...args: unknown[]) => void) => void;
    };
    controlsEvents.addEventListener("objectChange", handleObjectChange as (...args: unknown[]) => void);
    controlsEvents.addEventListener("dragging-changed", handleDraggingChanged as (...args: unknown[]) => void);
    return () => {
      console.debug("[GIZMO] unregister events");
      controlsEvents.removeEventListener("objectChange", handleObjectChange as (...args: unknown[]) => void);
      controlsEvents.removeEventListener("dragging-changed", handleDraggingChanged as (...args: unknown[]) => void);
    };
  }, [transformControls]);

  useEffect(() => {
    if (!transformControls) return;
    transformControls.detach();
    if (plannedPathEditable && selectedIdx !== null) {
      const target = pointRefs.current[selectedIdx];
      if (target) {
        console.debug("[GIZMO] attach to", selectedIdx, target.position);
        transformControls.attach(target);
      }
    }
  }, [plannedPathEditable, selectedIdx, plannedPathPoints, transformControls]);

  const registerFit = (fn: () => void) => {
    fitRef.current = fn;
  };
  const registerReset = (fn: () => void) => {
    resetRef.current = fn;
  };
  const registerOrient = (fn: (plane: 'xy'|'xz'|'yz'|'iso') => void) => {
    orientRef.current = fn;
  };
  useImperativeHandle(ref, () => ({
    fitToView: () => fitRef.current?.(),
    zoomToCenter: () => resetRef.current?.(),
    orientToPlane: (plane: 'xy'|'xz'|'yz'|'iso') => orientRef.current?.(plane),
  }), []);

  useEffect(() => {
    let cancelled = false;
    async function load() {
      if (!source) {
        setGeom(null);
        setMesh(null);
        setBbox(null);
        resetRef.current?.();
        return;
      }
      
      // 判断文件类型
      let fileExtension = '';
      if (source.type === "url") {
        fileExtension = source.url.split('.').pop()?.toLowerCase() || '';
      } else {
        fileExtension = source.file.name.split('.').pop()?.toLowerCase() || '';
      }
      
      try {
        onLoadingChangeRef.current?.(true);
        
        if (fileExtension === 'ply') {
          // 加载 PLY 文件
          const plyLoader = new PLYLoader();
          let geometry: THREE.BufferGeometry;
          
          if (source.type === "url") {
            geometry = await plyLoader.loadAsync(source.url);
          } else {
            const blobUrl = URL.createObjectURL(source.file);
            geometry = await plyLoader.loadAsync(blobUrl);
            URL.revokeObjectURL(blobUrl);
          }
          
          if (cancelled) return;
          
          convertGeometryPositionsToViewer(geometry);

          // PLY 可能包含法线和颜色
          if (!geometry.attributes.normal) {
            geometry.computeVertexNormals();
          }
          
          // 创建增强的网格材质
          const material = new THREE.MeshStandardMaterial({
            color: geometry.attributes.color ? 0xffffff : 0xdddddd,
            vertexColors: !!geometry.attributes.color,
            flatShading: false,
            side: THREE.DoubleSide,
            roughness: 0.6,
            metalness: 0.1,
            emissive: new THREE.Color(0x000000),
            emissiveIntensity: 0,
          });
          
          const meshObj = new THREE.Mesh(geometry, material);
          meshObj.castShadow = true;
          meshObj.receiveShadow = true;
          
          setMesh(meshObj);
          setGeom(geometry);
          
          const b = geometry.boundingBox ?? new THREE.Box3().setFromObject(meshObj);
          const clone = b.clone();
          setBbox(clone);
          
          const pos = geometry.getAttribute("position");
          const count = pos?.count ?? 0;
          onLoadedRef.current?.({ bbox: clone, count });
          setTimeout(() => fitRef.current?.(), 0);
          
          console.log(`Loaded PLY mesh: ${count} vertices`);
        } else {
          // 加载 PCD 文件（点云）
          const pcdLoader = new PCDLoader();
          let points: THREE.Points | null = null;
          
          if (source.type === "url") {
            points = (await pcdLoader.loadAsync(source.url)) as THREE.Points;
          } else {
            const blobUrl = URL.createObjectURL(source.file);
            points = (await pcdLoader.loadAsync(blobUrl)) as THREE.Points;
            URL.revokeObjectURL(blobUrl);
          }
          
          if (cancelled) return;
          
          const geometry = points.geometry as THREE.BufferGeometry;
          const processed = convertGeometryPositionsToViewer(geometry);
          const optimized = performanceMode ? downsampleGeometry(processed, voxelSize) : processed;
          setGeom(optimized);
          setMesh(null);
          
          const bboxSource = optimized.boundingBox ?? new THREE.Box3().setFromObject(points);
          const clone = bboxSource.clone();
          setBbox(clone);
          
          const pos = optimized.getAttribute("position");
          const count = pos?.count ?? 0;
          onLoadedRef.current?.({ bbox: clone, count });
          setTimeout(() => fitRef.current?.(), 0);
          
          console.log(`Loaded PCD point cloud: ${count} points`);
        }
      } catch (e) {
        console.error(`Failed to load ${fileExtension.toUpperCase()} file:`, e);
        resetRef.current?.();
      } finally {
        onLoadingChangeRef.current?.(false);
      }
    }
    load();
    return () => {
      cancelled = true;
    };
  }, [source, performanceMode, voxelSize]);

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
        const z = pos.getY(i); if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
      }
      const spanZ = maxZ - minZ || 1;
      for (let i = 0; i < count; i++) {
        const z = (pos.getY(i) - minZ) / spanZ;
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
          toneMappingExposure: 1.8,
          outputColorSpace: THREE.SRGBColorSpace
        }}
      >
        <color attach="background" args={["#1a1b1e"]} />
        
        {/* 增强光照系统 */}
        {/* 环境光 - 提供基础亮度 */}
        <ambientLight intensity={1.5} />
        
        {/* 主光源 - 模拟太阳光 */}
        <directionalLight 
          position={[10, 10, 10]} 
          intensity={2.0}
          castShadow
          shadow-mapSize-width={2048}
          shadow-mapSize-height={2048}
        />
        
        {/* 补光 - 从不同角度照亮场景 */}
        <directionalLight position={[-8, 5, 8]} intensity={1.2} />
        <directionalLight position={[5, -5, 5]} intensity={0.8} />
        
        {/* 顶部点光源 - 增加高光效果 */}
        <pointLight position={[0, 0, 15]} intensity={1.5} distance={50} decay={2} />
        
        {/* 半球光 - 模拟天空和地面反射 */}
        <hemisphereLight 
          args={["#ffffff", "#555555", 1.2]} 
          position={[0, 10, 0]}
        />
        
        {/* 聚光灯 - 突出中心区域（可选）*/}
        <spotLight
          position={[0, 20, 0]}
          angle={0.5}
          penumbra={0.5}
          intensity={0.8}
          castShadow
        />
        
        {showGrid && (
          <Grid
            position={[0, 0, 0]}
            args={[10, 10]}
            cellSize={1}
            cellThickness={0.4}
            sectionSize={5}
            sectionThickness={1}
            fadeDistance={60}
            fadeStrength={1}
            infiniteGrid
            rotation={[0, 0, 0]}
            cellColor="#2a2a2a"
            sectionColor="#3a3a3a"
          />
        )}
        {showAxes && (
          <group>
            <RosAxes length={3} />
            <AxisLabels length={3} labelOffset={0.2} mode="ros" />
          </group>
        )}
        
        {/* 场景渲染：根据模式选择渲染方式 */}
        {showSceneCloud && effectiveRenderMode === 'points' && geom && (
          <points key={colorVersion} geometry={geom} frustumCulled={false}>
            <pointsMaterial
              ref={pointMaterialRef}
              size={appliedPointSize}
              sizeAttenuation
              vertexColors={!!geom.getAttribute("color")}
              map={roundPoints ? circleTexture.current ?? undefined : undefined}
              transparent
              alphaTest={roundPoints ? 0.5 : 0}
              depthWrite={!roundPoints}
            />
          </points>
        )}
        
        {/* 体素化点云渲染 */}
        {showSceneCloud && effectiveRenderMode === 'voxel' && geom && (
          <VoxelizedPointCloud 
            geometry={geom} 
            voxelSize={voxelSize}
          />
        )}

        {/* 预测轨迹渲染 */}
        {Array.isArray(predictedTrajectoryPoints) && predictedTrajectoryPoints.length > 1 && (
          <Line
            key={`predicted-${predictedTrajectoryPoints.length}`}
            points={predictedTrajectoryPoints.map((p) => [p.x, p.y, p.z]) as [number, number, number][]}
            color="#f97316"
            lineWidth={Math.max(1, plannedPathLineWidth * 0.8)}
            dashed
            transparent
            opacity={0.7}
          />
        )}

        {/* PLY 网格渲染 */}
        {showSceneCloud && effectiveRenderMode === 'mesh' && mesh && (
          <primitive object={mesh} key={colorVersion} />
        )}

        {/* 实时点云 */}
        {liveCloudGeometries.map(({ geometry, opacity }, idx) => (
          <points key={`live-cloud-${idx}`} geometry={geometry} frustumCulled={false}>
            <pointsMaterial
              size={0.035}
              sizeAttenuation
              color="#4ade80"
              transparent
              opacity={opacity}
            />
          </points>
        ))}

        {/* 编辑航线渲染 + Gizmo 编辑 */}
        {plannedPathVisible && Array.isArray(plannedPathPoints) && plannedPathPoints.length > 0 && (
          <group key={`path-${pathVersion}`}>
            {/* 待完成路径段（灰色） */}
            {(() => {
              console.debug("[LINE] render pending", pathVersion);
              if (plannedPathPoints.length < 2) return null;
              const hasStatus = Array.isArray(waypoints) && waypoints.length > 0;
              if (!hasStatus) {
                return (
                  <Line
                    key={`line-full-${pathVersion}`}
                    points={plannedPathPoints.map(p => [p.x, p.y, p.z]) as [number, number, number][]}
                    color="#94a3b8"
                    lineWidth={plannedPathLineWidth}
                    dashed={false}
                    depthTest
                    opacity={0.8}
                    transparent
                  />
                );
              }
              const lastCompleted = waypoints!.reduce((m, w) => (w.status === 'completed' ? Math.max(m, w.index ?? 0) : m), -1);
              if (lastCompleted >= plannedPathPoints.length - 1) return null;
              const pendingStart = Math.max(0, lastCompleted);
              const pendingPoints = plannedPathPoints.slice(pendingStart).map(p => [p.x, p.y, p.z]) as [number, number, number][];
              if (pendingPoints.length < 2) return null;
              return (
                <Line
                  key={`line-pending-${pathVersion}-${pendingPoints.length}`}
                  points={pendingPoints}
                  color="#94a3b8"
                  lineWidth={plannedPathLineWidth}
                  dashed={false}
                  depthTest
                  opacity={0.8}
                  transparent
                />
              );
            })()}
            {/* 已完成路径段（绿色） */}
            {(() => {
              console.debug("[LINE] render completed", pathVersion);
              if (!Array.isArray(waypoints) || waypoints.length === 0) return null;
              const lastCompleted = waypoints.reduce((m, w) => (w.status === 'completed' ? Math.max(m, w.index ?? 0) : m), -1);
              const endIdx = Math.min(lastCompleted, plannedPathPoints.length - 1);
              if (endIdx >= 1) {
                const pts = plannedPathPoints.slice(0, endIdx + 1).map(p => [p.x, p.y, p.z]) as [number, number, number][];
                return (
                  <Line
                    key={`line-completed-${pathVersion}-${endIdx}`}
                    points={pts}
                    color="#22c55e"
                    lineWidth={Math.max(plannedPathLineWidth, 1) + 1}
                    dashed={false}
                    depthTest
                    opacity={1}
                    transparent={false}
                  />
                );
              }
              return null;
            })()}
            {/* 航点球体 + 选中高亮 */}
            {plannedPathPoints.map((p, i) => {
              const waypoint = waypoints?.find(w => w.index === i);
              const getWaypointColor = () => {
                if (selectedIdx === i) return "#f59e0b";
                if (waypoint) {
                  switch (waypoint.status) {
                    case 'completed': return "#22c55e";
                    case 'active': return "#f59e0b";
                    case 'pending': return "#60a5fa";
                    case 'skipped': return "#6b7280";
                    default: return "#60a5fa";
                  }
                }
                if (i === 0) return "#22c55e";
                if (i === plannedPathPoints.length - 1) return "#ef4444";
                return "#60a5fa";
              };
              const waypointColor = getWaypointColor();
              const isActive = waypoint?.status === 'active' || currentWaypointIndex === i;
              return (
                <mesh
                  key={i}
                  ref={(node) => { pointRefs.current[i] = node; }}
                  position={[p.x, p.y, p.z] as [number, number, number]}
                  onClick={(e) => {
                    if (!plannedPathEditable) return;
                    e.stopPropagation();
                    if (e.altKey) {
                      const next = plannedPathPoints.slice();
                      next.splice(i, 1);
                      console.debug("[GIZMO] delete point", i);
                      onPlannedPointsChangeRef.current?.(next);
                      return;
                    }
                    const nextIdx = selectedIdx === i ? null : i;
                    setSelectedIdx(nextIdx);
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
            {plannedPathPoints.map((p, i) => {
              if (typeof p.w !== "number") return null;
              const dirBody = { x: Math.cos(p.w), y: Math.sin(p.w), z: 0 };
              const converted = convertBodyPositionToViewer(dirBody);
              const dir = new THREE.Vector3(converted.x, converted.y, converted.z);
              if (dir.lengthSq() < 1e-6) return null;
              dir.normalize();
              const length = Math.max(0.8, plannedPointSize * 12);
              const coneLength = length * 0.25;
              const shaftLength = length - coneLength;
              const shaftEnd = dir.clone().multiplyScalar(shaftLength);
              const tipEnd = dir.clone().multiplyScalar(length);
              const shaftPoints: [number, number, number][] = [[0, 0, 0], [shaftEnd.x, shaftEnd.y, shaftEnd.z]];
              const axis = new THREE.Vector3(0, 1, 0);
              const arrowQuat = new THREE.Quaternion().setFromUnitVectors(axis, dir);
              return (
                <group key={`yaw-${i}`} position={[p.x, p.y, p.z]}>
                  <Line
                    points={shaftPoints}
                    color="#f97316"
                    lineWidth={Math.max(1, plannedPathLineWidth * 0.6)}
                    transparent
                    opacity={0.9}
                  />
                  <mesh position={[tipEnd.x, tipEnd.y, tipEnd.z]} quaternion={arrowQuat}>
                    <coneGeometry args={[Math.max(0.05, plannedPointSize * 1.5), coneLength, 8]} />
                    <meshStandardMaterial color="#f97316" emissive="#f97316" emissiveIntensity={0.4} />
                  </mesh>
                </group>
              );
            })}
          </group>
        )}

        {plannedPathEditable && (
          <TransformControls
            ref={transformControlsRef}
            enabled={plannedPathEditable && selectedIdx !== null && !!pointRefs.current[selectedIdx ?? 0]}
            mode="translate"
            showX
            showY
            showZ
            size={0.8}
          />
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
        <SceneFitter bbox={bbox} plannedPoints={plannedPathPoints} controlsRef={controlsRef} registerFit={registerFit} />
        <SceneResetter controlsRef={controlsRef} registerReset={registerReset} />
        <CameraOrienter controlsRef={controlsRef} registerOrient={registerOrient} />
        <CameraFollower dronePosition={dronePosition ?? null} followDrone={followDrone} controlsRef={controlsRef} />
      </Canvas>
      {plannedPathEditable && selectedIdx == null && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center text-xs text-muted-foreground">
          <div className="bg-background/90 px-3 py-1 rounded border border-border/50 shadow">
            选中航点后可拖动 Gizmo 调整位置
          </div>
        </div>
      )}
    </div>
  );
});

export default PCDCanvas;
