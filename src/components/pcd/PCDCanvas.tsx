"use client";

import {
  forwardRef,
  useEffect,
  useImperativeHandle,
  useRef,
  useState,
} from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader.js";
import type { Source, DronePosition } from "@/types/mission";

export type PCDCanvasHandle = {
  fitToView: () => void;
};

export type PCDCanvasProps = {
  source?: Source | null;
  pointSize?: number;
  showGrid?: boolean;
  showAxes?: boolean;
  dronePosition?: DronePosition | null;
  showDrone?: boolean;
  showVelocityVector?: boolean;
  // 实时点云数据支持
  livePointCloud?: {
    points: Float32Array;
    colors?: Float32Array;
    count: number;
  } | null;
  showLivePointCloud?: boolean;
  // 栅格地图数据支持
  gridMapData?: {
    points: Float32Array;
    colors?: Float32Array;
    count: number;
    width: number;
    height: number;
  } | null;
  showGridMap?: boolean;
  onLoadedAction?: (info: { bbox: THREE.Box3; count: number }) => void;
  onLoadingChange?: (loading: boolean) => void;
};

// 无人机3D模型组件
function DroneModel({
  position,
  orientation,
  velocity,
  showVelocityVector,
}: {
  position: [number, number, number];
  orientation?: { x: number; y: number; z: number; w: number };
  velocity?: { x: number; y: number; z: number };
  showVelocityVector?: boolean;
}) {
  const meshRef = useRef<THREE.Group>(null);

  useEffect(() => {
    if (meshRef.current && orientation) {
      // 将四元数转换为Three.js的四元数并应用到模型
      const quaternion = new THREE.Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      );
      meshRef.current.setRotationFromQuaternion(quaternion);
    }
  }, [orientation]);

  // 计算速度向量的方向和长度
  const velocityVector = velocity
    ? {
        direction: new THREE.Vector3(
          velocity.x,
          velocity.y,
          velocity.z
        ).normalize(),
        magnitude: Math.sqrt(
          velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2
        ),
      }
    : null;

  return (
    <group ref={meshRef} position={position}>
      {/* 无人机主体 - 橙色方形 */}
      <mesh>
        <boxGeometry args={[0.3, 0.1, 0.3]} />
        <meshStandardMaterial color="#ff6b35" />
      </mesh>

      {/* 4个黑色圆片 */}
      {[
        [0.2, 0, 0.2], // 右前
        [-0.2, 0, 0.2], // 左前
        [-0.2, 0, -0.2], // 左后
        [0.2, 0, -0.2], // 右后
      ].map((pos, i) => (
        <group key={i}>
          {/* 螺旋桨 */}
          <mesh
            position={pos as [number, number, number]}
            rotation={[Math.PI / 2, 0, 0]}
          >
            <cylinderGeometry args={[0.08, 0.08, 0.02]} />
            <meshStandardMaterial color="#666666" transparent opacity={0.7} />
          </mesh>
        </group>
      ))}

      {/* 速度向量箭头 */}
      {showVelocityVector &&
        velocityVector &&
        velocityVector.magnitude > 0.1 && (
          <group>
            <arrowHelper
              args={[
                velocityVector.direction,
                new THREE.Vector3(0, 0, 0),
                velocityVector.magnitude * 0.5, // 缩放显示长度
                0x00ff00, // 绿色
                velocityVector.magnitude * 0.1, // 箭头头部长度
                velocityVector.magnitude * 0.05, // 箭头头部宽度
              ]}
            />
          </group>
        )}
    </group>
  );
}

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
      const newPos = new THREE.Vector3(
        center.x + dist,
        center.y + dist,
        center.z + dist
      );
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

export const PCDCanvas = forwardRef<PCDCanvasHandle, PCDCanvasProps>(
  function PCDCanvas(
    {
      source,
      pointSize = 0.01,
      showGrid = true,
      showAxes = true,
      dronePosition,
      showDrone = true,
      showVelocityVector = true,
      livePointCloud,
      showLivePointCloud = true,
      gridMapData,
      showGridMap = true,
      onLoadedAction,
      onLoadingChange,
    },
    ref
  ) {
    const [geom, setGeom] = useState<THREE.BufferGeometry | null>(null);
    const [bbox, setBbox] = useState<THREE.Box3 | null>(null);
    const [liveGeom, setLiveGeom] = useState<THREE.BufferGeometry | null>(null);
    const [gridMapGeom, setGridMapGeom] = useState<THREE.BufferGeometry | null>(
      null
    );
    const controlsRef = useRef<OrbitControlsImpl | null>(null);
    const fitRef = useRef<(() => void) | null>(null);

    const registerFit = (fn: () => void) => {
      fitRef.current = fn;
    };
    useImperativeHandle(
      ref,
      () => ({ fitToView: () => fitRef.current?.() }),
      []
    );

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
          onLoadingChange?.(true);
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
          const b =
            geometry.boundingBox ?? new THREE.Box3().setFromObject(points);
          const clone = b.clone();
          setBbox(clone);
          const pos = geometry.getAttribute("position");
          const count = pos?.count ?? 0;
          onLoadedAction?.({ bbox: clone, count });
          // auto fit
          setTimeout(() => fitRef.current?.(), 0);
        } catch (e) {
          console.error("Failed to load PCD:", e);
        } finally {
          onLoadingChange?.(false);
        }
      }
      load();
      return () => {
        cancelled = true;
      };
    }, [source, onLoadedAction, onLoadingChange]);

    // 处理实时点云数据
    useEffect(() => {
      if (!livePointCloud || !showLivePointCloud) {
        setLiveGeom(null);
        return;
      }

      console.log("Processing live point cloud:", {
        count: livePointCloud.count,
        pointsLength: livePointCloud.points.length,
        colorsLength: livePointCloud.colors?.length,
      });

      // 创建新的几何体
      const geometry = new THREE.BufferGeometry();

      // 设置位置属性
      geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(livePointCloud.points, 3)
      );

      // 设置颜色属性（如果有的话）
      if (livePointCloud.colors) {
        geometry.setAttribute(
          "color",
          new THREE.BufferAttribute(livePointCloud.colors, 3)
        );
      }

      // 计算边界框
      geometry.computeBoundingBox();

      setLiveGeom(geometry);

      // 清理函数
      return () => {
        geometry.dispose();
      };
    }, [livePointCloud, showLivePointCloud]);

    // 处理栅格地图数据
    useEffect(() => {
      if (!gridMapData || !showGridMap) {
        setGridMapGeom(null);
        return;
      }

      // 创建新的几何体
      const geometry = new THREE.BufferGeometry();

      // 设置位置属性
      geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(gridMapData.points, 3)
      );

      // 设置颜色属性（如果有的话）
      if (gridMapData.colors) {
        geometry.setAttribute(
          "color",
          new THREE.BufferAttribute(gridMapData.colors, 3)
        );
      }

      // 计算边界框
      geometry.computeBoundingBox();

      setGridMapGeom(geometry);

      // 清理函数
      return () => {
        geometry.dispose();
      };
    }, [gridMapData, showGridMap]);

    return (
      <Canvas camera={{ position: [2, 2, 2], fov: 60 }} dpr={[1, 2]}>
        <color attach="background" args={["#111214"]} />
        <ambientLight intensity={0.5} />
        <directionalLight position={[5, 5, 5]} intensity={0.8} />
        {showGrid && (
          <Grid
            args={[10, 10]}
            cellColor="#2a2a2a"
            sectionColor="#3a3a3a"
            infiniteGrid
          />
        )}
        {showAxes && <axesHelper args={[1]} />}
        {geom && (
          <points geometry={geom} frustumCulled={false}>
            <pointsMaterial
              size={pointSize}
              sizeAttenuation
              vertexColors={!!geom.getAttribute("color")}
            />
          </points>
        )}
        {/* 渲染实时点云 */}
        {liveGeom && showLivePointCloud && (
          <points geometry={liveGeom} frustumCulled={false}>
            <pointsMaterial
              size={pointSize * 0.5}
              sizeAttenuation
              vertexColors={!!liveGeom.getAttribute("color")}
              color="#00ff00"
            />
          </points>
        )}

        {/* 渲染栅格地图 */}
        {gridMapGeom && showGridMap && (
          <points geometry={gridMapGeom} frustumCulled={false}>
            <pointsMaterial
              size={pointSize * 0.8}
              sizeAttenuation
              vertexColors={!!gridMapGeom.getAttribute("color")}
              color="#ff3333"
            />
          </points>
        )}

        {/* 渲染无人机模型 */}
        {showDrone && dronePosition && (
          <DroneModel
            position={[dronePosition.x, dronePosition.y, dronePosition.z]}
            orientation={dronePosition.orientation}
            velocity={dronePosition.velocity}
            showVelocityVector={showVelocityVector}
          />
        )}
        <OrbitControls ref={controlsRef} makeDefault />
        <SceneFitter
          bbox={bbox}
          controlsRef={controlsRef}
          registerFit={registerFit}
        />
      </Canvas>
    );
  }
);

export default PCDCanvas;
