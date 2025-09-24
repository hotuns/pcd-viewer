"use client";

import {
  forwardRef,
  useEffect,
  useImperativeHandle,
  useMemo,
  useRef,
  useState,
} from "react";
import { Canvas, useThree, useFrame } from "@react-three/fiber";
import { OrbitControls, Grid, Html, Stats } from "@react-three/drei";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader.js";

export type Source =
  | { type: "file"; file: File }
  | { type: "url"; url: string };

export type PCDCanvasHandle = {
  fitToView: () => void;
};

export type PCDCanvasProps = {
  source?: Source | null;
  pointSize?: number;
  showGrid?: boolean;
  showAxes?: boolean;
  sizeAttenuation?: boolean;
  autoFit?: boolean;
  colorMode?: "original" | "height" | "intensity" | "classification";
  onLoadedAction?: (info: { bbox: THREE.Box3; count: number }) => void;
  onLoadingChange?: (loading: boolean) => void;
  showPerf?: boolean;
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

function PerfHud() {
  const { gl } = useThree();
  const [text, setText] = useState<string>("");
  const accRef = useRef(0);
  useFrame((_, delta) => {
    accRef.current += delta;
    if (accRef.current < 0.25) return; // update ~4fps
    accRef.current = 0;
    const info = gl.info;
    const { memory, render } = info;
    setText(
      `draws ${render.calls}  tris ${render.triangles}  points ${render.points}  lines ${render.lines}  geoms ${memory.geometries}  tex ${memory.textures}`
    );
  });
  return (
    <Html position={[0, 0, 0]} transform={false} fullscreen>
      <div className="pointer-events-none fixed bottom-2 right-2 text-[10px] leading-4 bg-black/60 text-white/90 rounded px-2 py-1 border border-white/10">
        {text || "perf: -"}
      </div>
    </Html>
  );
}

function Points({
  geometry,
  size,
  sizeAttenuation,
  colorVersion,
}: {
  geometry: THREE.BufferGeometry;
  size: number;
  sizeAttenuation: boolean;
  colorVersion: number;
}) {
  const hasColors = !!geometry.getAttribute("color");
  const material = useMemo(
    () =>
      new THREE.PointsMaterial({
        size,
        vertexColors: hasColors,
        sizeAttenuation,
      }),
    [size, sizeAttenuation, geometry, hasColors, colorVersion]
  );
  useEffect(() => () => material.dispose(), [material]);
  return (
    <points geometry={geometry} material={material} frustumCulled={false} />
  );
}

export const PCDCanvas = forwardRef<PCDCanvasHandle, PCDCanvasProps>(
  function PCDCanvas(
    {
      source,
      pointSize = 0.01,
      showGrid = true,
      showAxes = true,
      sizeAttenuation = true,
      autoFit = true,
      colorMode = "original",
      onLoadedAction,
      onLoadingChange,
      showPerf = false,
    },
    ref
  ) {
    const [geom, setGeom] = useState<THREE.BufferGeometry | null>(null);
    const [bbox, setBbox] = useState<THREE.Box3 | null>(null);
    const controlsRef = useRef<OrbitControlsImpl | null>(null);
    const fitRef = useRef<(() => void) | null>(null);
    const originalColorRef = useRef<THREE.BufferAttribute | null>(null);
    const [colorVersion, setColorVersion] = useState(0);
    const pointsRef = useRef<THREE.Points | null>(null);
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
        if (!source) return;
        const loader = new PCDLoader();
        try {
          onLoadingChange?.(true);
          let points: THREE.Points | null = null;
          if (source.type === "url") {
            const url = source.url;
            points = (await loader.loadAsync(url)) as THREE.Points;
          } else {
            const file = source.file;
            const blobUrl = URL.createObjectURL(file);
            points = (await loader.loadAsync(blobUrl)) as THREE.Points;
            URL.revokeObjectURL(blobUrl);
          }
          if (cancelled) return;
          const geometry = points.geometry as THREE.BufferGeometry;
          geometry.computeBoundingBox();
          setGeom(geometry);
          // save original colors if any
          const orig = geometry.getAttribute("color") as
            | THREE.BufferAttribute
            | undefined;
          originalColorRef.current = orig ? orig.clone() : null;
          const b = geometry.boundingBox ?? new THREE.Box3().setFromObject(points);
          const clone = b.clone();
          setBbox(clone);
          const pos = geometry.getAttribute("position");
          const count = (pos?.count ?? 0);
          onLoadedAction?.({ bbox: clone, count });
          // auto fit after load
          if (autoFit) setTimeout(() => fitRef.current?.(), 0);
          // apply initial color mode
          setTimeout(() => setColorVersion((v) => v + 1), 0);
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
      // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [source]);

    // utilities for color mapping
    const turbo = (t: number) => {
      // simple turbo-like colormap approximation (t in [0,1])
      const r = Math.min(1, Math.max(0, 1.0 + 0.0 * t - 2.0 * Math.abs(t - 0.5)));
      const g = Math.min(1, Math.max(0, 1.2 - 4.0 * Math.pow(t - 0.5, 2)));
      const b = Math.min(1, Math.max(0, 1.0 - 2.0 * t + 2.0 * Math.abs(t - 0.5)));
      return [r, g, b] as [number, number, number];
    };

    const classColor = (c: number) => {
      const palette: [number, number, number][] = [
        [0.8, 0.8, 0.8], // 0
        [0.8, 0.4, 0.4], // 1
        [0.4, 0.8, 0.4], // 2
        [0.4, 0.4, 0.8], // 3
        [0.8, 0.8, 0.4], // 4
        [0.8, 0.4, 0.8], // 5
        [0.4, 0.8, 0.8], // 6
        [0.9, 0.6, 0.3], // 7
        [0.6, 0.3, 0.9], // 8
        [0.3, 0.9, 0.6], // 9
      ];
      return palette[c % palette.length];
    };

    // apply color mode when geometry/bbox/mode changes
    useEffect(() => {
      if (!geom) return;
      const pos = geom.getAttribute("position") as THREE.BufferAttribute | undefined;
      if (!pos) return;

      const applyOriginal = () => {
        if (originalColorRef.current) {
          geom.setAttribute("color", originalColorRef.current.clone());
        } else {
          if (geom.getAttribute("color")) geom.deleteAttribute("color");
        }
        setColorVersion((v) => v + 1);
      };

      if (colorMode === "original") {
        applyOriginal();
        return;
      }

      let buffer: Float32Array | null = null;
      const count = pos.count;

      if (colorMode === "height") {
        const bb = bbox ?? (() => { const b = new THREE.Box3(); b.setFromBufferAttribute(pos); return b; })();
        const min = bb.min.z, max = bb.max.z;
        const range = Math.max(1e-6, max - min);
        buffer = new Float32Array(count * 3);
        for (let i = 0; i < count; i++) {
          const z = pos.getZ(i);
          const t = (z - min) / range;
          const [r, g, b] = turbo(t);
          const j = i * 3;
          buffer[j] = r; buffer[j + 1] = g; buffer[j + 2] = b;
        }
      } else if (colorMode === "intensity") {
        const intenAttr = (geom.getAttribute("intensity") || geom.getAttribute("Intensity")) as
          | THREE.BufferAttribute
          | undefined;
        if (!intenAttr) {
          // fallback to height
          const bb = bbox ?? (() => { const b = new THREE.Box3(); b.setFromBufferAttribute(pos); return b; })();
          const min = bb.min.z, max = bb.max.z;
          const range = Math.max(1e-6, max - min);
          buffer = new Float32Array(count * 3);
          for (let i = 0; i < count; i++) {
            const z = pos.getZ(i);
            const t = (z - min) / range;
            const [r, g, b] = turbo(t);
            const j = i * 3;
            buffer[j] = r; buffer[j + 1] = g; buffer[j + 2] = b;
          }
        } else {
          let min = Infinity, max = -Infinity;
          for (let i = 0; i < count; i++) {
            const v = intenAttr.getX(i);
            if (v < min) min = v; if (v > max) max = v;
          }
          const range = Math.max(1e-6, max - min);
          buffer = new Float32Array(count * 3);
          for (let i = 0; i < count; i++) {
            const t = (intenAttr.getX(i) - min) / range;
            const [r, g, b] = turbo(t);
            const j = i * 3;
            buffer[j] = r; buffer[j + 1] = g; buffer[j + 2] = b;
          }
        }
      } else if (colorMode === "classification") {
        const classAttr = (geom.getAttribute("classification") || geom.getAttribute("label") || geom.getAttribute("class")) as
          | THREE.BufferAttribute
          | undefined;
        buffer = new Float32Array(count * 3);
        for (let i = 0; i < count; i++) {
          const cVal = classAttr ? classAttr.getX(i) : 0;
          const [r, g, b] = classColor(Math.round(cVal));
          const j = i * 3;
          buffer[j] = r; buffer[j + 1] = g; buffer[j + 2] = b;
        }
      }

      if (buffer) {
        geom.setAttribute("color", new THREE.BufferAttribute(buffer, 3));
        (geom.getAttribute("color") as THREE.BufferAttribute).needsUpdate = true;
        setColorVersion((v) => v + 1);
      }
    }, [geom, bbox, colorMode]);

    // no-op: annotations removed for performance

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
        <points ref={pointsRef} geometry={geom} frustumCulled={false}>
          <pointsMaterial size={pointSize} sizeAttenuation={sizeAttenuation} vertexColors={!!geom.getAttribute('color')} />
        </points>
      )}
      <OrbitControls ref={controlsRef} makeDefault />
      {showPerf && (
        <>
          <Stats className="select-none" showPanel={0} />
          <PerfHud />
        </>
      )}
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
