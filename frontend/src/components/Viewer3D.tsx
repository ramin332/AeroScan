import { useRef, useMemo, useState, useCallback, useEffect } from 'react';
import { Canvas, useThree, useFrame, type ThreeEvent } from '@react-three/fiber';
import { OrbitControls, Line } from '@react-three/drei';
import * as THREE from 'three';
import type { RawMeshData, ThreeJSData, WaypointData } from '../api/types';
import { DroneMarker, getVisitedIndex, DRONE_LAYER } from './DroneAnimation';

// Layers: 0 = base scene + ghost mesh, 1 = drone (hidden from PIP), 3 = solid mesh (PIP only)
const MESH_SOLID_LAYER = 3;

// Convert ENU (x=East, y=North, z=Up) to Three.js (x=right, y=up, z=toward camera)
function enu(x: number, y: number, z: number): [number, number, number] {
  return [x, z, -y];
}

function FacadeMesh({ vertices, color, hidden, highlighted }: { vertices: number[][]; color: string; hidden?: boolean; highlighted?: boolean }) {
  const geometry = useMemo(() => {
    const geo = new THREE.BufferGeometry();
    const positions: number[] = [];
    for (let i = 1; i < vertices.length - 1; i++) {
      positions.push(vertices[0][0], vertices[0][2], -vertices[0][1]);
      positions.push(vertices[i][0], vertices[i][2], -vertices[i][1]);
      positions.push(vertices[i + 1][0], vertices[i + 1][2], -vertices[i + 1][1]);
    }
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geo.computeVertexNormals();
    return geo;
  }, [vertices]);

  const edgePoints = useMemo(() => {
    const pts: [number, number, number][] = [];
    for (let i = 0; i < vertices.length; i++) {
      pts.push(enu(vertices[i][0], vertices[i][1], vertices[i][2]));
    }
    pts.push(pts[0]); // close loop
    return pts;
  }, [vertices]);

  if (hidden) return null;

  return (
    <group>
      <mesh geometry={geometry} renderOrder={highlighted ? 10 : 0}>
        <meshStandardMaterial
          color={color} transparent
          opacity={highlighted ? 0.85 : 0.35}
          side={THREE.DoubleSide} roughness={0.8}
          depthWrite={highlighted ? true : false}
          depthTest={true}
          emissive={highlighted ? color : '#000000'}
          emissiveIntensity={highlighted ? 0.3 : 0}
        />
      </mesh>
      <Line points={edgePoints} color={color} lineWidth={highlighted ? 2.5 : 1.5} />
    </group>
  );
}

// Adaptive sphere detail: fewer polys when there are many waypoints
function sphereArgs(count: number): [number, number, number] {
  if (count > 2000) return [0.06, 3, 2];
  if (count > 500)  return [0.08, 4, 3];
  return [0.1, 6, 4];
}

const BATCH_SIZE = 500; // matrices per frame

function WaypointSpheres({ waypoints, color, bright }: { waypoints: WaypointData[]; color: string; bright?: boolean }) {
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const [radius, wSeg, hSeg] = sphereArgs(waypoints.length);

  useEffect(() => {
    if (!meshRef.current || waypoints.length === 0) return;
    const mesh = meshRef.current;
    const dummy = new THREE.Object3D();
    let offset = 0;
    let raf: number;

    function batch() {
      const end = Math.min(offset + BATCH_SIZE, waypoints.length);
      for (let i = offset; i < end; i++) {
        const wp = waypoints[i];
        dummy.position.set(...enu(wp.x, wp.y, wp.z));
        dummy.updateMatrix();
        mesh.setMatrixAt(i, dummy.matrix);
      }
      mesh.instanceMatrix.needsUpdate = true;
      offset = end;
      if (offset < waypoints.length) {
        raf = requestAnimationFrame(batch);
      }
    }
    batch();
    return () => cancelAnimationFrame(raf);
  }, [waypoints]);

  if (waypoints.length === 0) return null;

  return (
    <instancedMesh ref={meshRef} args={[undefined, undefined, waypoints.length]} frustumCulled={false}>
      <sphereGeometry args={[bright ? radius * 1.3 : radius, wSeg, hSeg]} />
      <meshStandardMaterial color={color} transparent opacity={bright ? 0.85 : 0.45} roughness={0.4} metalness={bright ? 0.2 : 0.1} emissive={bright ? color : '#000000'} emissiveIntensity={bright ? 0.3 : 0} />
    </instancedMesh>
  );
}

function CameraArrows({ waypoints, color, bright }: { waypoints: WaypointData[]; color: string; bright?: boolean }) {
  // For large sets, only draw every Nth arrow
  const stride = waypoints.length > 2000 ? 4 : waypoints.length > 500 ? 2 : 1;

  const geometry = useMemo(() => {
    const positions: number[] = [];
    for (let j = 0; j < waypoints.length; j += stride) {
      const wp = waypoints[j];
      const [ox, oy, oz] = enu(wp.x, wp.y, wp.z);
      const hr = (wp.heading * Math.PI) / 180;
      const pr = (wp.gimbal_pitch * Math.PI) / 180;
      const dx = Math.sin(hr) * Math.cos(pr);
      const dy = Math.cos(hr) * Math.cos(pr);
      const dz = Math.sin(pr);
      const [ex, ey, ez] = enu(wp.x + dx * 2, wp.y + dy * 2, wp.z + dz * 2);
      positions.push(ox, oy, oz, ex, ey, ez);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    return geo;
  }, [waypoints, stride]);

  return (
    <lineSegments geometry={geometry}>
      <lineBasicMaterial color={color} transparent opacity={bright ? 0.6 : 0.25} />
    </lineSegments>
  );
}

function FlightPath({ waypoints }: { waypoints: WaypointData[] }) {
  const geometry = useMemo(() => {
    if (waypoints.length < 2) return null;
    const positions: number[] = [];
    for (let i = 0; i < waypoints.length - 1; i++) {
      const [x1, y1, z1] = enu(waypoints[i].x, waypoints[i].y, waypoints[i].z);
      const [x2, y2, z2] = enu(waypoints[i + 1].x, waypoints[i + 1].y, waypoints[i + 1].z);
      positions.push(x1, y1, z1, x2, y2, z2);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    return geo;
  }, [waypoints]);

  if (!geometry) return null;
  return (
    <lineSegments geometry={geometry}>
      <lineBasicMaterial color="white" transparent opacity={0.15} />
    </lineSegments>
  );
}

function RawMeshView({ mesh, dimmed }: { mesh: RawMeshData; dimmed?: boolean }) {
  const solidRef = useRef<THREE.Mesh>(null);

  const geometry = useMemo(() => {
    const geo = new THREE.BufferGeometry();
    // Convert ENU positions to Three.js coordinate system
    const pos = new Float32Array(mesh.positions.length);
    for (let i = 0; i < mesh.positions.length; i += 3) {
      pos[i] = mesh.positions[i];       // x → x
      pos[i + 1] = mesh.positions[i + 2]; // z → y (up)
      pos[i + 2] = -mesh.positions[i + 1]; // -y → z (toward camera)
    }
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setIndex(new THREE.BufferAttribute(new Uint32Array(mesh.indices), 1));
    geo.computeVertexNormals();
    return geo;
  }, [mesh]);

  // Solid mesh only visible to PIP camera (layer 3); ghost mesh stays on default layer 0
  useEffect(() => {
    if (solidRef.current) {
      solidRef.current.layers.set(MESH_SOLID_LAYER);
    }
  });

  return (
    <group>
      {/* Ghost mesh: semi-transparent for orbital view */}
      <group>
        <mesh geometry={geometry} renderOrder={-1}>
          <meshStandardMaterial
            color="#8899bb"
            transparent
            opacity={dimmed ? 0.08 : 0.25}
            side={THREE.DoubleSide}
            roughness={0.7}
            metalness={0.1}
            depthWrite={false}
          />
        </mesh>
        <mesh geometry={geometry} renderOrder={-1}>
          <meshStandardMaterial
            color="#aabbdd"
            wireframe
            transparent
            opacity={dimmed ? 0.04 : 0.15}
            depthWrite={false}
          />
        </mesh>
      </group>

      {/* Solid mesh: opaque for PIP camera preview (realistic building) */}
      <mesh ref={solidRef} geometry={geometry}>
        <meshStandardMaterial
          color="#c8ccd4"
          side={THREE.DoubleSide}
          roughness={0.85}
          metalness={0.0}
          flatShading
        />
      </mesh>
    </group>
  );
}

function VisitedOverlay({ waypoints, visitedUpTo }: { waypoints: WaypointData[]; visitedUpTo: number }) {
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const count = Math.min(visitedUpTo + 1, waypoints.length);
  const [radius, wSeg, hSeg] = sphereArgs(waypoints.length);

  useEffect(() => {
    if (!meshRef.current || count <= 0) return;
    const dummy = new THREE.Object3D();
    for (let i = 0; i < count; i++) {
      const wp = waypoints[i];
      dummy.position.set(...enu(wp.x, wp.y, wp.z));
      dummy.updateMatrix();
      meshRef.current.setMatrixAt(i, dummy.matrix);
    }
    meshRef.current.instanceMatrix.needsUpdate = true;
    meshRef.current.count = count;
  }, [waypoints, count]);

  if (count <= 0) return null;

  return (
    <instancedMesh ref={meshRef} args={[undefined, undefined, waypoints.length]} frustumCulled={false}>
      <sphereGeometry args={[radius + 0.02, wSeg, hSeg]} />
      <meshStandardMaterial color="#888" transparent opacity={0.35} />
    </instancedMesh>
  );
}

function Scene({ data, onWaypointClick, visitedIndex, showRawMesh, captureView, activeFacades }: { data: ThreeJSData; onWaypointClick: (wp: WaypointData | null) => void; visitedIndex: number; showRawMesh: boolean; captureView: boolean; activeFacades: Set<number> | null }) {
  const { raycaster } = useThree();

  // Separate inspection and transition waypoints
  const { facadeWaypoints, transitionWaypoints } = useMemo(() => {
    const groups: Record<number, WaypointData[]> = {};
    const transit: WaypointData[] = [];
    data.waypoints.forEach((wp) => {
      if (wp.is_transition) {
        transit.push(wp);
      } else {
        (groups[wp.facade_index] ||= []).push(wp);
      }
    });
    return { facadeWaypoints: groups, transitionWaypoints: transit };
  }, [data.waypoints]);

  const handleClick = useCallback(
    (e: ThreeEvent<MouseEvent>) => {
      // Find closest waypoint to click ray
      const ray = raycaster.ray;
      let bestDist = 1.5;
      let bestWp: WaypointData | null = null;
      const pt = new THREE.Vector3();
      data.waypoints.forEach((wp) => {
        pt.set(...enu(wp.x, wp.y, wp.z));
        const d = ray.distanceToPoint(pt);
        if (d < bestDist) {
          bestDist = d;
          bestWp = wp;
        }
      });
      onWaypointClick(bestWp);
      e.stopPropagation();
    },
    [data.waypoints, raycaster, onWaypointClick],
  );

  return (
    <group onClick={handleClick}>
      {/* Ground */}
      <gridHelper args={[100, 50, '#333355', '#222244']} />
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]}>
        <planeGeometry args={[100, 100]} />
        <meshStandardMaterial color="#0f1117" roughness={1} />
      </mesh>

      {/* Facades */}
      {data.facades.map((f) => {
        if (captureView && activeFacades) {
          const isActive = activeFacades.has(f.index);
          return (
            <FacadeMesh key={f.index} vertices={f.vertices} color={f.color}
              hidden={!isActive} highlighted={isActive} />
          );
        }
        // Capture view with nothing selected: hide all facades (grey mesh only)
        if (captureView) return <FacadeMesh key={f.index} vertices={f.vertices} color={f.color} hidden />;
        return <FacadeMesh key={f.index} vertices={f.vertices} color={f.color} />;
      })}

      {/* Waypoints + arrows */}
      {Object.entries(facadeWaypoints).map(([fi, wps]) => {
        const facadeIdx = parseInt(fi);
        const facade = data.facades.find((f) => f.index === facadeIdx);
        const color = facade?.color || '#888';
        if (captureView && activeFacades) {
          const isActive = activeFacades.has(facadeIdx);
          return (
            <group key={fi}>
              <WaypointSpheres waypoints={wps} color={isActive ? color : '#1a1a2a'} bright={isActive} />
              {isActive && <CameraArrows waypoints={wps} color={color} bright />}
            </group>
          );
        }
        if (captureView) {
          // Nothing selected: dim everything
          return (
            <group key={fi}>
              <WaypointSpheres waypoints={wps} color="#333" />
            </group>
          );
        }
        return (
          <group key={fi}>
            <WaypointSpheres waypoints={wps} color={color} />
            <CameraArrows waypoints={wps} color={color} />
          </group>
        );
      })}

      {/* Transition waypoints */}
      {transitionWaypoints.length > 0 && (
        <WaypointSpheres waypoints={transitionWaypoints} color={captureView ? '#111122' : '#555'} />
      )}

      {/* Flight path */}
      <FlightPath waypoints={data.waypoints} />

      {/* Grey overlay on visited waypoints during playback */}
      {visitedIndex >= 0 && (
        <VisitedOverlay waypoints={data.waypoints} visitedUpTo={visitedIndex} />
      )}

      {/* Raw 3D mesh from uploaded file */}
      {showRawMesh && data.rawMesh && (
        <RawMeshView mesh={data.rawMesh} dimmed={captureView && activeFacades != null} />
      )}
    </group>
  );
}

interface CameraFOV {
  fov_h_deg: number;
  fov_v_deg: number;
  distance_m: number;
}

interface Timeline {
  totalTime: number;
  cumTimes: number[];
}

const PIP_W = 280;
const PIP_H = 210;
const PIP_X = 12;
const PIP_TOP = 12; // CSS px from top of canvas
const SNAP_W = 200;
const SNAP_H = 150;

function CameraPreview({ waypoints, progress, cameraFov, timeline, onSnapshot }: {
  waypoints: WaypointData[];
  progress: number;
  cameraFov?: CameraFOV;
  timeline?: Timeline;
  onSnapshot: (dataUrl: string, wpIdx: number, photoNum: number) => void;
}) {
  const pipCameraRef = useRef<THREE.PerspectiveCamera>(null);
  const { gl, scene, camera, size } = useThree();
  const lastPhotoWpRef = useRef(-1);
  const photoCountRef = useRef(0);

  const positions = useMemo(
    () => waypoints.map((wp) => new THREE.Vector3(wp.x, wp.z, -wp.y)),
    [waypoints],
  );

  // Offscreen render target for snapshot capture
  const renderTarget = useMemo(() => {
    const rt = new THREE.WebGLRenderTarget(SNAP_W * 2, SNAP_H * 2);
    return rt;
  }, []);

  // Reset photo counter on new data
  useEffect(() => {
    lastPhotoWpRef.current = -1;
    photoCountRef.current = 0;
  }, [waypoints]);

  const vFov = cameraFov?.fov_v_deg || 50;

  useFrame(() => {
    if (!pipCameraRef.current || positions.length < 2) return;

    // Interpolate drone position (same logic as DroneMarker)
    let idx: number, t: number;
    if (timeline && timeline.cumTimes.length === positions.length) {
      const currentTime = progress * timeline.totalTime;
      idx = 0;
      for (let i = 1; i < timeline.cumTimes.length; i++) {
        if (timeline.cumTimes[i] >= currentTime) break;
        idx = i;
      }
      idx = Math.min(idx, positions.length - 2);
      const segStart = timeline.cumTimes[idx];
      const segEnd = timeline.cumTimes[idx + 1];
      const segDur = segEnd - segStart;
      t = segDur > 0 ? Math.min((currentTime - segStart) / segDur, 1) : 0;
    } else {
      const n = positions.length - 1;
      const rawIdx = progress * n;
      idx = Math.min(Math.floor(rawIdx), n - 1);
      t = rawIdx - idx;
    }

    const wpIdx = Math.min(idx + 1, waypoints.length - 1);
    const wp = waypoints[wpIdx];
    const eased = wp.is_transition ? t * t * (3 - 2 * t) : t;
    const pos = new THREE.Vector3().lerpVectors(positions[idx], positions[idx + 1], eased);

    // Position + orient PIP camera to match drone heading + gimbal
    pipCameraRef.current.position.copy(pos);
    const headingRad = -(wp.heading * Math.PI) / 180;
    const pitchRad = (wp.gimbal_pitch * Math.PI) / 180;
    pipCameraRef.current.rotation.set(pitchRad, headingRad, 0, 'YXZ');
    pipCameraRef.current.aspect = PIP_W / PIP_H;
    pipCameraRef.current.updateProjectionMatrix();
    pipCameraRef.current.updateMatrixWorld();

    // Layer setup:
    // Main camera: layer 0 (base + ghost mesh) + 1 (drone)
    // PIP camera:  layer 0 (base + ghost mesh) + 3 (solid mesh), no drone
    // The solid opaque mesh on layer 3 renders over the transparent ghost mesh on layer 0
    camera.layers.enable(DRONE_LAYER);
    pipCameraRef.current.layers.set(0);
    pipCameraRef.current.layers.enable(MESH_SOLID_LAYER);

    // 1. Render main scene (full viewport, auto-clear)
    gl.setViewport(0, 0, size.width, size.height);
    gl.render(scene, camera);

    // 2. Render PIP (drone camera view — solid building)
    const pipY = size.height - PIP_TOP - PIP_H; // WebGL y is bottom-up
    gl.autoClear = false;
    gl.setScissorTest(true);
    gl.setScissor(PIP_X, pipY, PIP_W, PIP_H);
    gl.setViewport(PIP_X, pipY, PIP_W, PIP_H);
    gl.clear(true, true, false);
    gl.render(scene, pipCameraRef.current);
    gl.setScissorTest(false);
    gl.setViewport(0, 0, size.width, size.height);
    gl.autoClear = true;

    // 3. Capture snapshot at photo waypoints (t > 0.85, inspection WP)
    if (!wp.is_transition && t > 0.85 && lastPhotoWpRef.current !== wpIdx) {
      lastPhotoWpRef.current = wpIdx;
      photoCountRef.current++;

      // Render to offscreen target at snapshot camera position
      pipCameraRef.current.aspect = SNAP_W / SNAP_H;
      pipCameraRef.current.updateProjectionMatrix();
      gl.setRenderTarget(renderTarget);
      gl.clear(true, true, false);
      gl.render(scene, pipCameraRef.current);
      gl.setRenderTarget(null);

      // Read pixels and create data URL
      const w = renderTarget.width, h = renderTarget.height;
      const buf = new Uint8Array(w * h * 4);
      gl.readRenderTargetPixels(renderTarget, 0, 0, w, h, buf);
      const c = document.createElement('canvas');
      c.width = w; c.height = h;
      const ctx = c.getContext('2d')!;
      const img = ctx.createImageData(w, h);
      for (let y = 0; y < h; y++) {
        const src = (h - y - 1) * w * 4;
        const dst = y * w * 4;
        img.data.set(buf.subarray(src, src + w * 4), dst);
      }
      ctx.putImageData(img, 0, 0);
      onSnapshot(c.toDataURL('image/jpeg', 0.7), wpIdx, photoCountRef.current);
    }
  }, 1); // priority 1: take over rendering for dual viewport

  return (
    <perspectiveCamera
      ref={pipCameraRef}
      fov={vFov}
      aspect={PIP_W / PIP_H}
      near={0.1}
      far={200}
    />
  );
}

export function Viewer3D({ data, cameraFov }: { data: ThreeJSData | null; cameraFov?: CameraFOV }) {
  const [selectedWp, setSelectedWp] = useState<WaypointData | null>(null);
  const [playing, setPlaying] = useState(false);
  const [progress, setProgress] = useState(0);
  const [speed, setSpeed] = useState(1);
  const [snapshot, setSnapshot] = useState<{ url: string; wpIdx: number; photoNum: number } | null>(null);
  const [usePhysics, setUsePhysics] = useState(true);
  const [showRawMesh, setShowRawMesh] = useState(true);
  const [captureView, setCaptureView] = useState(false);
  const [captureDir, setCaptureDir] = useState<string | null>(null);
  const showPip = playing || progress > 0;
  const animRef = useRef<number | null>(null);
  const lastTimeRef = useRef(0);

  // Reset playback when data changes
  useEffect(() => {
    setPlaying(false);
    setProgress(0);
    setSnapshot(null);
  }, [data]);

  const handleSnapshot = useCallback((url: string, wpIdx: number, photoNum: number) => {
    setSnapshot({ url, wpIdx, photoNum });
  }, []);

  // Compute physics-based timeline: segment times from distances + speeds + dwell
  const timeline = useMemo(() => {
    if (!data || data.waypoints.length < 2) return { totalTime: 1, cumTimes: [0, 1] };
    const wps = data.waypoints;
    const DWELL_S = 1.0;       // hover time at each inspection WP for photo
    const INSPECT_SPEED = 3.0; // m/s
    const TRANSIT_SPEED = 9.0; // m/s

    const segTimes: number[] = [0]; // cumulative time at each WP
    let t = 0;
    for (let i = 1; i < wps.length; i++) {
      const dx = wps[i].x - wps[i - 1].x;
      const dy = wps[i].y - wps[i - 1].y;
      const dz = wps[i].z - wps[i - 1].z;
      const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
      const spd = wps[i].is_transition ? TRANSIT_SPEED : INSPECT_SPEED;
      t += dist / spd;
      // Dwell at inspection waypoints
      if (!wps[i].is_transition) t += DWELL_S;
      segTimes.push(t);
    }
    return { totalTime: t, cumTimes: segTimes };
  }, [data]);

  // Animation loop
  useEffect(() => {
    if (!playing || !data) return;

    const baseSec = usePhysics
      ? timeline.totalTime
      : Math.max(5, data.waypoints.length * 0.08);
    const duration = (baseSec / speed) * 1000;

    lastTimeRef.current = performance.now();

    const tick = (now: number) => {
      const dt = now - lastTimeRef.current;
      lastTimeRef.current = now;
      setProgress((prev) => {
        const next = prev + dt / duration;
        if (next >= 1) {
          setPlaying(false);
          return 1;
        }
        return next;
      });
      animRef.current = requestAnimationFrame(tick);
    };

    animRef.current = requestAnimationFrame(tick);
    return () => {
      if (animRef.current) cancelAnimationFrame(animRef.current);
    };
  }, [playing, speed, data]);

  // Group facades and waypoints by cardinal direction for legend
  // (must be before early return to satisfy Rules of Hooks)
  const directionGroups = useMemo(() => {
    if (!data) return {};
    const groups: Record<string, { color: string; facades: number; waypoints: number }> = {};
    for (const f of data.facades) {
      const dir = f.direction || 'other';
      if (!groups[dir]) groups[dir] = { color: f.color, facades: 0, waypoints: 0 };
      groups[dir].facades++;
    }
    for (const wp of data.waypoints) {
      if (wp.is_transition) continue;
      const facade = data.facades.find((f) => f.index === wp.facade_index);
      const dir = facade?.direction || 'other';
      if (groups[dir]) groups[dir].waypoints++;
    }
    return groups;
  }, [data]);

  // Current waypoint index for display (must be before early return for hook order)
  const currentWpIdx = data
    ? Math.min(Math.floor(progress * (data.waypoints.length - 1)), data.waypoints.length - 1)
    : 0;
  const currentWp = data?.waypoints[currentWpIdx] ?? null;

  // Active facade set for capture view
  const activeFacadeSet = useMemo(() => {
    if (!captureView || !data) return null;
    if (selectedWp) return new Set([selectedWp.facade_index]);
    if (progress > 0 && currentWp && !currentWp.is_transition) return new Set([currentWp.facade_index]);
    if (captureDir) {
      const indices = new Set<number>();
      for (const f of data.facades) {
        if (f.direction === captureDir) indices.add(f.index);
      }
      return indices.size > 0 ? indices : null;
    }
    return null;
  }, [captureView, selectedWp, progress, currentWp, captureDir, data]);

  if (!data) {
    return <div className="empty-state">Ariana Engineering &times; AeroScan</div>;
  }

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      <Canvas
        camera={{ position: [35, 25, 35], fov: 50, near: 0.1, far: 500 }}
        style={{ background: '#0f1117' }}
        frameloop={playing ? 'always' : 'demand'}
        onClick={() => setSelectedWp(null)}
      >
        <ambientLight intensity={0.5} />
        <directionalLight position={[20, 40, 30]} intensity={0.8} />
        <hemisphereLight args={['#4488cc', '#332211', 0.3]} />
        <fog attach="fog" args={['#0f1117', 80, 200]} />
        <OrbitControls
          target={[0, data.buildingHeight / 2, 0]}
          enableDamping
          dampingFactor={0.08}
        />
        <Scene data={data} onWaypointClick={setSelectedWp} visitedIndex={playing || progress > 0 ? getVisitedIndex(progress, data.waypoints.length) : -1} showRawMesh={showRawMesh} captureView={captureView} activeFacades={activeFacadeSet} />
        {(playing || progress > 0) && (
          <DroneMarker waypoints={data.waypoints} progress={progress} cameraFov={cameraFov} timeline={usePhysics ? timeline : undefined} />
        )}
        {showPip && (
          <CameraPreview waypoints={data.waypoints} progress={progress} cameraFov={cameraFov} timeline={usePhysics ? timeline : undefined} onSnapshot={handleSnapshot} />
        )}
      </Canvas>

      {/* Camera preview frame */}
      {showPip && (
        <div className="camera-pip-frame">
          <span className="camera-pip-label">Camera Preview</span>
          <span className="camera-pip-wp">WP {currentWpIdx} · {currentWp?.heading}° · {currentWp?.gimbal_pitch}°</span>
        </div>
      )}

      {/* Last captured photo */}
      {snapshot && (
        <div className="snapshot-frame">
          <img src={snapshot.url} alt={`Photo ${snapshot.photoNum}`} />
          <div className="snapshot-info">
            <span className="snapshot-label">Photo {snapshot.photoNum}</span>
            <span className="snapshot-wp">WP {snapshot.wpIdx}</span>
          </div>
        </div>
      )}

      {/* Legend */}
      <div className="legend-3d">
        <label className="legend-item" style={{ cursor: 'pointer', marginBottom: 4 }}>
          <input
            type="checkbox"
            checked={captureView}
            onChange={(e) => setCaptureView(e.target.checked)}
            style={{ marginRight: 6 }}
          />
          <span>Capture view</span>
        </label>
        {data.rawMesh && (
          <label className="legend-item" style={{ cursor: 'pointer', marginBottom: 4 }}>
            <input
              type="checkbox"
              checked={showRawMesh}
              onChange={(e) => setShowRawMesh(e.target.checked)}
              style={{ marginRight: 6 }}
            />
            <span>Raw mesh</span>
          </label>
        )}
        {Object.entries(directionGroups).map(([dir, g]) => {
          const isActive = captureView && captureDir === dir;
          return (
            <div key={dir} className={`legend-item${captureView ? ' clickable' : ''}`}
              onClick={captureView ? () => setCaptureDir(captureDir === dir ? null : dir) : undefined}
              style={captureView ? { cursor: 'pointer' } : undefined}>
              <span className="legend-dot" style={{
                backgroundColor: isActive ? g.color : captureView ? '#333344' : g.color,
                boxShadow: isActive ? `0 0 6px ${g.color}` : undefined,
              }} />
              <span style={{ color: isActive ? 'var(--fg)' : captureView ? 'var(--fg3)' : undefined }}>
                {dir} ({g.facades}f / {g.waypoints}wp)
              </span>
            </div>
          );
        })}
      </div>

      {/* Playback controls */}
      <div className="playback-bar">
        <button
          className="play-btn"
          onClick={() => {
            if (progress >= 1) setProgress(0);
            setPlaying(!playing);
          }}
        >
          {playing ? '⏸' : '▶'}
        </button>
        <input
          type="range"
          className="progress-slider"
          min={0}
          max={1}
          step={0.001}
          value={progress}
          onChange={(e) => {
            setProgress(parseFloat(e.target.value));
            setPlaying(false);
          }}
        />
        <span className="playback-wp">
          WP {currentWpIdx}/{data.waypoints.length - 1}
          {currentWp?.is_transition ? ' (transit)' : ''}
        </span>
        <select
          className="speed-select"
          value={speed}
          onChange={(e) => setSpeed(parseFloat(e.target.value))}
        >
          <option value={0.25}>0.25x</option>
          <option value={0.5}>0.5x</option>
          <option value={1}>1x</option>
          <option value={2}>2x</option>
          <option value={4}>4x</option>
        </select>
        <div className="playback-toggle">
          <button
            className={`playback-toggle-btn ${usePhysics ? 'active' : ''}`}
            onClick={() => setUsePhysics(true)}
          >
            Real
          </button>
          <button
            className={`playback-toggle-btn ${!usePhysics ? 'active' : ''}`}
            onClick={() => setUsePhysics(false)}
          >
            Fast
          </button>
        </div>
        <button
          className="play-btn"
          onClick={() => { setProgress(0); setPlaying(false); }}
          title="Reset"
        >
          ↺
        </button>
      </div>

      {/* Waypoint info */}
      {selectedWp && (
        <div className="wp-info">
          <b>WP {selectedWp.index}</b><br />
          Facade: {data.facades.find((f) => f.index === selectedWp.facade_index)?.label}<br />
          Pos: ({selectedWp.x}, {selectedWp.y}, {selectedWp.z})m<br />
          Heading: {selectedWp.heading}° · Gimbal: {selectedWp.gimbal_pitch}°<br />
          Component: {selectedWp.component}
        </div>
      )}
    </div>
  );
}
