import { useRef, useMemo, useState, useCallback, useEffect } from 'react';
import { Canvas, useThree, useFrame, type ThreeEvent } from '@react-three/fiber';
import { OrbitControls, Line, PivotControls } from '@react-three/drei';
import * as THREE from 'three';
import type { MappingBox, MissionAreaData, PointCloudData, RawMeshData, ThreeJSData, WaypointData } from '../api/types';
import { DroneMarker, getVisitedIndex, DRONE_LAYER } from './DroneAnimation';
import { useStore } from '../store';

// Layers: 0 = base scene + ghost mesh, 1 = drone (hidden from PIP), 3 = solid mesh (PIP only)
const MESH_SOLID_LAYER = 3;

/** Repositions the camera to frame the scene when the centroid changes. */
function CameraReframe({ target, radius }: { target: [number, number, number]; radius: number }) {
  const { camera, controls } = useThree() as { camera: THREE.PerspectiveCamera; controls: { target: THREE.Vector3; update: () => void } | null };
  useEffect(() => {
    const d = Math.max(radius * 1.8, 25);
    camera.position.set(target[0] + d * 0.8, target[1] + d * 0.6, target[2] + d * 0.8);
    camera.lookAt(target[0], target[1], target[2]);
    camera.updateProjectionMatrix();
    if (controls?.target) {
      controls.target.set(target[0], target[1], target[2]);
      controls.update();
    }
  }, [target[0], target[1], target[2], radius, camera, controls]);
  return null;
}

/** Component that enables MESH_SOLID_LAYER on all scene lights so the PIP camera can see lit surfaces */
function EnableLightLayers() {
  const { scene } = useThree();
  useEffect(() => {
    scene.traverse((obj) => {
      if ((obj as THREE.Light).isLight) {
        obj.layers.enable(MESH_SOLID_LAYER);
      }
    });
  });
  return null;
}

// Convert ENU (x=East, y=North, z=Up) to Three.js (x=right, y=up, z=toward camera)
function enu(x: number, y: number, z: number): [number, number, number] {
  return [x, z, -y];
}

function FacadeMesh({ vertices, color, hidden, highlighted, dimmed, disabled, onClick }: { vertices: number[][]; color: string; hidden?: boolean; highlighted?: boolean; dimmed?: boolean; disabled?: boolean; onClick?: (e: ThreeEvent<MouseEvent>) => void }) {
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

  const displayColor = disabled ? '#555' : dimmed ? '#667' : color;
  const opacity = disabled ? 0.12 : dimmed ? 0.15 : highlighted ? 0.55 : 0.2;
  const occlude = !disabled && !dimmed; // depth-only pass blocks things behind the facade

  return (
    <group onClick={onClick}>
      {/* Pass 1: invisible depth-only — writes to z-buffer so waypoints behind are occluded */}
      {occlude && (
        <mesh geometry={geometry} renderOrder={-1}>
          <meshBasicMaterial colorWrite={false} depthWrite side={THREE.DoubleSide} />
        </mesh>
      )}
      {/* Pass 2: visible transparent — does NOT write depth, avoids sorting artifacts */}
      <mesh geometry={geometry} renderOrder={highlighted ? 10 : 1}>
        <meshStandardMaterial
          color={displayColor} transparent
          opacity={opacity}
          side={THREE.DoubleSide} roughness={0.8}
          depthWrite={false}
          depthTest
          emissive={highlighted ? color : '#000000'}
          emissiveIntensity={highlighted ? 0.3 : 0}
        />
      </mesh>
      <Line points={edgePoints} color={displayColor} lineWidth={highlighted ? 2.5 : disabled ? 0.5 : dimmed ? 0.8 : 1.5} opacity={disabled ? 0.3 : dimmed ? 0.5 : 1} transparent={disabled || dimmed} />
    </group>
  );
}

// Adaptive sphere detail: fewer polys when there are many waypoints
function sphereArgs(count: number): [number, number, number] {
  if (count > 2000) return [0.08, 3, 2];
  if (count > 500)  return [0.1, 4, 3];
  return [0.13, 6, 4];
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
      <meshStandardMaterial color={color} transparent opacity={bright ? 0.95 : 0.65} roughness={0.4} metalness={bright ? 0.2 : 0.1} emissive={bright ? color : '#000000'} emissiveIntensity={bright ? 0.3 : 0} />
    </instancedMesh>
  );
}

interface CameraFOV {
  fov_h_deg: number;
  fov_v_deg: number;
  distance_m: number;
}

function CameraArrows({ waypoints, color, bright, cameraFov }: { waypoints: WaypointData[]; color: string; bright?: boolean; cameraFov?: CameraFOV }) {
  // For large sets, only draw every Nth arrow
  const stride = waypoints.length > 2000 ? 4 : waypoints.length > 500 ? 2 : 1;
  // Full frustum wireframe on sparse subset only to avoid visual noise
  const frustumStride = Math.max(stride, waypoints.length > 200 ? 12 : waypoints.length > 50 ? 8 : 4);

  const geometry = useMemo(() => {
    const positions: number[] = [];
    const arrowLen = cameraFov ? Math.min(cameraFov.distance_m * 0.4, 3) : 2;
    const frustumDist = cameraFov ? Math.min(cameraFov.distance_m, 8) : 2;
    const tanH = cameraFov ? Math.tan((cameraFov.fov_h_deg / 2) * Math.PI / 180) : 0;
    const tanV = cameraFov ? Math.tan((cameraFov.fov_v_deg / 2) * Math.PI / 180) : 0;
    const hasFov = tanH > 0 && tanV > 0;

    for (let j = 0; j < waypoints.length; j += stride) {
      const wp = waypoints[j];
      const [ox, oy, oz] = enu(wp.x, wp.y, wp.z);
      const hr = (wp.heading * Math.PI) / 180;
      const pr = (wp.gimbal_pitch * Math.PI) / 180;

      // Forward direction in ENU
      const fwd_x = Math.sin(hr) * Math.cos(pr);
      const fwd_y = Math.cos(hr) * Math.cos(pr);
      const fwd_z = Math.sin(pr);

      // Short direction line for every sampled waypoint
      const [ex, ey, ez] = enu(wp.x + fwd_x * arrowLen, wp.y + fwd_y * arrowLen, wp.z + fwd_z * arrowLen);
      positions.push(ox, oy, oz, ex, ey, ez);

      // Full frustum wireframe on sparse subset only
      if (!hasFov || j % frustumStride !== 0) continue;

      const r_x = Math.cos(hr);
      const r_y = -Math.sin(hr);
      const u_x = r_y * fwd_z;
      const u_y = -r_x * fwd_z;
      const u_z = r_x * fwd_y - r_y * fwd_x;

      const corners: [number, number, number][] = [];
      for (const [sh, sv] of [[-1, -1], [1, -1], [1, 1], [-1, 1]] as const) {
        const cx = wp.x + (fwd_x + r_x * tanH * sh + u_x * tanV * sv) * frustumDist;
        const cy = wp.y + (fwd_y + r_y * tanH * sh + u_y * tanV * sv) * frustumDist;
        const cz = wp.z + (fwd_z + u_z * tanV * sv) * frustumDist;
        const [tx, ty, tz] = enu(cx, cy, cz);
        corners.push([tx, ty, tz]);
        positions.push(ox, oy, oz, tx, ty, tz);
      }
      for (let i = 0; i < 4; i++) {
        const a = corners[i], b = corners[(i + 1) % 4];
        positions.push(a[0], a[1], a[2], b[0], b[1], b[2]);
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    return geo;
  }, [waypoints, stride, frustumStride, cameraFov]);

  return (
    <lineSegments geometry={geometry}>
      <lineBasicMaterial color={color} transparent opacity={bright ? 0.6 : 0.25} />
    </lineSegments>
  );
}

function OriginalGimbalArrows({ waypoints, color, cameraFov }: { waypoints: WaypointData[]; color: string; cameraFov?: CameraFOV }) {
  // Match DJI's Smart 3D Capture Quality Report: one frustum per photo,
  // which means one per pose in the SmartOblique rosette — typically 5 per
  // waypoint (1 nadir + 4 obliques). Falls back to the placemark's planned
  // pitch/yaw for waypoints without a rosette (e.g. plain waylines).
  // Use real camera intrinsics from summary.camera when available; otherwise
  // fall back to M4E wide defaults.
  const fovHDeg = cameraFov?.fov_h_deg ?? 71.5;
  const fovVDeg = cameraFov?.fov_v_deg ?? 56.9;
  const frustumLen = cameraFov ? Math.min(cameraFov.distance_m * 0.08, 0.6) : 0.4;
  const halfW = frustumLen * Math.tan((fovHDeg * Math.PI) / 180 / 2);
  const halfH = frustumLen * Math.tan((fovVDeg * Math.PI) / 180 / 2);

  const { edgeGeo } = useMemo(() => {
    const edgePositions: number[] = [];
    for (const wp of waypoints) {
      const plannedPitch = wp.pitch_before !== undefined ? wp.pitch_before : wp.gimbal_pitch;
      const plannedYaw = wp.yaw_before !== undefined ? wp.yaw_before : wp.heading;
      const poses = wp.smart_oblique_poses && wp.smart_oblique_poses.length > 0
        ? wp.smart_oblique_poses
        : [{ pitch: plannedPitch, yaw: plannedYaw }];

      for (const p of poses) {
        const yr = (p.yaw * Math.PI) / 180;
        const pr = (p.pitch * Math.PI) / 180;
        // Forward (look direction) in ENU
        const fx = Math.sin(yr) * Math.cos(pr);
        const fy = Math.cos(yr) * Math.cos(pr);
        const fz = Math.sin(pr);
        // Right vector (horizontal, perpendicular to forward & world up)
        const rx = Math.cos(yr);
        const ry = -Math.sin(yr);
        const rz = 0;
        // Up vector = right × forward (ENU, right-handed)
        const ux = ry * fz - rz * fy;
        const uy = rz * fx - rx * fz;
        const uz = rx * fy - ry * fx;

        // Apex (waypoint) in THREE coords
        const [ax, ay, az] = enu(wp.x, wp.y, wp.z);
        // Base center
        const bcx = wp.x + fx * frustumLen;
        const bcy = wp.y + fy * frustumLen;
        const bcz = wp.z + fz * frustumLen;
        // 4 base corners (TL, TR, BR, BL)
        const corners: [number, number, number][] = [
          enu(bcx - rx * halfW + ux * halfH, bcy - ry * halfW + uy * halfH, bcz - rz * halfW + uz * halfH),
          enu(bcx + rx * halfW + ux * halfH, bcy + ry * halfW + uy * halfH, bcz + rz * halfW + uz * halfH),
          enu(bcx + rx * halfW - ux * halfH, bcy + ry * halfW - uy * halfH, bcz + rz * halfW - uz * halfH),
          enu(bcx - rx * halfW - ux * halfH, bcy - ry * halfW - uy * halfH, bcz - rz * halfW - uz * halfH),
        ];
        // Apex → each corner (4 edges)
        for (const c of corners) {
          edgePositions.push(ax, ay, az, c[0], c[1], c[2]);
        }
        // Base rectangle (4 edges)
        for (let i = 0; i < 4; i++) {
          const a = corners[i];
          const b = corners[(i + 1) % 4];
          edgePositions.push(a[0], a[1], a[2], b[0], b[1], b[2]);
        }
      }
    }
    if (edgePositions.length === 0) return { edgeGeo: null };
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(edgePositions, 3));
    return { edgeGeo: g };
  }, [waypoints, frustumLen, halfW, halfH]);

  if (!edgeGeo) return null;
  return (
    <lineSegments geometry={edgeGeo}>
      <lineBasicMaterial color={color} transparent opacity={0.55} />
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

/** Renders the DJI tileset OBB as a semi-transparent block (matches DJI's
 *  Smart 3D Capture Quality Report "Mapping" toggle). Inputs are in ENU; we
 *  swizzle to Three.js (x=E, y=Up, z=-N) for center and each half-axis. */
function MappingBoxView({ box }: { box: MappingBox }) {
  const { position, matrix } = useMemo(() => {
    const enuToThree = (v: readonly [number, number, number]) =>
      new THREE.Vector3(v[0], v[2], -v[1]);
    const center = enuToThree(box.center);
    // Full-axis vectors (unit box is 2×2×2, so columns are full axes, not halves)
    const ax = enuToThree(box.axes[0]);
    const ay = enuToThree(box.axes[1]);
    const az = enuToThree(box.axes[2]);
    const m = new THREE.Matrix4().makeBasis(ax, ay, az);
    return { position: center, matrix: m };
  }, [box]);

  return (
    <group position={position}>
      <group matrixAutoUpdate={false} matrix={matrix}>
        <mesh>
          <boxGeometry args={[2, 2, 2]} />
          <meshBasicMaterial
            color="#4aa3ff"
            transparent
            opacity={0.18}
            side={THREE.DoubleSide}
            depthWrite={false}
          />
        </mesh>
        <lineSegments>
          <edgesGeometry args={[new THREE.BoxGeometry(2, 2, 2)]} />
          <lineBasicMaterial color="#7ec4ff" transparent opacity={0.8} />
        </lineSegments>
      </group>
    </group>
  );
}

function RawMeshView({ mesh, dimmed }: { mesh: RawMeshData; dimmed?: boolean }) {
  const solidRef = useRef<THREE.Mesh>(null);
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);

  useEffect(() => {
    let disposed = false;
    let prev: THREE.BufferGeometry | null = null;

    (async () => {
      const { positions, indices } = await decodeRawMesh(mesh);
      if (disposed) return;

      const geo = new THREE.BufferGeometry();
      // Convert ENU (x=E, y=N, z=Up) → Three.js (x=right, y=up, z=toward camera).
      // Swap in-place on the Float32Array to avoid a second allocation.
      for (let i = 0; i < positions.length; i += 3) {
        const y = positions[i + 1];
        const z = positions[i + 2];
        positions[i + 1] = z;
        positions[i + 2] = -y;
      }
      geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geo.setIndex(new THREE.BufferAttribute(indices, 1));
      geo.computeVertexNormals();
      setGeometry((old) => {
        prev = old;
        return geo;
      });
      if (prev) prev.dispose();
    })();

    return () => {
      disposed = true;
    };
  }, [mesh]);

  // Solid mesh only visible to PIP camera (layer 3); ghost mesh stays on default layer 0
  useEffect(() => {
    if (solidRef.current) {
      solidRef.current.layers.set(MESH_SOLID_LAYER);
    }
  });

  if (!geometry) return null;

  return (
    <group>
      {/* Building mesh: prominent in orbital view, dimmed during flight */}
      <group>
        <mesh geometry={geometry} renderOrder={-1}>
          <meshStandardMaterial
            color="#a0b0c8"
            transparent
            opacity={dimmed ? 0.1 : 0.7}
            side={THREE.DoubleSide}
            roughness={0.5}
            metalness={0.15}
            depthWrite={!dimmed}
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

/** Decode rawMesh (binary base64+gzip fast path or legacy flat-list fallback). */
async function decodeRawMesh(mesh: RawMeshData): Promise<{ positions: Float32Array; indices: Uint32Array }> {
  if (mesh.positions_b64 && mesh.indices_b64) {
    const [posBuf, idxBuf] = await Promise.all([
      gunzipBase64(mesh.positions_b64),
      gunzipBase64(mesh.indices_b64),
    ]);
    // Stored as Float32Array (positions) + Int32Array (indices).
    const positions = new Float32Array(posBuf.buffer, posBuf.byteOffset, posBuf.byteLength / 4).slice();
    const indicesI32 = new Int32Array(idxBuf.buffer, idxBuf.byteOffset, idxBuf.byteLength / 4);
    const indices = new Uint32Array(indicesI32);
    return { positions, indices };
  }
  const posArr = mesh.positions ?? [];
  const idxArr = mesh.indices ?? [];
  return {
    positions: Float32Array.from(posArr),
    indices: Uint32Array.from(idxArr),
  };
}

async function gunzipBase64(b64: string): Promise<Uint8Array> {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  // DecompressionStream is available in all evergreen browsers.
  const stream = new Response(
    new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip')),
  );
  return new Uint8Array(await stream.arrayBuffer());
}

function PointCloudView({ data }: { data: PointCloudData }) {
  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const src = data.positions;
    const pos = new Float32Array(src.length);
    for (let i = 0; i < src.length; i += 3) {
      pos[i] = src[i];
      pos[i + 1] = src[i + 2];
      pos[i + 2] = -src[i + 1];
    }
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    if (data.colors && data.colors.length === src.length) {
      g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(data.colors), 3));
    }
    return g;
  }, [data]);

  const hasColor = data.colors && data.colors.length === data.positions.length;

  return (
    <points geometry={geometry} frustumCulled={false}>
      <pointsMaterial
        size={0.04}
        sizeAttenuation
        vertexColors={hasColor}
        color={hasColor ? undefined : '#9aa'}
        transparent
        opacity={0.9}
      />
    </points>
  );
}

function MissionAreaView({ data }: { data: MissionAreaData }) {
  const points = useMemo(() => {
    const pts = data.vertices.map((v) => enu(v[0], v[1], v[2]));
    if (pts.length > 2) pts.push(pts[0]);
    return pts;
  }, [data]);
  if (points.length < 2) return null;
  return (
    <Line points={points} color="#22d3ee" lineWidth={1.5} dashed dashSize={0.5} gapSize={0.3} />
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

type ViewMode = 'selection' | 'plan' | 'flight';

function Scene({ data, onWaypointClick, visitedIndex, viewMode, activeFacades, lightMode, cameraFov }: { data: ThreeJSData; onWaypointClick: (wp: WaypointData | null) => void; visitedIndex: number; viewMode: ViewMode; activeFacades: Set<number> | null; lightMode: boolean; cameraFov?: CameraFOV }) {
  const { raycaster } = useThree();
  const disabledFacades = useStore((s) => s.disabledFacades);
  const toggleFacade = useStore((s) => s.toggleFacade);
  const enabledCandidates = useStore((s) => s.enabledCandidates);
  const toggleCandidate = useStore((s) => s.toggleCandidate);
  const exclusionZones = useStore((s) => s.exclusionZones);
  const updateExclusionZone = useStore((s) => s.updateExclusionZone);
  const showOriginalGimbals = useStore((s) => s.showOriginalGimbals);
  const showMappingBox = useStore((s) => s.showMappingBox);

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
      <gridHelper args={[100, 50, lightMode ? '#b0b4c0' : '#333355', lightMode ? '#ccd0da' : '#222244']} />
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]}>
        <planeGeometry args={[100, 100]} />
        <meshStandardMaterial color={lightMode ? '#dde0e8' : '#0f1117'} roughness={1} />
      </mesh>

      {/* Facades */}
      {data.facades.map((f) => {
        const isDisabled = disabledFacades.has(f.index);
        const handleFacadeClick = (e: ThreeEvent<MouseEvent>) => { e.stopPropagation(); toggleFacade(f.index); };
        if (viewMode === 'selection') {
          return (
            <FacadeMesh key={f.index} vertices={f.vertices} color={f.color}
              hidden={isDisabled}
              highlighted={!isDisabled}
              onClick={handleFacadeClick} />
          );
        }
        if (viewMode === 'flight' && activeFacades) {
          const isActive = activeFacades.has(f.index);
          return (
            <FacadeMesh key={f.index} vertices={f.vertices} color={f.color}
              highlighted={isActive} dimmed={!isActive} disabled={isDisabled}
              onClick={handleFacadeClick} />
          );
        }
        if (viewMode === 'flight') {
          return (
            <FacadeMesh key={f.index} vertices={f.vertices} color={f.color}
              dimmed disabled={isDisabled}
              onClick={handleFacadeClick} />
          );
        }
        return (
          <FacadeMesh key={f.index} vertices={f.vertices} color={f.color}
            disabled={isDisabled}
            onClick={handleFacadeClick} />
        );
      })}

      {/* Candidate facades — rejected regions the user can click to include */}
      {viewMode === 'selection' && data.candidateFacades?.map((f) => {
        const isEnabled = enabledCandidates.has(f.index);
        return (
          <FacadeMesh key={`cand-${f.index}`} vertices={f.vertices}
            color={isEnabled ? '#44dd66' : '#444455'}
            highlighted={isEnabled}
            onClick={(e) => { e.stopPropagation(); toggleCandidate(f.index); }} />
        );
      })}

      {/* Exclusion zones — draggable via PivotControls */}
      {exclusionZones.map((zone) => {
        const color = zone.zone_type === 'no_fly' ? '#ff2222' : '#ff8800';
        const wireColor = zone.zone_type === 'no_fly' ? '#ff4444' : '#ffaa33';
        return (
          <PivotControls
            key={zone.id}
            scale={0.6}
            depthTest={false}
            disableRotations
            disableScaling
            activeAxes={[true, true, true]}
            anchor={[0, 0, 0]}
            offset={enu(zone.center_x, zone.center_y, zone.center_z)}
            onDragEnd={() => {
              // PivotControls applies transform to children — read from matrix
            }}
            onDrag={(localMatrix) => {
              const pos = new THREE.Vector3();
              localMatrix.decompose(pos, new THREE.Quaternion(), new THREE.Vector3());
              // PivotControls offset is in Three.js coords, delta is also Three.js
              // Three.js (x, y, z) -> ENU (x, -z, y) but PivotControls gives delta from offset
              updateExclusionZone(zone.id, {
                center_x: zone.center_x + pos.x,
                center_y: zone.center_y + (-pos.z),
                center_z: zone.center_z + pos.y,
              });
            }}
          >
            <group>
              <mesh>
                <boxGeometry args={[zone.size_x, zone.size_z, zone.size_y]} />
                <meshStandardMaterial color={color} transparent opacity={0.15} side={THREE.DoubleSide} depthWrite={false} />
              </mesh>
              <lineSegments>
                <edgesGeometry args={[new THREE.BoxGeometry(zone.size_x, zone.size_z, zone.size_y)]} />
                <lineBasicMaterial color={wireColor} transparent opacity={0.6} />
              </lineSegments>
            </group>
          </PivotControls>
        );
      })}

      {/* Waypoints + arrows — hidden in selection mode */}
      {viewMode !== 'selection' && Object.entries(facadeWaypoints).map(([fi, wps]) => {
        const facadeIdx = parseInt(fi);
        const facade = data.facades.find((f) => f.index === facadeIdx);
        const color = facade?.color || '#888';
        if (viewMode === 'flight' && activeFacades) {
          const isActive = activeFacades.has(facadeIdx);
          return (
            <group key={fi}>
              <WaypointSpheres waypoints={wps} color={isActive ? color : '#1a1a2a'} bright={isActive} />
              {isActive && <CameraArrows waypoints={wps} color={color} bright cameraFov={cameraFov} />}
              {isActive && showOriginalGimbals && <OriginalGimbalArrows waypoints={wps} color="#ff3344" cameraFov={cameraFov} />}
            </group>
          );
        }
        if (viewMode === 'flight') {
          return (
            <group key={fi}>
              <WaypointSpheres waypoints={wps} color="#333" />
            </group>
          );
        }
        // Plan mode: waypoint dots + simple direction lines (no frustum clutter)
        return (
          <group key={fi}>
            <WaypointSpheres waypoints={wps} color={color} />
            <CameraArrows waypoints={wps} color={color} />
            {showOriginalGimbals && <OriginalGimbalArrows waypoints={wps} color="#ff3344" cameraFov={cameraFov} />}
          </group>
        );
      })}

      {/* Transition waypoints */}
      {viewMode !== 'selection' && transitionWaypoints.length > 0 && (
        <WaypointSpheres waypoints={transitionWaypoints} color={viewMode === 'flight' ? '#111122' : '#555'} />
      )}

      {/* Flight path */}
      {viewMode !== 'selection' && <FlightPath waypoints={data.waypoints} />}

      {/* Grey overlay on visited waypoints during playback */}
      {viewMode !== 'selection' && visitedIndex >= 0 && (
        <VisitedOverlay waypoints={data.waypoints} visitedUpTo={visitedIndex} />
      )}

      {/* Raw 3D mesh from uploaded file */}
      {data.rawMesh && (
        <RawMeshView mesh={data.rawMesh} dimmed={viewMode === 'flight' && activeFacades != null} />
      )}

      {/* Imported DJI Smart3D reference point cloud */}
      {data.pointCloud && <PointCloudView data={data.pointCloud} />}

      {/* DJI tileset oriented bounding box (Mapping layer). Default off. */}
      {showMappingBox && data.mappingBox && <MappingBoxView box={data.mappingBox} />}

      {/* Mission-area polygon intentionally not drawn in 3D — it clutters the
          point cloud with an unrelated extruded boundary. Still rendered on the
          2D map tab via LeafletData.missionAreaPoly. */}
    </group>
  );
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

function SceneBackground({ lightMode }: { lightMode: boolean }) {
  const { scene } = useThree();
  useEffect(() => {
    scene.background = new THREE.Color(lightMode ? '#e8eaef' : '#0f1117');
  }, [lightMode, scene]);
  return null;
}

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
    pipCameraRef.current.layers.set(MESH_SOLID_LAYER);

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

export function Viewer3D({ data, cameraFov, defaultViewMode = 'plan' }: { data: ThreeJSData | null; cameraFov?: CameraFOV | null; defaultViewMode?: ViewMode }) {
  const [selectedWp, setSelectedWp] = useState<WaypointData | null>(null);
  const [playing, setPlaying] = useState(false);
  const [progress, setProgress] = useState(0);
  const [speed, setSpeed] = useState(1);
  const [snapshot, setSnapshot] = useState<{ url: string; wpIdx: number; photoNum: number } | null>(null);
  const [usePhysics, setUsePhysics] = useState(true);
  const [viewMode, setViewMode] = useState<ViewMode>(defaultViewMode);
  const [captureDir, setCaptureDir] = useState<string | null>(null);
  const { lightMode, disabledFacades, enabledCandidates, cameraFovOverride } = useStore();
  const effectiveCameraFov: CameraFOV | undefined = useMemo(() => {
    if (!cameraFov) return undefined;
    const o = cameraFovOverride;
    if (o.fov_h_deg == null && o.fov_v_deg == null && o.distance_m == null) return cameraFov;
    return {
      fov_h_deg: o.fov_h_deg ?? cameraFov.fov_h_deg,
      fov_v_deg: o.fov_v_deg ?? cameraFov.fov_v_deg,
      distance_m: o.distance_m ?? cameraFov.distance_m,
    };
  }, [cameraFov, cameraFovOverride]);
  const showPip = playing || progress > 0;
  const animRef = useRef<number | null>(null);
  const lastTimeRef = useRef(0);

  // Reset playback when data changes
  useEffect(() => {
    setPlaying(false);
    setProgress(0);
    setSnapshot(null);
  }, [data]);

  // Scene centroid + bounding radius in Three.js space. For DJI KMZ imports
  // we drive framing from the authoritative tileset OBB; otherwise fall back
  // to the waypoint/facade extent so uploaded-mesh scenes still frame.
  const { sceneCentroid, sceneRadius } = useMemo<{
    sceneCentroid: [number, number, number];
    sceneRadius: number;
  }>(() => {
    if (!data) return { sceneCentroid: [0, 0, 0], sceneRadius: 30 };
    if (data.mappingBox) {
      const { center, axes } = data.mappingBox;
      // ENU → Three.js (x=E, y=Up=ENU.z, z=-N)
      const cx = center[0], cy = center[2], cz = -center[1];
      // axes are full half-axis vectors; diagonal half-length bounds the box.
      const diag = Math.sqrt(
        axes[0].reduce((s, v) => s + v * v, 0) +
        axes[1].reduce((s, v) => s + v * v, 0) +
        axes[2].reduce((s, v) => s + v * v, 0),
      );
      return { sceneCentroid: [cx, cy, cz], sceneRadius: diag };
    }
    const pts: [number, number, number][] = [];
    for (const wp of data.waypoints) pts.push([wp.x, wp.z, -wp.y]);
    for (const f of data.facades) {
      for (const p of f.vertices) pts.push([p[0], p[2], -p[1]]);
    }
    if (pts.length === 0) return { sceneCentroid: [0, data.buildingHeight / 2, 0], sceneRadius: 30 };
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (const [x, y, z] of pts) {
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
      if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, cz = (minZ + maxZ) / 2;
    const radius = Math.max(maxX - minX, maxY - minY, maxZ - minZ) / 2;
    return { sceneCentroid: [cx, cy, cz], sceneRadius: radius };
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

  // Active facade set for flight view
  const activeFacadeSet = useMemo(() => {
    if (viewMode !== 'flight' || !data) return null;
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
  }, [viewMode, selectedWp, progress, currentWp, captureDir, data]);

  if (!data) {
    return <div className="empty-state">Ariana Engineering &times; AeroScan</div>;
  }

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      <Canvas
        camera={{ position: [35, 25, 35], fov: 50, near: 0.1, far: 500 }}
        style={{ background: lightMode ? '#e8eaef' : '#0f1117' }}
        frameloop={playing ? 'always' : 'demand'}
        onClick={() => setSelectedWp(null)}
      >
        <SceneBackground lightMode={lightMode} />
        <ambientLight intensity={lightMode ? 0.8 : 0.5} />
        <directionalLight position={[20, 40, 30]} intensity={lightMode ? 1.0 : 0.8} />
        <hemisphereLight args={[lightMode ? '#88aadd' : '#4488cc', lightMode ? '#ccbb99' : '#332211', lightMode ? 0.5 : 0.3]} />
        <EnableLightLayers />
        <fog attach="fog" args={[lightMode ? '#e8eaef' : '#0f1117', 80, 200]} />
        <OrbitControls
          makeDefault
          target={sceneCentroid}
          enableDamping
          dampingFactor={0.08}
        />
        <CameraReframe target={sceneCentroid} radius={sceneRadius} />
        <Scene data={data} onWaypointClick={setSelectedWp} visitedIndex={playing || progress > 0 ? getVisitedIndex(progress, data.waypoints.length) : -1} viewMode={viewMode} activeFacades={activeFacadeSet} lightMode={lightMode} cameraFov={effectiveCameraFov} />
        {(playing || progress > 0) && (
          <DroneMarker waypoints={data.waypoints} progress={progress} cameraFov={effectiveCameraFov} timeline={usePhysics ? timeline : undefined} />
        )}
        {showPip && (
          <CameraPreview waypoints={data.waypoints} progress={progress} cameraFov={effectiveCameraFov} timeline={usePhysics ? timeline : undefined} onSnapshot={handleSnapshot} />
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

      {/* View mode selector + legend */}
      <div className="legend-3d">
        <div style={{ display: 'flex', gap: 2, marginBottom: 6, background: 'rgba(0,0,0,0.3)', borderRadius: 5, padding: 2 }}>
          {(['selection', 'plan', 'flight'] as ViewMode[]).map((mode) => (
            <button
              key={mode}
              onClick={() => setViewMode(mode)}
              style={{
                flex: 1, fontSize: 10, padding: '3px 6px', border: 'none', borderRadius: 4, cursor: 'pointer',
                background: viewMode === mode ? 'var(--accent)' : 'transparent',
                color: viewMode === mode ? '#fff' : 'var(--fg3)',
                fontWeight: viewMode === mode ? 600 : 400,
              }}
            >
              {mode.charAt(0).toUpperCase() + mode.slice(1)}
            </button>
          ))}
        </div>
        {Object.entries(directionGroups).map(([dir, g]) => {
          const isActive = viewMode === 'flight' && captureDir === dir;
          return (
            <div key={dir} className={`legend-item${viewMode === 'flight' ? ' clickable' : ''}`}
              onClick={viewMode === 'flight' ? () => setCaptureDir(captureDir === dir ? null : dir) : undefined}
              style={viewMode === 'flight' ? { cursor: 'pointer' } : undefined}>
              <span className="legend-dot" style={{
                backgroundColor: isActive ? g.color : viewMode === 'flight' ? '#333344' : g.color,
                boxShadow: isActive ? `0 0 6px ${g.color}` : undefined,
              }} />
              <span style={{ color: isActive ? 'var(--fg)' : viewMode === 'flight' ? 'var(--fg3)' : undefined }}>
                {dir} ({g.facades}f / {g.waypoints}wp)
              </span>
            </div>
          );
        })}
      </div>

      {/* Selection mode HUD */}
      {viewMode === 'selection' && (
        <div style={{
          position: 'absolute', bottom: 52, left: '50%', transform: 'translateX(-50%)',
          background: 'rgba(15,23,42,0.9)', borderRadius: 8, padding: '6px 16px',
          backdropFilter: 'blur(6px)', border: '1px solid rgba(255,255,255,0.1)',
          fontSize: 12, color: '#94a3b8', whiteSpace: 'nowrap', zIndex: 10,
        }}>
          <span style={{ color: '#e2e8f0' }}>
            {data.facades.filter((f) => !disabledFacades.has(f.index)).length}
          </span>
          {' / '}{data.facades.length} facades selected
          {data.candidateFacades && data.candidateFacades.length > 0 && (
            <span> · <span style={{ color: '#4ade80' }}>{enabledCandidates.size}</span> candidates added</span>
          )}
          <span style={{ marginLeft: 12, color: '#64748b' }}>Click to include/exclude</span>
        </div>
      )}

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
