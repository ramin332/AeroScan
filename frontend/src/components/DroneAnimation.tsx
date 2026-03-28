import { useRef, useMemo, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import type { WaypointData } from '../api/types';

/**
 * Coordinate conventions:
 *
 * ENU (our geometry): x=East, y=North, z=Up
 * Three.js (OpenGL):  x=Right, y=Up, z=Backward
 * DJI body (NED):     x=Forward, y=Right, z=Down
 *
 * ENU → Three.js: (x, y, z) → (x, z, -y)
 *
 * DJI heading: degrees clockwise from North (0°=N, 90°=E, 180°=S)
 * DJI gimbal pitch: 0°=horizontal (forward), -90°=nadir (down), +35°=up
 *
 * Rotation chain (world to camera):
 *   1. Heading (yaw): rotate around world Up axis
 *   2. Gimbal pitch: rotate around body Right axis (after heading applied)
 */

function toScene(x: number, y: number, z: number): THREE.Vector3 {
  return new THREE.Vector3(x, z, -y);
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

interface Props {
  waypoints: WaypointData[];
  progress: number;
  cameraFov?: CameraFOV;
  timeline?: Timeline;
}

/** Layer 1 is used to hide the drone marker from the PIP camera preview. */
export const DRONE_LAYER = 1;

export function DroneMarker({ waypoints, progress, cameraFov, timeline }: Props) {
  const groupRef = useRef<THREE.Group>(null);
  const gimbalRef = useRef<THREE.Group>(null);
  const footprintRef = useRef<THREE.Mesh>(null);

  // Put all drone visuals on layer 1 so the PIP camera (layer 0 only) doesn't see them
  useEffect(() => {
    if (!groupRef.current) return;
    groupRef.current.traverse((obj) => {
      obj.layers.set(DRONE_LAYER);
    });
  });

  const positions = useMemo(
    () => waypoints.map((wp) => toScene(wp.x, wp.y, wp.z)),
    [waypoints],
  );

  // Camera frustum geometry from real specs
  const { beamLength, halfW, halfH } = useMemo(() => {
    if (!cameraFov) return { beamLength: 5, halfW: 1.2, halfH: 0.9 };
    const d = cameraFov.distance_m;
    return {
      beamLength: d,
      halfW: Math.tan((cameraFov.fov_h_deg / 2) * Math.PI / 180) * d,
      halfH: Math.tan((cameraFov.fov_v_deg / 2) * Math.PI / 180) * d,
    };
  }, [cameraFov]);

  // Frustum wireframe
  const frustumGeo = useMemo(() => {
    const geo = new THREE.BufferGeometry();
    const d = beamLength;
    const positions = new Float32Array([
      0,0,0, -halfW,-halfH,-d,   0,0,0, halfW,-halfH,-d,
      0,0,0, halfW,halfH,-d,     0,0,0, -halfW,halfH,-d,
      -halfW,-halfH,-d, halfW,-halfH,-d,
      halfW,-halfH,-d, halfW,halfH,-d,
      halfW,halfH,-d, -halfW,halfH,-d,
      -halfW,halfH,-d, -halfW,-halfH,-d,
    ]);
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    return geo;
  }, [beamLength, halfW, halfH]);

  // Footprint quad at far end
  const footprintGeo = useMemo(() => {
    const geo = new THREE.BufferGeometry();
    const d = beamLength;
    const v = new Float32Array([
      -halfW,-halfH,-d, halfW,-halfH,-d, halfW,halfH,-d,
      -halfW,-halfH,-d, halfW,halfH,-d, -halfW,halfH,-d,
    ]);
    geo.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
    geo.computeVertexNormals();
    return geo;
  }, [beamLength, halfW, halfH]);

  useFrame(() => {
    if (!groupRef.current || positions.length < 2) return;

    // Map progress (0-1) to the physics-based timeline
    let idx: number, t: number;
    if (timeline && timeline.cumTimes.length === positions.length) {
      const currentTime = progress * timeline.totalTime;
      // Find segment: binary search in cumTimes
      idx = 0;
      for (let i = 1; i < timeline.cumTimes.length; i++) {
        if (timeline.cumTimes[i] >= currentTime) break;
        idx = i;
      }
      idx = Math.min(idx, positions.length - 2);
      const segStart = timeline.cumTimes[idx];
      const segEnd = timeline.cumTimes[idx + 1];
      const segDuration = segEnd - segStart;
      t = segDuration > 0 ? Math.min((currentTime - segStart) / segDuration, 1) : 0;
    } else {
      // Fallback: uniform interpolation
      const n = positions.length - 1;
      const rawIdx = progress * n;
      idx = Math.min(Math.floor(rawIdx), n - 1);
      t = rawIdx - idx;
    }

    // Smooth ease for transit, linear for inspection (stop-and-shoot)
    const wp = waypoints[Math.min(idx + 1, waypoints.length - 1)];
    const eased = wp.is_transition
      ? t * t * (3 - 2 * t)  // smoothstep ease-in-out for transit
      : t;                    // linear for inspection (physically: constant speed)

    const pos = new THREE.Vector3().lerpVectors(positions[idx], positions[idx + 1], eased);
    groupRef.current.position.copy(pos);

    // Heading rotation around Y (up in Three.js)
    // DJI heading: 0°=North, CW positive. In Three.js, North = -Z.
    // Heading 0° → drone faces -Z → rotation Y = 0
    // Heading 90° → drone faces +X → rotation Y = -π/2
    groupRef.current.rotation.set(0, -(wp.heading * Math.PI) / 180, 0);

    // Gimbal pitch around drone's local X (right) axis.
    // DJI: 0° = forward (-Z in Three.js), -90° = nadir (-Y in Three.js)
    // Three.js right-hand rule around +X: positive rotates -Z toward +Y (up).
    // So pitch -90° needs rotation.x = -π/2 to point -Z → -Y (down).
    // Therefore: rotation.x = pitch_in_radians (no negation).
    if (gimbalRef.current) {
      gimbalRef.current.rotation.set((wp.gimbal_pitch * Math.PI) / 180, 0, 0);
    }

    // Photo flash
    if (footprintRef.current) {
      const mat = footprintRef.current.material as THREE.MeshStandardMaterial;
      const isPhoto = t > 0.85 && !wp.is_transition;
      mat.opacity = isPhoto ? 0.25 : 0.06;
    }
  });

  if (positions.length < 2) return null;

  return (
    <group ref={groupRef}>
      {/* Drone dot */}
      <mesh>
        <sphereGeometry args={[0.2, 12, 8]} />
        <meshStandardMaterial color="#fff" emissive="#fff" emissiveIntensity={0.6} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.35, 0.42, 16]} />
        <meshBasicMaterial color="#fff" transparent opacity={0.2} side={THREE.DoubleSide} />
      </mesh>

      {/* Heading: line along -Z (forward in Three.js local) */}
      <mesh position={[0, 0, -0.6]}>
        <boxGeometry args={[0.04, 0.04, 0.6]} />
        <meshBasicMaterial color="#fff" transparent opacity={0.35} />
      </mesh>

      {/* Gimbal group: default looks along -Z (forward), pitch rotates around X */}
      <group ref={gimbalRef}>
        {/* Camera lens */}
        <mesh position={[0, 0, -0.2]}>
          <sphereGeometry args={[0.06, 8, 6]} />
          <meshStandardMaterial color="#f59e0b" emissive="#f59e0b" emissiveIntensity={1.0} />
        </mesh>

        {/* Frustum wireframe */}
        <lineSegments geometry={frustumGeo}>
          <lineBasicMaterial color="#f59e0b" transparent opacity={0.35} />
        </lineSegments>

        {/* Footprint fill */}
        <mesh ref={footprintRef} geometry={footprintGeo}>
          <meshStandardMaterial
            color="#f59e0b"
            emissive="#f59e0b"
            emissiveIntensity={0.1}
            transparent
            opacity={0.06}
            side={THREE.DoubleSide}
          />
        </mesh>
      </group>
    </group>
  );
}

export function getVisitedIndex(progress: number, total: number): number {
  return Math.floor(progress * (total - 1));
}
