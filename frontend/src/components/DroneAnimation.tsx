import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Line } from '@react-three/drei';
import * as THREE from 'three';
import type { WaypointData } from '../api/types';

function toScene(x: number, y: number, z: number): THREE.Vector3 {
  return new THREE.Vector3(x, z, -y);
}

interface Props {
  waypoints: WaypointData[];
  progress: number; // 0..1
}

export function DroneMarker({ waypoints, progress }: Props) {
  const groupRef = useRef<THREE.Group>(null);
  const gimbalRef = useRef<THREE.Group>(null);
  const beamRef = useRef<THREE.Mesh>(null);

  const positions = useMemo(
    () => waypoints.map((wp) => toScene(wp.x, wp.y, wp.z)),
    [waypoints],
  );

  useFrame(() => {
    if (!groupRef.current || positions.length < 2) return;

    const n = positions.length - 1;
    const rawIdx = progress * n;
    const idx = Math.min(Math.floor(rawIdx), n - 1);
    const t = rawIdx - idx;

    // Position: linear interp
    const pos = new THREE.Vector3().lerpVectors(positions[idx], positions[idx + 1], t);
    groupRef.current.position.copy(pos);

    // Heading: rotate whole drone around Y
    const wp = waypoints[Math.min(idx + 1, waypoints.length - 1)];
    groupRef.current.rotation.y = -(wp.heading * Math.PI) / 180;

    // Gimbal pitch:
    //   DJI convention: 0° = horizontal (camera looks forward), -90° = nadir (straight down)
    //   Three.js: gimbal group default points along -Z (forward in drone local)
    //   We rotate around X: pitch 0° → look forward (no rotation), pitch -90° → look down (+90° on X)
    if (gimbalRef.current) {
      gimbalRef.current.rotation.x = -(wp.gimbal_pitch * Math.PI) / 180;
    }

    // Flash beam on photo
    if (beamRef.current) {
      const mat = beamRef.current.material as THREE.MeshStandardMaterial;
      const isPhoto = t > 0.85 && !wp.is_transition;
      mat.opacity = isPhoto ? 0.4 : 0.12;
      mat.emissiveIntensity = isPhoto ? 0.8 : 0.1;
    }
  });

  if (positions.length < 2) return null;

  return (
    <group ref={groupRef}>
      {/* Drone marker: white dot + ring */}
      <mesh>
        <sphereGeometry args={[0.25, 12, 8]} />
        <meshStandardMaterial color="#fff" emissive="#fff" emissiveIntensity={0.6} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.45, 0.55, 16]} />
        <meshBasicMaterial color="#fff" transparent opacity={0.25} side={THREE.DoubleSide} />
      </mesh>

      {/* Heading line: thin white line pointing forward (-Z in local = north when heading=0) */}
      <Line
        points={[[0, 0, 0], [0, 0, -1.0]]}
        color="#fff"
        lineWidth={1.5}
        opacity={0.4}
        transparent
      />

      {/* Gimbal group: rotates around X axis based on pitch.
          Default orientation: beam points along -Z (forward).
          pitch=0 → forward, pitch=-90 → down (+90° rotation around X) */}
      <group ref={gimbalRef}>
        {/* Camera lens */}
        <mesh position={[0, 0, -0.3]}>
          <sphereGeometry args={[0.08, 8, 6]} />
          <meshStandardMaterial color="#f59e0b" emissive="#f59e0b" emissiveIntensity={0.8} />
        </mesh>

        {/* Camera beam: cone projecting along -Z (forward in gimbal space) */}
        <mesh ref={beamRef} position={[0, 0, -3]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.05, 1.2, 5, 8, 1, true]} />
          <meshStandardMaterial
            color="#f59e0b"
            emissive="#f59e0b"
            emissiveIntensity={0.1}
            transparent
            opacity={0.12}
            side={THREE.DoubleSide}
          />
        </mesh>

        {/* Camera center line along -Z */}
        <Line
          points={[[0, 0, -0.3], [0, 0, -5.5]]}
          color="#f59e0b"
          lineWidth={1}
          opacity={0.5}
          transparent
        />
      </group>
    </group>
  );
}

export function getVisitedIndex(progress: number, total: number): number {
  return Math.floor(progress * (total - 1));
}
