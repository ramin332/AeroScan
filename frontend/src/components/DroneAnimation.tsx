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
  const gimbalGroupRef = useRef<THREE.Group>(null);
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

    // Position
    const pos = new THREE.Vector3().lerpVectors(positions[idx], positions[idx + 1], t);
    groupRef.current.position.copy(pos);

    // Heading (rotate whole group)
    const wp = waypoints[Math.min(idx + 1, waypoints.length - 1)];
    groupRef.current.rotation.y = -(wp.heading * Math.PI) / 180;

    // Gimbal pitch
    if (gimbalGroupRef.current) {
      gimbalGroupRef.current.rotation.x = (wp.gimbal_pitch * Math.PI) / 180;
    }

    // Pulse the beam opacity for photo waypoints
    if (beamRef.current) {
      const mat = beamRef.current.material as THREE.MeshStandardMaterial;
      const isPhoto = t > 0.85 && !wp.is_transition;
      mat.opacity = isPhoto ? 0.5 : 0.15;
      mat.emissiveIntensity = isPhoto ? 1.0 : 0.2;
    }
  });

  if (positions.length < 2) return null;

  return (
    <group ref={groupRef}>
      {/* Drone: small white dot with ring */}
      <mesh>
        <sphereGeometry args={[0.3, 12, 8]} />
        <meshStandardMaterial color="#fff" emissive="#fff" emissiveIntensity={0.5} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.5, 0.6, 16]} />
        <meshBasicMaterial color="#fff" transparent opacity={0.3} side={THREE.DoubleSide} />
      </mesh>

      {/* Heading indicator: small forward line */}
      <Line
        points={[[0, 0, 0], [0, 0, -1.2]]}
        color="#fff"
        lineWidth={1.5}
        opacity={0.5}
        transparent
      />

      {/* Gimbal + camera beam */}
      <group ref={gimbalGroupRef}>
        {/* Camera lens dot */}
        <mesh position={[0, -0.15, 0]}>
          <sphereGeometry args={[0.1, 8, 6]} />
          <meshStandardMaterial color="#f59e0b" emissive="#f59e0b" emissiveIntensity={0.8} />
        </mesh>

        {/* Camera beam / projection cone showing where camera is looking */}
        <mesh ref={beamRef} position={[0, -3, 0]}>
          <cylinderGeometry args={[0.05, 1.5, 5, 8, 1, true]} />
          <meshStandardMaterial
            color="#f59e0b"
            emissive="#f59e0b"
            emissiveIntensity={0.2}
            transparent
            opacity={0.15}
            side={THREE.DoubleSide}
          />
        </mesh>

        {/* Camera center line */}
        <Line
          points={[[0, -0.15, 0], [0, -5.5, 0]]}
          color="#f59e0b"
          lineWidth={1}
          opacity={0.6}
          transparent
        />
      </group>
    </group>
  );
}

export function getVisitedIndex(progress: number, total: number): number {
  return Math.floor(progress * (total - 1));
}
