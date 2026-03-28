import { useRef, useMemo, useState, useCallback, useEffect } from 'react';
import { Canvas, useThree, type ThreeEvent } from '@react-three/fiber';
import { OrbitControls, Line } from '@react-three/drei';
import * as THREE from 'three';
import type { RawMeshData, ThreeJSData, WaypointData } from '../api/types';
import { DroneMarker, getVisitedIndex } from './DroneAnimation';

// Convert ENU (x=East, y=North, z=Up) to Three.js (x=right, y=up, z=toward camera)
function enu(x: number, y: number, z: number): [number, number, number] {
  return [x, z, -y];
}

function FacadeMesh({ vertices, color }: { vertices: number[][]; color: string }) {
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

  return (
    <group>
      <mesh geometry={geometry}>
        <meshStandardMaterial color={color} transparent opacity={0.35} side={THREE.DoubleSide} roughness={0.8} />
      </mesh>
      <Line points={edgePoints} color={color} lineWidth={1.5} />
    </group>
  );
}

function WaypointSpheres({ waypoints, color }: { waypoints: WaypointData[]; color: string }) {
  const meshRef = useRef<THREE.InstancedMesh>(null);

  useMemo(() => {
    if (!meshRef.current) return;
    const dummy = new THREE.Object3D();
    waypoints.forEach((wp, i) => {
      dummy.position.set(...enu(wp.x, wp.y, wp.z));
      dummy.updateMatrix();
      meshRef.current!.setMatrixAt(i, dummy.matrix);
    });
    meshRef.current.instanceMatrix.needsUpdate = true;
  }, [waypoints]);

  return (
    <instancedMesh ref={meshRef} args={[undefined, undefined, waypoints.length]}>
      <sphereGeometry args={[0.25, 8, 6]} />
      <meshStandardMaterial color={color} roughness={0.4} metalness={0.1} />
    </instancedMesh>
  );
}

function CameraArrows({ waypoints, color }: { waypoints: WaypointData[]; color: string }) {
  const lines = useMemo(() => {
    return waypoints.map((wp) => {
      const origin = enu(wp.x, wp.y, wp.z);
      const hr = (wp.heading * Math.PI) / 180;
      const pr = (wp.gimbal_pitch * Math.PI) / 180;
      const dx = Math.sin(hr) * Math.cos(pr);
      const dy = Math.cos(hr) * Math.cos(pr);
      const dz = Math.sin(pr);
      const end = enu(wp.x + dx * 2, wp.y + dy * 2, wp.z + dz * 2);
      return [origin, end] as [[number, number, number], [number, number, number]];
    });
  }, [waypoints]);

  return (
    <>
      {lines.map((pts, i) => (
        <Line key={i} points={pts} color={color} lineWidth={1} opacity={0.5} transparent />
      ))}
    </>
  );
}

function FlightPath({ waypoints }: { waypoints: WaypointData[] }) {
  const points = useMemo(
    () => waypoints.map((wp) => enu(wp.x, wp.y, wp.z) as [number, number, number]),
    [waypoints],
  );
  if (points.length < 2) return null;
  return <Line points={points} color="white" lineWidth={0.5} opacity={0.3} transparent />;
}

function RawMeshView({ mesh }: { mesh: RawMeshData }) {
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

  return (
    <group>
      <mesh geometry={geometry}>
        <meshStandardMaterial
          color="#8899bb"
          transparent
          opacity={0.25}
          side={THREE.DoubleSide}
          roughness={0.7}
          metalness={0.1}
        />
      </mesh>
      <mesh geometry={geometry}>
        <meshStandardMaterial
          color="#aabbdd"
          wireframe
          transparent
          opacity={0.15}
        />
      </mesh>
    </group>
  );
}

function VisitedOverlay({ waypoints, visitedUpTo }: { waypoints: WaypointData[]; visitedUpTo: number }) {
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const count = Math.min(visitedUpTo + 1, waypoints.length);

  useMemo(() => {
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
    <instancedMesh ref={meshRef} args={[undefined, undefined, waypoints.length]}>
      <sphereGeometry args={[0.3, 6, 4]} />
      <meshStandardMaterial color="#888" transparent opacity={0.6} />
    </instancedMesh>
  );
}

function Scene({ data, onWaypointClick, visitedIndex, showRawMesh }: { data: ThreeJSData; onWaypointClick: (wp: WaypointData | null) => void; visitedIndex: number; showRawMesh: boolean }) {
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
      {data.facades.map((f) => (
        <FacadeMesh key={f.index} vertices={f.vertices} color={f.color} />
      ))}

      {/* Inspection waypoints + arrows per facade */}
      {Object.entries(facadeWaypoints).map(([fi, wps]) => {
        const facade = data.facades.find((f) => f.index === parseInt(fi));
        const color = facade?.color || '#888';
        return (
          <group key={fi}>
            <WaypointSpheres waypoints={wps} color={color} />
            <CameraArrows waypoints={wps} color={color} />
          </group>
        );
      })}

      {/* Transition waypoints (smaller, grey) */}
      {transitionWaypoints.length > 0 && (
        <WaypointSpheres waypoints={transitionWaypoints} color="#555" />
      )}

      {/* Flight path */}
      <FlightPath waypoints={data.waypoints} />

      {/* Grey overlay on visited waypoints during playback */}
      {visitedIndex >= 0 && (
        <VisitedOverlay waypoints={data.waypoints} visitedUpTo={visitedIndex} />
      )}

      {/* Raw 3D mesh from uploaded file */}
      {showRawMesh && data.rawMesh && (
        <RawMeshView mesh={data.rawMesh} />
      )}
    </group>
  );
}

interface CameraFOV {
  fov_h_deg: number;
  fov_v_deg: number;
  distance_m: number;
}

export function Viewer3D({ data, cameraFov }: { data: ThreeJSData | null; cameraFov?: CameraFOV }) {
  const [selectedWp, setSelectedWp] = useState<WaypointData | null>(null);
  const [playing, setPlaying] = useState(false);
  const [progress, setProgress] = useState(0);
  const [speed, setSpeed] = useState(1);
  const [showRawMesh, setShowRawMesh] = useState(true);
  const animRef = useRef<number | null>(null);
  const lastTimeRef = useRef(0);

  // Reset playback when data changes
  useEffect(() => {
    setPlaying(false);
    setProgress(0);
  }, [data]);

  // Animation loop
  useEffect(() => {
    if (!playing || !data) return;

    // Base duration: 30s for the whole mission at 1x speed
    const duration = (30 / speed) * 1000;

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

  if (!data) {
    return <div className="empty-state">Click "Generate Mission" to start</div>;
  }

  // Current waypoint index for display
  const currentWpIdx = Math.min(
    Math.floor(progress * (data.waypoints.length - 1)),
    data.waypoints.length - 1,
  );
  const currentWp = data.waypoints[currentWpIdx];

  // Group counts for legend
  const facadeCounts: Record<number, number> = {};
  data.waypoints.forEach((wp) => {
    if (!wp.is_transition) facadeCounts[wp.facade_index] = (facadeCounts[wp.facade_index] || 0) + 1;
  });

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      <Canvas
        camera={{ position: [35, 25, 35], fov: 50, near: 0.1, far: 500 }}
        style={{ background: '#0f1117' }}
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
        <Scene data={data} onWaypointClick={setSelectedWp} visitedIndex={playing || progress > 0 ? getVisitedIndex(progress, data.waypoints.length) : -1} showRawMesh={showRawMesh} />
        {(playing || progress > 0) && (
          <DroneMarker waypoints={data.waypoints} progress={progress} cameraFov={cameraFov} />
        )}
      </Canvas>

      {/* Legend */}
      <div className="legend-3d">
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
        {data.facades.map((f) => (
          <div key={f.index} className="legend-item">
            <span className="legend-dot" style={{ backgroundColor: f.color }} />
            <span>{f.label} ({facadeCounts[f.index] || 0})</span>
          </div>
        ))}
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
