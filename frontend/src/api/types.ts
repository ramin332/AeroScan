// Types matching the FastAPI Pydantic models

export interface BuildingParams {
  lat: number;
  lon: number;
  width: number;
  depth: number;
  height: number;
  heading_deg: number;
  roof_type: 'flat' | 'pitched';
  roof_pitch_deg: number;
  num_stories: number;
}

export interface MissionParams {
  target_gsd_mm_per_px: number;
  camera: 'wide' | 'medium_tele' | 'telephoto';
  front_overlap: number;
  side_overlap: number;
  flight_speed_ms: number;
  obstacle_clearance_m: number;
  mission_name: string;
}

export interface GenerateRequest {
  preset?: string | null;
  building_id?: string | null;
  building: BuildingParams;
  mission: MissionParams;
}

export interface UploadedBuilding {
  id: string;
  name: string;
  source_type: string;
  lat: number;
  lon: number;
  height: number;
  num_stories: number;
  roof_type: string;
  roof_pitch_deg: number;
  heading_deg: number;
  properties: Record<string, unknown>;
  created_at: string;
}

export interface BuildingUploadRequest {
  name: string;
  geojson: Record<string, unknown>;
  height: number;
  num_stories: number;
  roof_type: 'flat' | 'pitched';
  roof_pitch_deg: number;
}

export interface FacadeTransition {
  from_facade: number;
  to_facade: number;
  heading_change_deg: number;
}

export interface Summary {
  waypoint_count: number;
  inspection_waypoints: number;
  transition_waypoints: number;
  photo_count: number;
  facade_count: number;
  camera_distance_m: number;
  photo_footprint_m: [number, number];
  total_path_m: number;
  estimated_flight_time_s: number;
  transitions: FacadeTransition[];
  facade_waypoint_counts: Record<number, number>;
  camera: {
    name: string;
    fov_h_deg: number;
    fov_v_deg: number;
    distance_m: number;
    focal_length_mm: number;
  };
}

// Three.js viewer data
export interface FacadeData {
  vertices: number[][];
  normal: number[];
  label: string;
  index: number;
  component: string;
  color: string;
}

export interface WaypointData {
  x: number;
  y: number;
  z: number;
  heading: number;
  gimbal_pitch: number;
  facade_index: number;
  index: number;
  component: string;
  is_transition: boolean;
}

export interface ThreeJSData {
  facades: FacadeData[];
  waypoints: WaypointData[];
  buildingLabel: string;
  buildingDims: string;
  buildingHeight: number;
}

// Leaflet viewer data
export interface LeafletWaypoint {
  index: number;
  lat: number;
  lon: number;
  alt: number;
  heading: number;
  gimbal_pitch: number;
  facade_index: number;
  component: string;
}

export interface FacadeMeta {
  label: string;
  color: string;
  azimuth: number;
  component: string;
}

export interface LeafletData {
  facadeGroups: Record<string, LeafletWaypoint[]>;
  facadeMeta: Record<string, FacadeMeta>;
  flightPath: [number, number][];
  buildingPoly: [number, number][];
  center: [number, number];
  buildingLabel: string;
  buildingDims: string;
  waypointCount: number;
  facadeCount: number;
}

export interface ViewerData {
  threejs: ThreeJSData;
  leaflet: LeafletData;
}

export interface GenerateResponse {
  version_id: string;
  timestamp: string;
  summary: Summary;
  viewer_data: ViewerData;
  config_snapshot: {
    building: BuildingParams;
    mission: MissionParams;
  };
}

export interface VersionSummary {
  version_id: string;
  timestamp: string;
  mission_name: string;
  waypoint_count: number;
  config_snapshot: {
    building: BuildingParams;
    mission: MissionParams;
  };
}

export interface PresetsResponse {
  presets: Record<string, Partial<BuildingParams>>;
}

export interface CameraSpec {
  focal_length_mm: number;
  sensor_width_mm: number;
  sensor_height_mm: number;
  image_width_px: number;
  image_height_px: number;
  fov_deg: number;
  min_interval_s: number;
}

export interface DroneSpec {
  name: string;
  cameras: Record<string, CameraSpec>;
  gimbal: {
    tilt_min_deg: number;
    tilt_max_deg: number;
    pan_min_deg: number;
    pan_max_deg: number;
  };
  flight: {
    max_speed_ms: number;
    inspection_speed_ms: number;
    min_altitude_m: number;
    max_waypoints: number;
    max_flight_time_manifold_min: number;
  };
}
