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
  gimbal_pitch_margin_deg: number;
  min_photo_distance_m: number;
  yaw_rate_deg_per_s: number;
  stop_at_waypoint: boolean;
}

export interface ValidationIssue {
  severity: 'error' | 'warning' | 'info';
  code: string;
  message: string;
  waypoint_indices: number[];
  facade_index: number | null;
}

export interface AlgorithmParams {
  // Flight time estimation
  hover_time_per_wp_s: number;
  takeoff_landing_overhead_s: number;
  battery_warning_threshold: number;
  battery_info_threshold: number;
  gimbal_near_limit_deg: number;
  // Geometry / grid generation
  facade_edge_inset_m: number;
  transition_altitude_margin_m: number;
  roof_normal_threshold: number;
  min_altitude_m: number;
  // Mesh import
  default_building_height_m: number;
  min_mesh_faces: number;
  downward_face_threshold: number;
  ground_level_threshold_m: number;
  occlusion_ray_offset_m: number;
  occlusion_hit_fraction: number;
  flat_roof_normal_threshold: number;
  wall_normal_threshold: number;
  auto_scale_height_threshold_m: number;
  auto_scale_target_height_m: number;
  region_growing_angle_deg: number;
  // Surface sampling
  surface_sample_count: number;
  surface_dedup_radius_m: number;
  surface_dedup_max_angle_deg: number;
  // Waypoint LOS occlusion
  enable_waypoint_los: boolean;
  los_tolerance_m: number;
  los_min_visible_ratio: number;
  // Path collision checking
  enable_path_collision_check: boolean;
  path_collision_margin_m: number;
  // Path optimization
  grid_density: number;
  enable_path_dedup: boolean;
  enable_path_tsp: boolean;
  enable_sweep_reversal: boolean;
  dedup_max_gimbal_diff_deg: number;
  tsp_method: 'auto' | 'nearest_neighbor' | 'greedy' | 'simulated_annealing' | 'threshold_accepting';
  // KMZ export
  min_waypoint_height_m: number;
}

export interface ExclusionZone {
  id: string;
  label: string;
  center_x: number;
  center_y: number;
  center_z: number;
  size_x: number;
  size_y: number;
  size_z: number;
  zone_type: 'no_fly' | 'no_inspect' | 'inclusion';
  /** ENU [x, y] vertex pairs for polygon zones. If absent, zone is an axis-aligned box. */
  polygon_vertices?: [number, number][];
}

export interface GenerateRequest {
  preset?: string | null;
  building_id?: string | null;
  building: BuildingParams;
  mission: MissionParams;
  algorithm: AlgorithmParams;
  min_facade_area?: number;
  extraction_method?: string;
  waypoint_strategy?: string;
  disabled_facades?: number[];
  enabled_candidates?: number[];
  exclusion_zones?: ExclusionZone[];
}

export interface UploadedBuilding {
  id: string;
  name: string;
  source_type: string;
  lat: number;
  lon: number;
  height: number;
  width: number;
  depth: number;
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
  source?: string;
  parent_version_id?: string;
  gimbal_before?: GimbalStats;
  gimbal_after?: GimbalStats;
  gimbal_diff?: GimbalDiffEntry[];
  facade_coverage?: FacadeCoverageEntry[];
}

export interface FacadeCoverageEntry {
  facade_index: number;
  label: string;
  area_m2: number;
  waypoint_count: number;
  mean_pitch_abs_deg: number | null;
  mean_perpendicularity: number | null;
  mean_distance_m: number | null;
}

export interface GimbalStats {
  count: number;
  pitch_mean: number;
  pitch_min: number;
  pitch_max: number;
  pitch_median: number;
  yaw_unique: number;
}

export interface GimbalDiffEntry {
  index: number;
  pitch_before: number;
  pitch_after: number;
  yaw_before: number;
  yaw_after: number;
  facade_index: number;
}

// Three.js viewer data
export interface FacadeData {
  vertices: number[][];
  normal: number[];
  label: string;
  index: number;
  component: string;
  color: string;
  direction: string;
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
  pitch_before?: number;
  yaw_before?: number;
  smart_oblique_poses?: { pitch: number; yaw: number }[];
}

export interface RawMeshData {
  // Legacy shape (kept for small meshes): flat JS-number arrays.
  positions?: number[]; // flat [x0,y0,z0, x1,y1,z1, ...]
  indices?: number[];   // flat [i0,j0,k0, i1,j1,k1, ...]
  // Fast path: gzip+base64 of Float32Array (xyz) / Int32Array (triangles).
  positions_b64?: string;
  indices_b64?: string;
  n_vertices?: number;
  n_faces?: number;
}

export interface PointCloudData {
  positions: number[]; // flat [x0,y0,z0, x1,y1,z1, ...] in ENU meters
  colors: number[];    // flat [r0,g0,b0, ...] in 0..1
}

export interface MissionAreaData {
  vertices: number[][]; // [[x,y,z], ...] in ENU meters
}

export interface ThreeJSData {
  facades: FacadeData[];
  candidateFacades?: FacadeData[];
  waypoints: WaypointData[];
  buildingLabel: string;
  buildingDims: string;
  buildingHeight: number;
  rawMesh?: RawMeshData | null;
  pointCloud?: PointCloudData;
  missionArea?: MissionAreaData;
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
  missionAreaPoly?: [number, number][]; // [lat, lon] pairs, DJI Smart3D import only
}

export interface ViewerData {
  threejs: ThreeJSData;
  leaflet: LeafletData;
}

export interface PerfStats {
  total_ms: number;
  building_ms: number;
  waypoints_ms: number;
  summary_ms: number;
  validate_ms: number;
  generation: {
    facades_total: number;
    facades_with_waypoints: number;
    waypoints_before_dedup: number;
    waypoints_after_dedup: number;
    waypoints_deduped: number;
    per_facade: { facade_index: number; label: string; waypoints: number; before_dedup: number }[];
    optimization: {
      waypoints_merged: number;
      facade_order: number[];
      facades_reversed: number[];
      transit_distance_before_m: number;
      transit_distance_after_m: number;
      transit_saved_m: number;
      two_opt_improvements: number;
    };
  };
  extraction: {
    method: string;
    input_faces: number;
    regions_found: number;
    filtered_by_area: number;
    filtered_by_normal: number;
    filtered_by_ground: number;
    filtered_by_occlusion: number;
    facades_extracted: number;
    walls: number;
    roofs: number;
  } | null;
  validation_counts: {
    errors: number;
    warnings: number;
    info: number;
  };
}

export interface BenchmarkResult {
  method: string;
  time_ms: number;
  waypoints: number;
  transit_before_m: number;
  transit_after_m: number;
  transit_saved_m: number;
  facades_reversed: number;
  merged: number;
}

export interface GenerateResponse {
  version_id: string;
  timestamp: string;
  summary: Summary;
  viewer_data: ViewerData;
  validation: ValidationIssue[];
  can_export: boolean;
  perf?: PerfStats;
  config_snapshot: {
    building: BuildingParams;
    mission: MissionParams;
    algorithm?: AlgorithmParams;
    disabled_facades?: number[];
    enabled_candidates?: number[];
    exclusion_zones?: ExclusionZone[];
    building_id?: string;
  };
  building_id?: string;
  imported_name?: string;
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

// Simulation / Reconstruction types

export interface SimulationComparison {
  original: {
    facade_count: number;
    dimensions: [number, number, number];
    inspection_waypoints: number;
  };
  reconstructed: {
    facade_count: number;
    dimensions: [number, number, number];
    inspection_waypoints: number;
  };
  diff: {
    facade_count: number;
    width_m: number;
    depth_m: number;
    height_m: number;
    waypoint_diff: number;
  };
  method: string;
  render_scale: number;
  voxel_size_m: number;
  num_photos: number;
}

export interface SimulationPhoto {
  index: number;
  path: string;
  facade_index: number;
  position: [number, number, number];
  heading: number;
  gimbal_pitch: number;
}

export interface SimulationResult {
  viewer_data: ViewerData;
  summary: {
    waypoint_count: number;
    inspection_waypoints: number;
    facade_count: number;
    building_dims: string;
  };
  comparison: SimulationComparison;
  photos: SimulationPhoto[];
  photos_total: number;
  output_dir: string;
  mesh_path: string;
}

export interface SimulationStatus {
  task_id: string;
  status: 'pending' | 'rendering' | 'reconstructing' | 'importing' | 'generating' | 'complete' | 'error';
  progress: number;
  message: string;
  result?: SimulationResult;
  error?: string;
}
