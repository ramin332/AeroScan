import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { AlgorithmParams, BuildingParams, ExclusionZone, GenerateResponse, MissionParams, SimulationResult, UploadedBuilding, VersionSummary } from './api/types';
import * as api from './api/client';

export const DEFAULT_BUILDING: BuildingParams = {
  lat: 53.2012,
  lon: 5.7999,
  width: 20,
  depth: 10,
  height: 8,
  heading_deg: 0,
  roof_type: 'flat',
  roof_pitch_deg: 0,
  num_stories: 1,
};

export const DEFAULT_MISSION: MissionParams = {
  target_gsd_mm_per_px: 2.0,
  camera: 'wide',
  front_overlap: 0.80,
  side_overlap: 0.70,
  flight_speed_ms: 2.0,
  obstacle_clearance_m: 2.0,
  mission_name: 'AeroScan Inspection',
  gimbal_pitch_margin_deg: 5.0,
  min_photo_distance_m: 1.5,
  yaw_rate_deg_per_s: 60.0,
  stop_at_waypoint: false,
};

// NEN-2767 inspection preset: stop-and-shoot, perpendicular gimbal, moderate
// speed, tight overlap. Applied when regenerating a clean inspection mission
// from an imported DJI reconnaissance scan.
export const NEN2767_MISSION: MissionParams = {
  target_gsd_mm_per_px: 2.0,
  camera: 'wide',
  front_overlap: 0.80,
  side_overlap: 0.70,
  flight_speed_ms: 3.0,
  obstacle_clearance_m: 2.0,
  mission_name: 'NEN-2767 Inspection',
  gimbal_pitch_margin_deg: 5.0,
  min_photo_distance_m: 4.0,
  yaw_rate_deg_per_s: 60.0,
  stop_at_waypoint: true,
};

export const DEFAULT_ALGORITHM: AlgorithmParams = {
  // Flight time estimation
  hover_time_per_wp_s: 1.0,
  takeoff_landing_overhead_s: 60.0,
  battery_warning_threshold: 0.80,
  battery_info_threshold: 0.65,
  gimbal_near_limit_deg: -80.0,
  // Geometry / grid generation
  facade_edge_inset_m: 0.1,
  transition_altitude_margin_m: 2.0,
  roof_normal_threshold: 0.5,
  min_altitude_m: 2.0,
  // Mesh import
  default_building_height_m: 8.0,
  min_mesh_faces: 4,
  downward_face_threshold: -0.3,
  ground_level_threshold_m: 0.3,
  occlusion_ray_offset_m: 0.05,
  occlusion_hit_fraction: 0.5,
  flat_roof_normal_threshold: 0.95,
  wall_normal_threshold: 0.3,
  auto_scale_height_threshold_m: 50.0,
  auto_scale_target_height_m: 8.0,
  region_growing_angle_deg: 15.0,
  // Surface sampling
  surface_sample_count: 2000,
  surface_dedup_radius_m: 0.5,
  surface_dedup_max_angle_deg: 30.0,
  // Waypoint LOS occlusion
  enable_waypoint_los: true,
  los_tolerance_m: 0.5,
  los_min_visible_ratio: 0.4,
  // Path collision checking
  enable_path_collision_check: true,
  path_collision_margin_m: 0.5,
  // Path optimization
  grid_density: 1.0,
  enable_path_dedup: true,
  enable_path_tsp: true,
  enable_sweep_reversal: true,
  dedup_max_gimbal_diff_deg: 20.0,
  tsp_method: 'auto' as const,
  // KMZ export
  min_waypoint_height_m: 2.0,
};

type Tab = '3d' | 'map' | 'sim';

interface AppState {
  selectedBuildingId: string | null;

  // Params
  building: BuildingParams;
  mission: MissionParams;
  algorithm: AlgorithmParams;

  // Exclusion zones & facade toggling
  disabledFacades: Set<number>;
  enabledCandidates: Set<number>;
  exclusionZones: ExclusionZone[];
  zoneDrawMode: boolean;

  // Result
  result: GenerateResponse | null;
  versions: VersionSummary[];
  loading: boolean;
  uploading: boolean;
  uploadProgress: number;   // 0-1
  uploadMessage: string;
  lastPhaseTimings: Array<{ label: string; seconds: number }> | null;
  activeTab: Tab;

  // Actions
  setBuilding: (patch: Partial<BuildingParams>) => void;
  setMission: (patch: Partial<MissionParams>) => void;
  setAlgorithm: (patch: Partial<AlgorithmParams>) => void;
  resetAlgorithm: () => void;
  setActiveTab: (tab: Tab) => void;
  toggleFacade: (index: number) => void;
  toggleCandidate: (index: number) => void;
  addExclusionZone: (zone: ExclusionZone) => void;
  removeExclusionZone: (id: string) => void;
  updateExclusionZone: (id: string, patch: Partial<ExclusionZone>) => void;
  setZoneDrawMode: (v: boolean) => void;
  generate: () => Promise<void>;
  loadVersion: (id: string) => Promise<void>;
  deleteVersion: (id: string) => Promise<void>;
  deleteAllVersions: () => Promise<void>;
  refreshVersions: () => Promise<void>;
  rewriteGimbals: () => Promise<void>;
  generateInspectionMission: () => Promise<void>;

  // Theme
  lightMode: boolean;
  setLightMode: (v: boolean) => void;

  // Section open/close state
  sectionState: Record<string, boolean>;
  setSectionOpen: (key: string, open: boolean) => void;

  // Building upload actions
  importKmz: (file: File, voxelSize?: number | null, mode?: 'raw' | 'facades') => Promise<void>;
  kmzImportMode: 'raw' | 'facades';
  setKmzImportMode: (v: 'raw' | 'facades') => void;
  lastKmzFile: File | null;
  toggleKmzFacades: () => Promise<void>;
  refineKmz: (voxelSize: number) => Promise<void>;
  triggerRefine: () => void;
  triggerMissionUpdate: () => void;
  missionUpdateRunning: boolean;
  missionUpdatePending: boolean;
  refineRunning: boolean;
  refinePending: boolean;
  optimizeKmz: (buildingId?: string | null) => Promise<void>;
  cancelOptimize: () => void;
  kmzOptimizing: boolean;
  kmzOptimizeMessage: string;
  kmzAutoRefine: boolean;
  setKmzAutoRefine: (v: boolean) => void;
  // Reconstruction knobs (voxel + CGAL alpha_wrap_3)
  kmzOptimizeMin: number;   // finest voxel in optimize schedule (m)
  kmzOptimizeMax: number;   // coarsest voxel in optimize schedule (m)
  kmzOptimizeSteps: number; // number of passes
  kmzAwAlpha: number | null;   // override CGAL alpha (null = auto from voxel)
  kmzAwOffset: number | null;  // override CGAL offset (null = auto from voxel)
  setKmzReconParams: (p: Partial<{
    kmzOptimizeMin: number; kmzOptimizeMax: number; kmzOptimizeSteps: number;
    kmzAwAlpha: number | null; kmzAwOffset: number | null;
  }>) => void;
  // Facade detection (CGAL Shape-Detection) knobs. null = backend default.
  kmzFdEpsilon: number | null;         // plane-fit ε (m)
  kmzFdClusterEpsilon: number | null;  // max inlier gap (m)
  kmzFdMinPoints: number | null;       // min inliers per region
  kmzFdMinWallArea: number | null;     // min wall area (m²)
  kmzFdMinRoofArea: number | null;     // min roof area (m²)
  kmzFdMinDensity: number | null;      // min inlier density (pts/m²)
  kmzFdNormalThreshold: number | null; // normal-agreement (cos θ)
  setKmzFacadeParams: (p: Partial<{
    kmzFdEpsilon: number | null; kmzFdClusterEpsilon: number | null;
    kmzFdMinPoints: number | null; kmzFdMinWallArea: number | null;
    kmzFdMinRoofArea: number | null; kmzFdMinDensity: number | null;
    kmzFdNormalThreshold: number | null;
  }>) => void;
  // DJI vs Inspection mode (UI-only split; affects which sliders are shown).
  kmzMode: 'dji' | 'inspection';
  setKmzMode: (v: 'dji' | 'inspection') => void;
  // Saved buildings (persisted on backend sqlite). Displayed as "Saved buildings" list.
  buildings: import('./api/types').UploadedBuilding[];
  refreshBuildings: () => Promise<void>;
  selectBuilding: (id: string, mode?: 'dji' | 'inspection') => Promise<void>;
  deleteBuilding: (id: string) => Promise<void>;
  switchMode: (mode: 'dji' | 'inspection') => Promise<void>;
  stripRosetteOnly: () => Promise<void>;
  showOriginalGimbals: boolean;
  setShowOriginalGimbals: (v: boolean) => void;
  // When false, one frustum per waypoint (planned pose) — matches DJI's
  // Capture Quality Report. When true, all 5 rosette poses are drawn.
  showRosettePoses: boolean;
  setShowRosettePoses: (v: boolean) => void;
  showMappingBox: boolean;
  setShowMappingBox: (v: boolean) => void;
  // 'polygon' = mission-area polygon extent (matches RC Plus on-controller);
  // 'tileset' = 3D-Tiles root OBB (whole reconstructed cloud extent).
  mappingBoxSource: 'polygon' | 'tileset';
  setMappingBoxSource: (v: 'polygon' | 'tileset') => void;
  // Per-field overrides for the camera intrinsics used to draw frustums.
  // Null keys mean "use summary.camera value verbatim"; set by the sidebar to
  // let operators dial the frustum without re-importing.
  cameraFovOverride: { fov_h_deg: number | null; fov_v_deg: number | null; distance_m: number | null };
  setCameraFovOverride: (patch: Partial<{ fov_h_deg: number | null; fov_v_deg: number | null; distance_m: number | null }>) => void;
  resetCameraFovOverride: () => void;

  // Simulation / reconstruction
  simTaskId: string | null;
  simStatus: string | null;
  simProgress: number;
  simMessage: string;
  simStartTime: number | null;
  simResult: SimulationResult | null;
  simViewerData: import('./api/types').ViewerData | null;
  startSimulation: (renderScale?: number, voxelSize?: number) => Promise<void>;
  viewSimulationResult: () => void;
  deleteSimulation: () => Promise<void>;
  loadSimFromUrl: () => Promise<void>;
}

export const useStore = create<AppState>()(persist((set, get) => ({
  selectedBuildingId: null,
  disabledFacades: new Set<number>(),
  enabledCandidates: new Set<number>(),
  exclusionZones: [],
  zoneDrawMode: false,

  building: { ...DEFAULT_BUILDING },
  mission: { ...DEFAULT_MISSION },
  algorithm: { ...DEFAULT_ALGORITHM },
  result: null,
  versions: [],
  loading: false,
  uploading: false,
  uploadProgress: 0,
  uploadMessage: '',
  lastPhaseTimings: null,
  activeTab: '3d',
  lightMode: false,
  setLightMode: (v) => set({ lightMode: v }),
  sectionState: {},
  setSectionOpen: (key, open) => set((s) => ({ sectionState: { ...s.sectionState, [key]: open } })),

  setBuilding: (patch) =>
    set((s) => ({ building: { ...s.building, ...patch } })),

  setMission: (patch) =>
    set((s) => ({ mission: { ...s.mission, ...patch } })),

  setAlgorithm: (patch) =>
    set((s) => ({ algorithm: { ...s.algorithm, ...patch } })),

  resetAlgorithm: () => set({ algorithm: { ...DEFAULT_ALGORITHM } }),

  setActiveTab: (tab) => set({ activeTab: tab }),

  toggleFacade: (index) => {
    const next = new Set(get().disabledFacades);
    if (next.has(index)) next.delete(index);
    else next.add(index);
    set({ disabledFacades: next });
    get().generate();
  },
  toggleCandidate: (index) => {
    const next = new Set(get().enabledCandidates);
    if (next.has(index)) next.delete(index);
    else next.add(index);
    set({ enabledCandidates: next });
    get().generate();
  },
  addExclusionZone: (zone) => set({ exclusionZones: [...get().exclusionZones, zone] }),
  removeExclusionZone: (id) => set({ exclusionZones: get().exclusionZones.filter(z => z.id !== id) }),
  updateExclusionZone: (id, patch) => set({
    exclusionZones: get().exclusionZones.map(z => z.id === id ? { ...z, ...patch } : z),
  }),
  setZoneDrawMode: (v) => set({ zoneDrawMode: v }),

  generate: async () => {
    const { selectedBuildingId, building, mission, algorithm, disabledFacades, enabledCandidates, exclusionZones } = get();
    if (!selectedBuildingId) return;
    set({ loading: true });
    try {
      const result = await api.generate({
        building_id: selectedBuildingId,
        building,
        mission,
        algorithm,
        disabled_facades: disabledFacades.size > 0 ? [...disabledFacades] : undefined,
        enabled_candidates: enabledCandidates.size > 0 ? [...enabledCandidates] : undefined,
        exclusion_zones: exclusionZones.length > 0 ? exclusionZones : undefined,
      });
      set({ result, loading: false });
      get().refreshVersions();
    } catch (e) {
      console.error('Generate failed:', e);
      set({ loading: false });
    }
  },

  loadVersion: async (id) => {
    set({ loading: true });
    try {
      const result = await api.getVersion(id);
      set({
        result,
        building: result.config_snapshot.building,
        mission: result.config_snapshot.mission,
        algorithm: result.config_snapshot.algorithm || { ...DEFAULT_ALGORITHM },
        disabledFacades: new Set(result.config_snapshot.disabled_facades ?? []),
        enabledCandidates: new Set(result.config_snapshot.enabled_candidates ?? []),
        exclusionZones: result.config_snapshot.exclusion_zones ?? [],
        loading: false,
      });
    } catch (e) {
      console.error('Load version failed:', e);
      set({ loading: false });
    }
  },

  deleteVersion: async (id) => {
    try {
      await api.deleteVersion(id);
      get().refreshVersions();
    } catch (e) {
      console.error('Delete version failed:', e);
    }
  },

  deleteAllVersions: async () => {
    try {
      await api.deleteAllVersions();
      set({ versions: [] });
    } catch (e) {
      console.error('Delete all versions failed:', e);
    }
  },

  refreshVersions: async () => {
    try {
      const data = await api.getVersions();
      set({ versions: data.versions });
    } catch (e) {
      console.error('Refresh versions failed:', e);
    }
  },

  rewriteGimbals: async () => {
    const current = get().result;
    if (!current?.version_id) return;
    set({ loading: true });
    try {
      const res = await api.rewriteGimbals(current.version_id);
      await get().loadVersion(res.version_id);
      // Persist the tweaked DJI path as the new dji snapshot.
      const buildingId = get().result?.building_id ?? get().selectedBuildingId;
      const mode = get().kmzMode;
      if (buildingId && (mode === 'dji' || mode === 'inspection')) {
        try {
          await api.saveBuildingSnapshot(buildingId, mode, res.version_id);
        } catch (snapErr) {
          console.warn('Persist snapshot after rewrite failed:', snapErr);
        }
      }
      await Promise.all([get().refreshVersions(), get().refreshBuildings()]);
    } catch (e) {
      console.error('Rewrite gimbals failed:', e);
      set({ loading: false });
    }
  },

  // Generate a fresh NEN-2767 inspection mission from the KMZ-imported
  // building's facades — does NOT reuse DJI's trajectory. Produces a new
  // per-facade boustrophedon grid with stop-and-shoot, perpendicular
  // gimbals, and inspection-grade GSD. Leaves the original version for
  // side-by-side comparison.
  generateInspectionMission: async () => {
    const current = get().result;
    const buildingId = current?.building_id ?? current?.config_snapshot?.building_id;
    if (!buildingId) {
      console.error('No building_id on current result — cannot generate inspection mission');
      return;
    }
    // First-time entry to inspection mode seeds the NEN-2767 preset; afterwards
    // the current mission params (driven by the sidebar sliders) are used so
    // GSD/overlap/speed/camera changes actually re-render the path.
    const wasInspection = get().kmzMode === 'inspection';
    const seedMission: MissionParams = wasInspection ? get().mission : { ...NEN2767_MISSION };
    const { building, algorithm, disabledFacades, enabledCandidates, exclusionZones } = get();
    set({ loading: true, mission: seedMission, kmzMode: 'inspection' });
    try {
      const result = await api.generate({
        building_id: buildingId,
        building,
        mission: seedMission,
        algorithm,
        disabled_facades: disabledFacades.size > 0 ? [...disabledFacades] : undefined,
        enabled_candidates: enabledCandidates.size > 0 ? [...enabledCandidates] : undefined,
        exclusion_zones: exclusionZones.length > 0 ? exclusionZones : undefined,
      });
      set({ result, loading: false });
      // Persist the inspection snapshot so switching modes later is instant.
      try {
        await api.saveBuildingSnapshot(buildingId, 'inspection', result.version_id, { ...seedMission });
      } catch (snapErr) {
        console.warn('Persist inspection snapshot failed:', snapErr);
      }
      await Promise.all([get().refreshVersions(), get().refreshBuildings()]);
    } catch (e) {
      console.error('Generate inspection mission failed:', e);
      set({ loading: false });
    }
  },


  kmzOptimizing: false,
  kmzOptimizeMessage: '',
  kmzAutoRefine: false,
  setKmzAutoRefine: (v: boolean) => set({ kmzAutoRefine: v }),
  kmzOptimizeMin: 0.08,
  kmzOptimizeMax: 0.16,
  kmzOptimizeSteps: 3,
  kmzAwAlpha: null,
  kmzAwOffset: null,
  setKmzReconParams: (p) => set(p),
  // Facade detection defaults: null = use backend library defaults.
  kmzFdEpsilon: null,
  kmzFdClusterEpsilon: null,
  kmzFdMinPoints: null,
  kmzFdMinWallArea: null,
  kmzFdMinRoofArea: null,
  kmzFdMinDensity: null,
  kmzFdNormalThreshold: null,
  setKmzFacadeParams: (p) => set(p),
  kmzMode: 'dji',
  setKmzMode: (v) => set({ kmzMode: v }),
  buildings: [],
  refreshBuildings: async () => {
    try {
      const data = await api.listBuildings();
      set({ buildings: data.buildings });
    } catch (e) {
      console.error('Refresh buildings failed:', e);
    }
  },
  selectBuilding: async (id, mode) => {
    // Fast path: hit /buildings/{id}/load which returns the gzipped snapshot
    // for the requested mode plus the exact settings that produced it. No
    // alpha_wrap, no CGAL extraction — instant. Falls back to DJI refine only
    // if nothing is cached (409 from backend).
    set({ selectedBuildingId: id, loading: true });
    try {
      const resp = await api.loadBuilding(id, mode);
      const { result, settings, mode: loadedMode } = resp;
      const patch: Partial<AppState> = { result, loading: false, kmzMode: loadedMode };
      if (settings.voxel_size != null) patch.kmzOptimizeMin = settings.voxel_size;
      if ('aw_alpha' in settings) patch.kmzAwAlpha = settings.aw_alpha ?? null;
      if ('aw_offset' in settings) patch.kmzAwOffset = settings.aw_offset ?? null;
      if ('fd_epsilon' in settings) patch.kmzFdEpsilon = settings.fd_epsilon ?? null;
      if ('fd_cluster_epsilon' in settings) patch.kmzFdClusterEpsilon = settings.fd_cluster_epsilon ?? null;
      if ('fd_min_points' in settings) patch.kmzFdMinPoints = settings.fd_min_points ?? null;
      if ('fd_min_wall_area_m2' in settings) patch.kmzFdMinWallArea = settings.fd_min_wall_area_m2 ?? null;
      if ('fd_min_roof_area_m2' in settings) patch.kmzFdMinRoofArea = settings.fd_min_roof_area_m2 ?? null;
      if ('fd_min_density_per_m2' in settings) patch.kmzFdMinDensity = settings.fd_min_density_per_m2 ?? null;
      if ('fd_normal_threshold' in settings) patch.kmzFdNormalThreshold = settings.fd_normal_threshold ?? null;
      set(patch);
      await get().refreshVersions();
    } catch (e) {
      console.warn('Fast load failed, falling back to refine:', e);
      set({ loading: false });
      if (!mode || mode === 'dji') get().triggerRefine();
    }
  },
  switchMode: async (mode) => {
    // Mode toggle: load the requested snapshot. If not yet generated,
    // surface the failure so the UI can offer the appropriate generate
    // button (triggerRefine for dji, generateInspectionMission for inspection).
    const id = get().selectedBuildingId;
    set({ kmzMode: mode });
    if (!id) return;
    try {
      await get().selectBuilding(id, mode);
      await get().refreshBuildings();
    } catch (e) {
      console.warn('Mode switch failed:', e);
    }
  },
  deleteBuilding: async (id) => {
    try {
      await api.deleteBuilding(id);
      set((s) => {
        const wasSelected = s.selectedBuildingId === id;
        return {
          buildings: s.buildings.filter((b) => b.id !== id),
          selectedBuildingId: wasSelected ? null : s.selectedBuildingId,
          // Clear the viewer if the deleted building was the one on screen.
          result: wasSelected ? null : s.result,
        };
      });
    } catch (e) {
      console.error('Delete building failed:', e);
    }
  },
  stripRosetteOnly: async () => {
    // Keep DJI's gimbals, only strip the SmartOblique rosette + cap speed.
    // Uses the rewrite-gimbals endpoint with a very lenient pitch_margin
    // so no gimbal angle changes materially — and we still get the
    // smart_oblique strip + speed cap.
    const current = get().result;
    if (!current?.version_id) return;
    set({ loading: true });
    try {
      const res = await api.rewriteGimbals(current.version_id, {
        rewrite_angles: false,
        strip_smart_oblique: true,
      });
      await get().loadVersion(res.version_id);
      const buildingId = get().result?.building_id ?? get().selectedBuildingId;
      if (buildingId) {
        try {
          await api.saveBuildingSnapshot(buildingId, 'dji', res.version_id);
        } catch (snapErr) {
          console.warn('Persist snapshot after strip failed:', snapErr);
        }
      }
      await Promise.all([get().refreshVersions(), get().refreshBuildings()]);
    } catch (e) {
      console.error('Strip rosette failed:', e);
      set({ loading: false });
    }
  },
  refineRunning: false,
  refinePending: false,
  missionUpdateRunning: false,
  missionUpdatePending: false,
  triggerMissionUpdate: () => {
    // Coalescing fire-and-forget wrapper for mission-param changes (GSD,
    // overlap, flight_speed, camera, etc.). In inspection mode we regenerate
    // the NEN-2767 path; in dji mode there's nothing to regenerate from the
    // current sliders (DJI path comes from the imported KMZ), so it's a no-op.
    const run = async () => {
      if (get().missionUpdateRunning) {
        set({ missionUpdatePending: true });
        return;
      }
      set({ missionUpdateRunning: true, missionUpdatePending: false });
      try {
        if (get().kmzMode === 'inspection') {
          await get().generateInspectionMission();
        }
      } catch (e) {
        console.error('[triggerMissionUpdate] failed:', e);
      } finally {
        const pending = get().missionUpdatePending;
        set({ missionUpdateRunning: false, missionUpdatePending: false });
        if (pending) run();
      }
    };
    void run();
  },
  triggerRefine: () => {
    // Coalescing fire-and-forget wrapper. If a refine is in flight, mark
    // the next one as pending; when the in-flight one finishes we kick off
    // a fresh refine with the latest params.
    const run = async () => {
      if (get().refineRunning) {
        set({ refinePending: true });
        return;
      }
      set({ refineRunning: true, refinePending: false });
      try {
        await get().refineKmz(get().kmzOptimizeMin);
      } catch (e) {
        console.error('[triggerRefine] refine failed:', e);
      } finally {
        const pending = get().refinePending;
        set({ refineRunning: false, refinePending: false });
        if (pending) run();
      }
    };
    void run();
  },
  showOriginalGimbals: true,
  setShowOriginalGimbals: (v: boolean) => set({ showOriginalGimbals: v }),
  // Default ON: DJI Smart Auto Explore captures 5 photos per waypoint (1 nadir
  // + 4 obliques). Showing just the planned pose = -90° nadir only, which
  // looks "all straight down" and hides the oblique coverage that actually
  // drives the reconstruction.
  showRosettePoses: true,
  setShowRosettePoses: (v: boolean) => set({ showRosettePoses: v }),
  showMappingBox: false,
  setShowMappingBox: (v: boolean) => set({ showMappingBox: v }),
  mappingBoxSource: 'polygon',
  setMappingBoxSource: (v) => set({ mappingBoxSource: v }),
  cameraFovOverride: { fov_h_deg: null, fov_v_deg: null, distance_m: null },
  setCameraFovOverride: (patch) => set((s) => ({
    cameraFovOverride: { ...s.cameraFovOverride, ...patch },
  })),
  resetCameraFovOverride: () => set({
    cameraFovOverride: { fov_h_deg: null, fov_v_deg: null, distance_m: null },
  }),

  refineKmz: async (voxelSize: number) => {
    const s = get();
    if (!s.selectedBuildingId) return;
    // Clamp: values below each backend minimum fall back to library defaults
    // (null). This lets the UI sliders start at 0 (= "auto") without tripping
    // Pydantic validation on the way up.
    const gte = (v: number | null | undefined, lo: number) =>
      (v != null && v >= lo) ? v : null;
    set({ kmzOptimizeMessage: `Refining (voxel=${voxelSize.toFixed(2)}m)…` });
    let task_id: string;
    try {
      ({ task_id } = await api.refineKmzBuilding(s.selectedBuildingId, voxelSize, {
        awAlpha: s.kmzAwAlpha, awOffset: s.kmzAwOffset,
        fdEpsilon: gte(s.kmzFdEpsilon, 0.005),
        fdClusterEpsilon: gte(s.kmzFdClusterEpsilon, 0.02),
        fdMinPoints: gte(s.kmzFdMinPoints, 5),
        fdMinWallAreaM2: gte(s.kmzFdMinWallArea, 0.05),
        fdMinRoofAreaM2: gte(s.kmzFdMinRoofArea, 0.05),
        fdMinDensityPerM2: gte(s.kmzFdMinDensity, 1.0),
        fdNormalThreshold: gte(s.kmzFdNormalThreshold, 0.5),
      }));
    } catch (e) {
      console.error('Refine request failed:', e);
      set({ kmzOptimizeMessage: `Refine failed: ${String(e)}` });
      throw e;
    }
    const result = await new Promise<GenerateResponse & { building_id?: string }>((resolve, reject) => {
      const poll = async () => {
        try {
          const status = await api.getKmzImportStatus(task_id);
          set({ kmzOptimizeMessage: status.message });
          if (status.status === 'complete' && status.result) {
            resolve(status.result as GenerateResponse & { building_id?: string });
          } else if (status.status === 'error') {
            reject(new Error(status.error || 'Refine failed'));
          } else {
            setTimeout(poll, 1000);
          }
        } catch (e) { reject(e); }
      };
      poll();
    });
    set({ result });
    await get().refreshVersions();
  },

  cancelOptimize: () => set({ kmzOptimizing: false, kmzOptimizeMessage: 'Cancelled' }),

  optimizeKmz: async (buildingId?: string | null) => {
    const target = buildingId ?? get().selectedBuildingId;
    if (!target) return;
    if (get().kmzOptimizing) return;
    const { kmzOptimizeMin, kmzOptimizeMax, kmzOptimizeSteps } = get();
    const vMin = Math.max(0.04, Math.min(0.30, kmzOptimizeMin));
    const vMax = Math.max(vMin + 0.01, Math.min(0.40, kmzOptimizeMax));
    const n = Math.max(1, Math.min(6, kmzOptimizeSteps | 0));
    // Geometric ramp coarse → fine (coarse first stabilizes topology, fine passes add detail).
    const schedule: number[] = n === 1
      ? [vMin]
      : Array.from({ length: n }, (_, i) => {
          const t = i / (n - 1);
          return +(vMax * Math.pow(vMin / vMax, t)).toFixed(3);
        });
    set({ kmzOptimizing: true, kmzOptimizeMessage: 'Starting optimization…' });
    try {
      for (let i = 0; i < schedule.length; i++) {
        if (!get().kmzOptimizing) break;
        const v = schedule[i];
        set({ kmzOptimizeMessage: `Pass ${i + 1}/${schedule.length} (voxel=${v.toFixed(2)}m)…` });
        try {
          await get().refineKmz(v);
        } catch (e) {
          console.warn(`Optimize pass voxel=${v} failed:`, e);
          set({ kmzOptimizeMessage: `Pass ${i + 1} failed — stopping` });
          break;
        }
      }
      if (get().kmzOptimizing) {
        set({ kmzOptimizeMessage: 'Optimization complete' });
      }
    } finally {
      set({ kmzOptimizing: false });
      setTimeout(() => {
        if (!get().kmzOptimizing) set({ kmzOptimizeMessage: '' });
      }, 4000);
    }
  },

  kmzImportMode: 'raw',
  setKmzImportMode: (v) => set({ kmzImportMode: v }),
  lastKmzFile: null,
  toggleKmzFacades: async () => {
    const { lastKmzFile, result } = get();
    if (!lastKmzFile) return;
    const currentSource = result?.summary?.source;
    const nextMode = currentSource === 'kmz_raw' ? 'facades' : 'raw';
    await get().importKmz(lastKmzFile, null, nextMode);
  },

  importKmz: async (file: File, voxelSizeArg?: number | null, modeArg?: 'raw' | 'facades') => {
    const fileSizeMB = (file.size / (1024 * 1024)).toFixed(1);
    const voxelSize = voxelSizeArg ?? null;
    const mode = modeArg ?? get().kmzImportMode;
    set({ uploading: true, uploadProgress: 0, uploadMessage: `Uploading ${fileSizeMB} MB KMZ…`, lastKmzFile: file });
    try {
      const { task_id } = await api.importKmz(file, voxelSize, (loaded, total) => {
        const pct = total > 0 ? loaded / total : 0;
        const loadedMB = (loaded / (1024 * 1024)).toFixed(1);
        set({ uploadProgress: pct * 0.3, uploadMessage: `Uploading ${loadedMB}/${fileSizeMB} MB…` });
      }, mode);

      set({ uploadProgress: 0.3, uploadMessage: 'Parsing KMZ…' });
      const result = await new Promise<GenerateResponse & { building_id?: string }>((resolve, reject) => {
        const poll = async () => {
          try {
            const status = await api.getKmzImportStatus(task_id);
            const progress = 0.3 + status.progress * 0.7;
            set({ uploadProgress: progress, uploadMessage: status.message });
            if (status.status === 'complete' && status.result) {
              if (status.phase_timings) {
                set({ lastPhaseTimings: status.phase_timings });
                console.log('[aeroscan] import-kmz phase_timings', status.phase_timings);
              }
              resolve(status.result as GenerateResponse & { building_id?: string });
            } else if (status.status === 'error') {
              if (status.phase_timings) set({ lastPhaseTimings: status.phase_timings });
              reject(new Error(status.error || 'Import failed'));
            } else {
              setTimeout(poll, 1000);
            }
          } catch (e) {
            reject(e);
          }
        };
        poll();
      });

      set({
        result,
        uploading: false,
        uploadProgress: 0,
        uploadMessage: '',
        activeTab: '3d',
      });
      await get().refreshVersions();
      if (result.building_id) {
        set({ selectedBuildingId: result.building_id });
      }
      void get().refreshBuildings();

      // --- Background optimization chain (fire-and-forget, non-blocking) ---
      // Raw mode has no mesh to refine, so skip the chain entirely.
      if (mode === 'facades' && get().kmzAutoRefine && result.building_id) {
        void get().optimizeKmz(result.building_id);
      }
    } catch (e) {
      console.error('KMZ import failed:', e);
      set({ uploading: false, uploadProgress: 0, uploadMessage: String(e) });
    }
  },

  // --- Simulation / reconstruction ---
  simTaskId: null,
  simStatus: null,
  simProgress: 0,
  simMessage: '',
  simStartTime: null,
  simResult: null,
  simViewerData: null,

  startSimulation: async (renderScale?: number, voxelSize?: number) => {
    const { result } = get();
    if (!result) return;

    set({ simStatus: 'starting', simProgress: 0, simMessage: 'Starting simulation…', simResult: null, simStartTime: Date.now() });
    try {
      const { task_id } = await api.startSimulation(result.version_id, renderScale, voxelSize);
      set({ simTaskId: task_id, simStatus: 'rendering', simMessage: 'Rendering synthetic photos…' });

      const poll = async () => {
        const status = await api.getSimulationStatus(task_id);
        set({ simStatus: status.status, simProgress: status.progress, simMessage: status.message });

        if (status.status === 'complete' && status.result) {
          set({ simResult: status.result, simStatus: 'complete', activeTab: 'sim' });
          // Reset to ready state after a brief delay so sidebar shows presets again
          setTimeout(() => set({ simStatus: null, simProgress: 0, simMessage: '' }), 1500);
        } else if (status.status === 'error') {
          console.error('Simulation error:', status.error);
        } else {
          setTimeout(poll, 2000);
        }
      };
      poll();
    } catch (e) {
      console.error('Start simulation failed:', e);
      set({ simStatus: 'error', simMessage: String(e) });
    }
  },

  viewSimulationResult: () => {
    set({ activeTab: 'sim' });
  },

  deleteSimulation: async () => {
    const { simTaskId } = get();
    if (!simTaskId) return;
    try {
      await api.deleteSimulation(simTaskId);
    } catch { /* task may already be gone */ }
    set({ simTaskId: null, simStatus: null, simProgress: 0, simMessage: '', simStartTime: null, simResult: null, simViewerData: null });
  },

  loadSimFromUrl: async () => {
    const params = new URLSearchParams(window.location.search);
    const taskId = params.get('sim_task');
    if (!taskId) return;
    try {
      const status = await api.getSimulationStatus(taskId);
      if (status.status === 'complete' && status.result) {
        set({ simViewerData: status.result.viewer_data, simResult: status.result, simTaskId: taskId });
      }
    } catch (e) {
      console.error('Failed to load simulation:', e);
    }
  },
}), {
  name: 'aeroscan-settings',
  partialize: (state) => ({
    lightMode: state.lightMode,
    activeTab: state.activeTab,
    building: state.building,
    mission: state.mission,
    algorithm: state.algorithm,
    sectionState: state.sectionState,
    kmzMode: state.kmzMode,
    kmzOptimizeMin: state.kmzOptimizeMin,
    kmzOptimizeMax: state.kmzOptimizeMax,
    kmzOptimizeSteps: state.kmzOptimizeSteps,
    kmzAwAlpha: state.kmzAwAlpha,
    kmzAwOffset: state.kmzAwOffset,
    kmzFdEpsilon: state.kmzFdEpsilon,
    kmzFdClusterEpsilon: state.kmzFdClusterEpsilon,
    kmzFdMinPoints: state.kmzFdMinPoints,
    kmzFdMinWallArea: state.kmzFdMinWallArea,
    kmzFdMinRoofArea: state.kmzFdMinRoofArea,
    kmzFdMinDensity: state.kmzFdMinDensity,
    kmzFdNormalThreshold: state.kmzFdNormalThreshold,
  }),
}));
