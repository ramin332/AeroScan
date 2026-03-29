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

export const PRESETS: Record<string, Partial<BuildingParams>> = {
  simple_box: { width: 20, depth: 10, height: 8, heading_deg: 0, roof_type: 'flat', roof_pitch_deg: 0, num_stories: 3 },
  pitched_roof_house: { width: 30, depth: 10, height: 6, heading_deg: 45, roof_type: 'pitched', roof_pitch_deg: 30, num_stories: 2 },
  l_shaped_block: { width: 25, depth: 10, height: 9, heading_deg: 0, roof_type: 'flat', roof_pitch_deg: 0, num_stories: 3 },
  large_apartment_block: { width: 60, depth: 12, height: 18, heading_deg: 15, roof_type: 'flat', roof_pitch_deg: 0, num_stories: 6 },
};

type Tab = '3d' | 'map' | 'sim';
type BuildingSource = 'upload' | 'preset';

interface AppState {
  // Building source
  buildingSource: BuildingSource;
  selectedBuildingId: string | null;
  buildings: UploadedBuilding[];

  // Params
  preset: string | null;
  building: BuildingParams;
  mission: MissionParams;
  algorithm: AlgorithmParams;

  // Mesh settings
  minFacadeArea: number;
  extractionMethod: string;
  waypointStrategy: string;

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
  activeTab: Tab;

  // Actions
  setMinFacadeArea: (v: number) => void;
  setExtractionMethod: (v: string) => void;
  setWaypointStrategy: (v: string) => void;
  setBuildingSource: (source: BuildingSource) => void;
  setPreset: (name: string | null) => void;
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

  // Theme
  lightMode: boolean;
  setLightMode: (v: boolean) => void;

  // Section open/close state
  sectionState: Record<string, boolean>;
  setSectionOpen: (key: string, open: boolean) => void;

  // Building upload actions
  uploadBuilding: (file: File) => Promise<void>;
  selectBuilding: (id: string) => void;
  deleteBuilding: (id: string) => Promise<void>;
  refreshBuildings: () => Promise<void>;

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
  buildingSource: 'preset',
  selectedBuildingId: null,
  buildings: [],
  minFacadeArea: 1.0,
  extractionMethod: 'region_growing',
  waypointStrategy: 'facade_grid',
  disabledFacades: new Set<number>(),
  enabledCandidates: new Set<number>(),
  exclusionZones: [],
  zoneDrawMode: false,

  preset: 'simple_box',
  building: { ...DEFAULT_BUILDING },
  mission: { ...DEFAULT_MISSION },
  algorithm: { ...DEFAULT_ALGORITHM },
  result: null,
  versions: [],
  loading: false,
  uploading: false,
  activeTab: '3d',
  lightMode: false,
  setLightMode: (v) => set({ lightMode: v }),
  sectionState: {},
  setSectionOpen: (key, open) => set((s) => ({ sectionState: { ...s.sectionState, [key]: open } })),

  setMinFacadeArea: (v) => set({ minFacadeArea: v }),
  setExtractionMethod: (v: string) => set({ extractionMethod: v }),
  setWaypointStrategy: (v: string) => set({ waypointStrategy: v }),

  setBuildingSource: (source) => {
    set({ buildingSource: source });
    if (source === 'preset') {
      set({ selectedBuildingId: null });
    } else {
      set({ preset: null });
    }
  },

  setPreset: (name) => {
    if (name && PRESETS[name]) {
      set({
        preset: name,
        buildingSource: 'preset',
        selectedBuildingId: null,
        building: { ...DEFAULT_BUILDING, ...PRESETS[name] },
      });
    } else {
      set({ preset: null });
    }
  },

  setBuilding: (patch) =>
    set((s) => ({ building: { ...s.building, ...patch }, preset: null })),

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
    const { preset, selectedBuildingId, buildingSource, building, mission, algorithm, minFacadeArea, extractionMethod, waypointStrategy, disabledFacades, enabledCandidates, exclusionZones } = get();
    set({ loading: true });
    try {
      const result = await api.generate({
        preset: buildingSource === 'preset' ? preset : undefined,
        building_id: buildingSource === 'upload' ? selectedBuildingId : undefined,
        building,
        mission,
        algorithm,
        min_facade_area: buildingSource === 'upload' ? minFacadeArea : undefined,
        extraction_method: buildingSource === 'upload' ? extractionMethod : undefined,
        waypoint_strategy: buildingSource === 'upload' ? waypointStrategy : undefined,
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

  // --- Building upload ---

  uploadBuilding: async (file: File) => {
    set({ uploading: true });
    try {
      const ext = file.name.split('.').pop()?.toLowerCase() || '';
      const isMesh = ['obj', 'ply', 'stl', 'glb', 'gltf'].includes(ext);
      let uploaded;

      if (isMesh) {
        // Mesh file → multipart upload
        const name = file.name.replace(/\.[^.]+$/, '') || 'Mesh building';
        const { building, minFacadeArea } = get();
        uploaded = await api.uploadMeshFile(file, {
          name,
          lat: building.lat,
          lon: building.lon,
          height: 0, // auto-detect from mesh
          num_stories: 1,
          min_facade_area: minFacadeArea,
        });
      } else {
        // GeoJSON → JSON upload
        const text = await file.text();
        const geojson = JSON.parse(text);
        const name = file.name.replace(/\.(geo)?json$/i, '') || 'Uploaded building';

        let height = 8.0;
        let numStories = 1;
        let roofType: 'flat' | 'pitched' = 'flat';
        let roofPitchDeg = 0;

        const props = geojson.type === 'Feature'
          ? geojson.properties || {}
          : geojson.type === 'FeatureCollection' && geojson.features?.[0]
            ? geojson.features[0].properties || {}
            : {};

        if (props.height) height = Number(props.height);
        if (props.num_stories) numStories = Number(props.num_stories);
        if (props.roof_type === 'pitched') roofType = 'pitched';
        if (props.roof_pitch_deg) roofPitchDeg = Number(props.roof_pitch_deg);

        uploaded = await api.uploadBuilding({
          name,
          geojson,
          height,
          num_stories: numStories,
          roof_type: roofType,
          roof_pitch_deg: roofPitchDeg,
        });
      }

      // Add to list, select it, and load its params
      set((s) => ({
        buildings: [uploaded, ...s.buildings],
        selectedBuildingId: uploaded.id,
        buildingSource: 'upload',
        preset: null,
        building: {
          ...s.building,
          lat: uploaded.lat,
          lon: uploaded.lon,
          height: uploaded.height,
          num_stories: uploaded.num_stories,
          roof_type: uploaded.roof_type as 'flat' | 'pitched',
          roof_pitch_deg: uploaded.roof_pitch_deg,
        },
        uploading: false,
      }));
    } catch (e) {
      console.error('Upload failed:', e);
      set({ uploading: false });
    }
  },

  selectBuilding: (id) => {
    const { buildings } = get();
    const b = buildings.find((b) => b.id === id);
    if (b) {
      set({
        selectedBuildingId: id,
        buildingSource: 'upload',
        preset: null,
        building: {
          ...get().building,
          lat: b.lat,
          lon: b.lon,
          height: b.height,
          num_stories: b.num_stories,
          roof_type: b.roof_type as 'flat' | 'pitched',
          roof_pitch_deg: b.roof_pitch_deg,
        },
      });
    }
  },

  deleteBuilding: async (id) => {
    try {
      await api.deleteBuilding(id);
      set((s) => ({
        buildings: s.buildings.filter((b) => b.id !== id),
        selectedBuildingId: s.selectedBuildingId === id ? null : s.selectedBuildingId,
      }));
    } catch (e) {
      console.error('Delete building failed:', e);
    }
  },

  refreshBuildings: async () => {
    try {
      const data = await api.listBuildings();
      set({ buildings: data.buildings });
    } catch (e) {
      console.error('Refresh buildings failed:', e);
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
          set({ simResult: status.result });
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
    buildingSource: state.buildingSource,
    preset: state.preset,
    building: state.building,
    mission: state.mission,
    algorithm: state.algorithm,
    minFacadeArea: state.minFacadeArea,
    extractionMethod: state.extractionMethod,
    waypointStrategy: state.waypointStrategy,
    sectionState: state.sectionState,
  }),
}));
