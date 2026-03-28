import { create } from 'zustand';
import type { BuildingParams, GenerateResponse, MissionParams, UploadedBuilding, VersionSummary } from './api/types';
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
  flight_speed_ms: 3.0,
  obstacle_clearance_m: 2.0,
  mission_name: 'AeroScan Inspection',
  gimbal_pitch_margin_deg: 5.0,
  min_photo_distance_m: 1.5,
  yaw_rate_deg_per_s: 60.0,
};

export const PRESETS: Record<string, Partial<BuildingParams>> = {
  simple_box: { width: 20, depth: 10, height: 8, heading_deg: 0, roof_type: 'flat', roof_pitch_deg: 0, num_stories: 3 },
  pitched_roof_house: { width: 30, depth: 10, height: 6, heading_deg: 45, roof_type: 'pitched', roof_pitch_deg: 30, num_stories: 2 },
  l_shaped_block: { width: 25, depth: 10, height: 9, heading_deg: 0, roof_type: 'flat', roof_pitch_deg: 0, num_stories: 3 },
  large_apartment_block: { width: 60, depth: 12, height: 18, heading_deg: 15, roof_type: 'flat', roof_pitch_deg: 0, num_stories: 6 },
};

type Tab = '3d' | 'map';
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

  // Mesh settings
  minFacadeArea: number;

  // Result
  result: GenerateResponse | null;
  versions: VersionSummary[];
  loading: boolean;
  uploading: boolean;
  activeTab: Tab;

  // Actions
  setMinFacadeArea: (v: number) => void;
  setBuildingSource: (source: BuildingSource) => void;
  setPreset: (name: string | null) => void;
  setBuilding: (patch: Partial<BuildingParams>) => void;
  setMission: (patch: Partial<MissionParams>) => void;
  setActiveTab: (tab: Tab) => void;
  generate: () => Promise<void>;
  loadVersion: (id: string) => Promise<void>;
  deleteVersion: (id: string) => Promise<void>;
  refreshVersions: () => Promise<void>;

  // Building upload actions
  uploadBuilding: (file: File) => Promise<void>;
  selectBuilding: (id: string) => void;
  deleteBuilding: (id: string) => Promise<void>;
  refreshBuildings: () => Promise<void>;
}

export const useStore = create<AppState>((set, get) => ({
  buildingSource: 'preset',
  selectedBuildingId: null,
  buildings: [],
  minFacadeArea: 1.0,

  preset: 'simple_box',
  building: { ...DEFAULT_BUILDING },
  mission: { ...DEFAULT_MISSION },
  result: null,
  versions: [],
  loading: false,
  uploading: false,
  activeTab: '3d',

  setMinFacadeArea: (v) => set({ minFacadeArea: v }),

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

  setActiveTab: (tab) => set({ activeTab: tab }),

  generate: async () => {
    const { preset, selectedBuildingId, buildingSource, building, mission, minFacadeArea } = get();
    set({ loading: true });
    try {
      const result = await api.generate({
        preset: buildingSource === 'preset' ? preset : undefined,
        building_id: buildingSource === 'upload' ? selectedBuildingId : undefined,
        building,
        mission,
        min_facade_area: buildingSource === 'upload' ? minFacadeArea : undefined,
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
}));
