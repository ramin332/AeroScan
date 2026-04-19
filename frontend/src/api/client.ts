import type {
  BenchmarkResult,
  BuildingUploadRequest,
  DroneSpec,
  GenerateRequest,
  GenerateResponse,
  PresetsResponse,
  SimulationStatus,
  UploadedBuilding,
  VersionSummary,
} from './types';

const BASE = '/api';

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${BASE}${path}`, init);
  if (!res.ok) {
    const detail = await res.text();
    throw new Error(`API error ${res.status}: ${detail}`);
  }
  return res.json();
}

export async function getPresets(): Promise<PresetsResponse> {
  return request('/presets');
}

export async function getDrone(): Promise<DroneSpec> {
  return request('/drone');
}

export async function generate(req: GenerateRequest): Promise<GenerateResponse> {
  return request('/generate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(req),
  });
}

export async function benchmarkTsp(req: GenerateRequest): Promise<{ benchmark: BenchmarkResult[]; facade_count: number }> {
  return request('/benchmark-tsp', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(req),
  });
}

export async function getVersions(): Promise<{ versions: VersionSummary[] }> {
  return request('/versions');
}

export async function getVersion(id: string): Promise<GenerateResponse> {
  return request(`/versions/${id}`);
}

export async function deleteVersion(id: string): Promise<void> {
  await request(`/versions/${id}`, { method: 'DELETE' });
}

export async function deleteAllVersions(): Promise<void> {
  await request('/versions', { method: 'DELETE' });
}

export function kmzDownloadUrl(versionId: string): string {
  return `${BASE}/versions/${versionId}/kmz`;
}

export async function rewriteGimbals(
  versionId: string,
  params?: {
    max_distance_m?: number;
    pitch_margin_deg?: number;
    preserve_heading?: boolean;
    rewrite_angles?: boolean;
    flight_speed_ms?: number | null;
    strip_smart_oblique?: boolean;
  },
): Promise<{
  version_id: string;
  parent_version_id: string;
  rewritten_count: number;
  total_waypoints: number;
  timestamp: string;
  summary: Record<string, unknown>;
  viewer_data: { threejs: unknown; leaflet: unknown };
}> {
  return request(`/versions/${versionId}/rewrite-gimbals`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(params ?? {}),
  });
}

// --- Building CRUD ---

export async function uploadBuilding(req: BuildingUploadRequest): Promise<UploadedBuilding> {
  return request('/buildings', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(req),
  });
}

export async function listBuildings(): Promise<{ buildings: UploadedBuilding[] }> {
  return request('/buildings');
}

export async function getBuilding(id: string): Promise<UploadedBuilding> {
  return request(`/buildings/${id}`);
}

export async function deleteBuilding(id: string): Promise<void> {
  await request(`/buildings/${id}`, { method: 'DELETE' });
}

export interface LoadedBuildingSettings {
  voxel_size?: number | null;
  aw_alpha?: number | null;
  aw_offset?: number | null;
  fd_epsilon?: number | null;
  fd_cluster_epsilon?: number | null;
  fd_min_points?: number | null;
  fd_min_wall_area_m2?: number | null;
  fd_min_roof_area_m2?: number | null;
  fd_min_density_per_m2?: number | null;
  fd_normal_threshold?: number | null;
}

export async function loadBuilding(
  id: string,
  mode?: 'dji' | 'inspection',
): Promise<{
  result: GenerateResponse;
  settings: LoadedBuildingSettings;
  mode: 'dji' | 'inspection';
  available_modes: Array<'dji' | 'inspection'>;
}> {
  const qs = mode ? `?mode=${mode}` : '';
  return request(`/buildings/${id}/load${qs}`, { method: 'POST' });
}

export async function saveBuildingSnapshot(
  buildingId: string,
  mode: 'dji' | 'inspection',
  versionId: string,
  settings?: Record<string, unknown>,
): Promise<{ ok: true; mode: string; version_id: string }> {
  return request(`/buildings/${buildingId}/snapshots/${mode}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ version_id: versionId, settings: settings ?? null }),
  });
}

export async function deleteBuildingSnapshot(
  buildingId: string,
  mode: 'dji' | 'inspection',
): Promise<{ ok: true; removed: boolean }> {
  return request(`/buildings/${buildingId}/snapshots/${mode}`, { method: 'DELETE' });
}

export function uploadMeshFile(
  file: File,
  params: { name: string; lat: number; lon: number; height: number; num_stories: number; min_facade_area: number },
  onUploadProgress?: (loaded: number, total: number) => void,
): Promise<{ task_id: string }> {
  const form = new FormData();
  form.append('file', file);
  form.append('name', params.name);
  form.append('lat', String(params.lat));
  form.append('lon', String(params.lon));
  form.append('height', String(params.height));
  form.append('num_stories', String(params.num_stories));
  form.append('min_facade_area', String(params.min_facade_area));

  return new Promise((resolve, reject) => {
    const xhr = new XMLHttpRequest();
    xhr.open('POST', `${BASE}/buildings/upload-file`);

    if (onUploadProgress) {
      xhr.upload.onprogress = (e) => {
        if (e.lengthComputable) onUploadProgress(e.loaded, e.total);
      };
    }

    xhr.onload = () => {
      if (xhr.status >= 200 && xhr.status < 300) {
        resolve(JSON.parse(xhr.responseText));
      } else {
        reject(new Error(`API error ${xhr.status}: ${xhr.responseText}`));
      }
    };
    xhr.onerror = () => reject(new Error('Upload network error'));
    xhr.send(form);
  });
}

export async function getUploadStatus(taskId: string): Promise<{
  status: string; progress: number; message: string;
  result?: UploadedBuilding; error?: string;
  phase_timings?: Array<{ label: string; seconds: number }>;
}> {
  return request(`/buildings/upload-status/${taskId}`);
}

// --- KMZ import ---

export function importKmz(
  file: File,
  voxelSize?: number | null,
  onUploadProgress?: (loaded: number, total: number) => void,
  mode: 'raw' | 'facades' = 'facades',
): Promise<{ task_id: string }> {
  const form = new FormData();
  form.append('file', file);
  if (voxelSize && voxelSize > 0) form.append('voxel_size', String(voxelSize));
  form.append('import_mode', mode);

  return new Promise((resolve, reject) => {
    const xhr = new XMLHttpRequest();
    xhr.open('POST', `${BASE}/import-kmz`);

    if (onUploadProgress) {
      xhr.upload.onprogress = (e) => {
        if (e.lengthComputable) onUploadProgress(e.loaded, e.total);
      };
    }

    xhr.onload = () => {
      if (xhr.status >= 200 && xhr.status < 300) {
        resolve(JSON.parse(xhr.responseText));
      } else {
        reject(new Error(`API error ${xhr.status}: ${xhr.responseText}`));
      }
    };
    xhr.onerror = () => reject(new Error('Upload network error'));
    xhr.send(form);
  });
}

export async function getKmzImportStatus(taskId: string): Promise<{
  status: string; progress: number; message: string;
  result?: GenerateResponse & { building_id?: string; imported_name?: string };
  error?: string;
  phase_timings?: Array<{ label: string; seconds: number }>;
}> {
  // Reuses the shared upload-status endpoint
  return request(`/buildings/upload-status/${taskId}`);
}

export interface RefineKmzOpts {
  awAlpha?: number | null;
  awOffset?: number | null;
  fdEpsilon?: number | null;
  fdClusterEpsilon?: number | null;
  fdMinPoints?: number | null;
  fdMinWallAreaM2?: number | null;
  fdMinRoofAreaM2?: number | null;
  fdMinDensityPerM2?: number | null;
  fdNormalThreshold?: number | null;
}

export async function refineKmzBuilding(
  buildingId: string,
  voxelSize: number,
  opts?: RefineKmzOpts,
): Promise<{ task_id: string }> {
  const body: Record<string, number> = { voxel_size: voxelSize };
  if (opts?.awAlpha != null) body.aw_alpha = opts.awAlpha;
  if (opts?.awOffset != null) body.aw_offset = opts.awOffset;
  if (opts?.fdEpsilon != null) body.fd_epsilon = opts.fdEpsilon;
  if (opts?.fdClusterEpsilon != null) body.fd_cluster_epsilon = opts.fdClusterEpsilon;
  if (opts?.fdMinPoints != null) body.fd_min_points = opts.fdMinPoints;
  if (opts?.fdMinWallAreaM2 != null) body.fd_min_wall_area_m2 = opts.fdMinWallAreaM2;
  if (opts?.fdMinRoofAreaM2 != null) body.fd_min_roof_area_m2 = opts.fdMinRoofAreaM2;
  if (opts?.fdMinDensityPerM2 != null) body.fd_min_density_per_m2 = opts.fdMinDensityPerM2;
  if (opts?.fdNormalThreshold != null) body.fd_normal_threshold = opts.fdNormalThreshold;
  return request(`/buildings/${buildingId}/refine-kmz`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
}

// --- Simulation / Reconstruction ---

export async function listSimulations(): Promise<{ tasks: SimulationStatus[] }> {
  return request('/simulate-reconstruct');
}

export async function startSimulation(
  versionId?: string,
  renderScale = 0.1,
  voxelSize = 0.04,
): Promise<{ task_id: string; status: string; source_version: string }> {
  return request('/simulate-reconstruct', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      version_id: versionId ?? null,
      render_scale: renderScale,
      voxel_size: voxelSize,
    }),
  });
}

export async function getSimulationStatus(taskId: string): Promise<SimulationStatus> {
  return request(`/simulate-reconstruct/${taskId}`);
}

export async function deleteSimulation(taskId: string): Promise<void> {
  await request(`/simulate-reconstruct/${taskId}`, { method: 'DELETE' });
}

export function simulationPhotoUrl(taskId: string, wpIndex: number): string {
  return `${BASE}/simulate-reconstruct/${taskId}/photo/${wpIndex}`;
}
