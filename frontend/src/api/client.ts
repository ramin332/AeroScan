import type {
  BuildingUploadRequest,
  DroneSpec,
  GenerateRequest,
  GenerateResponse,
  PresetsResponse,
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

export async function uploadMeshFile(
  file: File,
  params: { name: string; lat: number; lon: number; height: number; num_stories: number; min_facade_area: number },
): Promise<UploadedBuilding> {
  const form = new FormData();
  form.append('file', file);
  form.append('name', params.name);
  form.append('lat', String(params.lat));
  form.append('lon', String(params.lon));
  form.append('height', String(params.height));
  form.append('num_stories', String(params.num_stories));
  form.append('min_facade_area', String(params.min_facade_area));
  return request('/buildings/upload-file', { method: 'POST', body: form });
}
