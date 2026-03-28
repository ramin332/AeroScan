import { useRef } from 'react';
import { useStore, PRESETS } from '../store';
import { kmzDownloadUrl } from '../api/client';
import { DroneInfo } from './DroneInfo';
import { VersionList } from './VersionList';

const PRESET_LABELS: Record<string, string> = {
  simple_box: 'Simple box (20x10x8m)',
  pitched_roof_house: 'Pitched roof (30x10x6m)',
  l_shaped_block: 'L-shaped block',
  large_apartment_block: 'Large apartment (60x12x18m)',
};

const CAMERA_LABELS: Record<string, string> = {
  wide: 'Wide 24mm',
  medium_tele: 'Medium tele 70mm',
  telephoto: 'Telephoto 168mm',
};

export function Sidebar() {
  const {
    buildingSource, selectedBuildingId, buildings, uploading,
    preset, building, mission, result, loading,
    setBuildingSource, setPreset, setBuilding, setMission,
    generate, uploadBuilding, selectBuilding, deleteBuilding,
  } = useStore();

  const fileRef = useRef<HTMLInputElement>(null);

  const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      uploadBuilding(file);
      e.target.value = '';
    }
  };

  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    const file = e.dataTransfer.files[0];
    if (file) {
      const ext = file.name.split('.').pop()?.toLowerCase() || '';
      const supported = ['json', 'geojson', 'obj', 'ply', 'stl', 'glb', 'gltf'];
      if (supported.includes(ext)) {
        uploadBuilding(file);
      }
    }
  };

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
  };

  const isUploadMode = buildingSource === 'upload';
  const selectedBuilding = buildings.find((b) => b.id === selectedBuildingId);
  const showBoxParams = !isUploadMode && preset !== 'l_shaped_block';

  return (
    <aside className="sidebar">
      <h1>AeroScan Flight Planner</h1>
      <DroneInfo />

      {/* ---- Building source toggle ---- */}
      <div className="section">
        <h3>Building</h3>
        <div className="source-toggle">
          <button
            className={`source-btn ${isUploadMode ? 'active' : ''}`}
            onClick={() => setBuildingSource('upload')}
          >
            Upload
          </button>
          <button
            className={`source-btn ${!isUploadMode ? 'active' : ''}`}
            onClick={() => setBuildingSource('preset')}
          >
            Preset
          </button>
        </div>
      </div>

      {/* ---- Upload mode ---- */}
      {isUploadMode && (
        <div className="section">
          <div
            className="upload-zone"
            onClick={() => fileRef.current?.click()}
            onDrop={handleDrop}
            onDragOver={handleDragOver}
          >
            <input
              ref={fileRef}
              type="file"
              accept=".json,.geojson,.obj,.ply,.stl,.glb,.gltf"
              onChange={handleFileChange}
              style={{ display: 'none' }}
            />
            {uploading ? (
              <span className="upload-text">Uploading...</span>
            ) : (
              <>
                <span className="upload-icon">+</span>
                <span className="upload-text">Upload Building</span>
                <span className="upload-hint">GeoJSON, OBJ, PLY, STL</span>
              </>
            )}
          </div>

          {buildings.length > 0 && (
            <div className="building-list">
              {buildings.map((b) => (
                <div
                  key={b.id}
                  className={`building-item ${selectedBuildingId === b.id ? 'active' : ''}`}
                  onClick={() => selectBuilding(b.id)}
                >
                  <div className="building-item-info">
                    <span className="building-name">{b.name}</span>
                    <span className="building-meta">
                      {b.lat.toFixed(4)}, {b.lon.toFixed(4)} &middot; {b.height}m
                    </span>
                  </div>
                  <span
                    className="del"
                    onClick={(e) => { e.stopPropagation(); deleteBuilding(b.id); }}
                  >
                    x
                  </span>
                </div>
              ))}
            </div>
          )}

          {/* Building info + editable location */}
          {selectedBuilding && (
            <div className="building-details">
              <div className="building-dims">
                {selectedBuilding.width > 0
                  ? `${selectedBuilding.width} \u00D7 ${selectedBuilding.depth} \u00D7 ${selectedBuilding.height} m`
                  : `${selectedBuilding.height} m tall`}
              </div>
              <div className="building-source">{selectedBuilding.source_type}</div>
              <Field label="Lat" value={building.lat} step={0.0001}
                onChange={(v) => setBuilding({ lat: v })} />
              <Field label="Lon" value={building.lon} step={0.0001}
                onChange={(v) => setBuilding({ lon: v })} />
            </div>
          )}
        </div>
      )}

      {/* ---- Preset mode ---- */}
      {!isUploadMode && (
        <>
          <div className="section">
            <select
              value={preset || ''}
              onChange={(e) => setPreset(e.target.value || null)}
            >
              <option value="">Custom building</option>
              {Object.keys(PRESETS).map((k) => (
                <option key={k} value={k}>{PRESET_LABELS[k] || k}</option>
              ))}
            </select>
          </div>

          {showBoxParams && (
            <div className="section">
              <h3>Dimensions</h3>
              <Field label="Width (m)" value={building.width} min={1} max={200} step={1}
                onChange={(v) => setBuilding({ width: v })} />
              <Field label="Depth (m)" value={building.depth} min={1} max={200} step={1}
                onChange={(v) => setBuilding({ depth: v })} />
              <Field label="Height (m)" value={building.height} min={1} max={100} step={0.5}
                onChange={(v) => setBuilding({ height: v })} />
              <SliderField label="Heading" value={building.heading_deg} min={0} max={360} step={5}
                format={(v) => `${v}\u00B0`}
                onChange={(v) => setBuilding({ heading_deg: v })} />
              <div className="field">
                <label>Roof</label>
                <select value={building.roof_type}
                  onChange={(e) => setBuilding({ roof_type: e.target.value as 'flat' | 'pitched' })}>
                  <option value="flat">Flat</option>
                  <option value="pitched">Pitched</option>
                </select>
              </div>
              {building.roof_type === 'pitched' && (
                <SliderField label="Pitch" value={building.roof_pitch_deg} min={5} max={60} step={5}
                  format={(v) => `${v}\u00B0`}
                  onChange={(v) => setBuilding({ roof_pitch_deg: v })} />
              )}
              <Field label="Lat" value={building.lat} step={0.0001}
                onChange={(v) => setBuilding({ lat: v })} />
              <Field label="Lon" value={building.lon} step={0.0001}
                onChange={(v) => setBuilding({ lon: v })} />
            </div>
          )}
        </>
      )}

      {/* ---- Mission parameters (always shown) ---- */}
      <div className="section">
        <h3>Inspection</h3>
        <SliderField label="GSD (mm/px)" value={mission.target_gsd_mm_per_px}
          min={1} max={5} step={0.25}
          onChange={(v) => setMission({ target_gsd_mm_per_px: v })} />
        <div className="field">
          <label>Camera</label>
          <select value={mission.camera}
            onChange={(e) => setMission({ camera: e.target.value as 'wide' | 'medium_tele' | 'telephoto' })}>
            {Object.entries(CAMERA_LABELS).map(([k, v]) => (
              <option key={k} value={k}>{v}</option>
            ))}
          </select>
        </div>
        <SliderField label="Front overlap" value={mission.front_overlap}
          min={0.5} max={0.95} step={0.05}
          format={(v) => `${Math.round(v * 100)}%`}
          onChange={(v) => setMission({ front_overlap: v })} />
        <SliderField label="Side overlap" value={mission.side_overlap}
          min={0.5} max={0.95} step={0.05}
          format={(v) => `${Math.round(v * 100)}%`}
          onChange={(v) => setMission({ side_overlap: v })} />
      </div>

      <div className="section">
        <h3>Flight</h3>
        <SliderField label="Inspect speed" value={mission.flight_speed_ms}
          min={1} max={8} step={0.5}
          format={(v) => `${v} m/s`}
          onChange={(v) => setMission({ flight_speed_ms: v })} />
        <SliderField label="Clearance" value={mission.obstacle_clearance_m}
          min={1} max={10} step={0.5}
          format={(v) => `${v} m`}
          onChange={(v) => setMission({ obstacle_clearance_m: v })} />
      </div>

      {/* ---- Generate + results ---- */}
      <div className="section">
        <button className="btn-primary" onClick={generate} disabled={loading || (isUploadMode && !selectedBuildingId)}>
          {loading ? 'Generating...' : 'Generate Mission'}
        </button>
        {isUploadMode && !selectedBuildingId && (
          <div className="upload-hint-msg">Upload and select a building first</div>
        )}
        {result && (
          <a className="btn-secondary" href={kmzDownloadUrl(result.version_id)} download>
            Download KMZ
          </a>
        )}
      </div>

      <div className="section">
        <h3>History</h3>
        <VersionList />
      </div>
    </aside>
  );
}

// --- Field components ---

function Field({ label, value, min, max, step, onChange }: {
  label: string; value: number; min?: number; max?: number; step?: number;
  onChange: (v: number) => void;
}) {
  return (
    <div className="field">
      <label>{label}</label>
      <input type="number" value={value} min={min} max={max} step={step}
        onChange={(e) => onChange(parseFloat(e.target.value) || 0)} />
    </div>
  );
}

function SliderField({ label, value, min, max, step, format, onChange }: {
  label: string; value: number; min: number; max: number; step: number;
  format?: (v: number) => string;
  onChange: (v: number) => void;
}) {
  return (
    <div className="field">
      <label>{label}</label>
      <input type="range" value={value} min={min} max={max} step={step}
        onChange={(e) => onChange(parseFloat(e.target.value))} />
      <span className="val">{format ? format(value) : value}</span>
    </div>
  );
}
