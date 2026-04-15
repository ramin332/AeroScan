import { useEffect, useRef } from 'react';
import { useStore, PRESETS, DEFAULT_ALGORITHM, DEFAULT_MISSION } from '../store';
import { kmzDownloadUrl } from '../api/client';
import type { AlgorithmParams, ExclusionZone } from '../api/types';
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
    buildingSource, selectedBuildingId, buildings, uploading, uploadProgress, uploadMessage, minFacadeArea, extractionMethod, waypointStrategy,
    preset, building, mission, algorithm, result, lightMode, setLightMode,
    disabledFacades, exclusionZones, toggleFacade, removeExclusionZone, updateExclusionZone,
    setBuildingSource, setPreset, setBuilding, setMission, setAlgorithm, resetAlgorithm, setMinFacadeArea, setExtractionMethod, setWaypointStrategy,
    uploadBuilding, selectBuilding, deleteBuilding,
    simStatus, simProgress, simMessage, startSimulation,
  } = useStore();

  const fileRef = useRef<HTMLInputElement>(null);

  const isUploadMode = buildingSource === 'upload';
  const autoGen = () => {
    const s = useStore.getState();
    const ok = s.buildingSource !== 'upload' || !!s.selectedBuildingId;
    if (ok) s.generate();
  };

  // Generate on initial mount with default params
  useEffect(() => { autoGen(); }, []); // eslint-disable-line react-hooks/exhaustive-deps

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

  const resetAll = () => {
    resetAlgorithm();
    setMission({
      gimbal_pitch_margin_deg: DEFAULT_MISSION.gimbal_pitch_margin_deg,
      min_photo_distance_m: DEFAULT_MISSION.min_photo_distance_m,
      yaw_rate_deg_per_s: DEFAULT_MISSION.yaw_rate_deg_per_s,
    });
    autoGen();
  };

  const selectedBuilding = buildings.find((b) => b.id === selectedBuildingId);
  const showBoxParams = !isUploadMode && preset !== 'l_shaped_block';

  return (
    <aside className="sidebar">
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', paddingRight: 12, borderBottom: '1px solid var(--border)' }}>
        <h1>AeroScan Flight Planner</h1>
        <div style={{ display: 'flex', gap: 4 }}>
          <button
            className="theme-toggle"
            onClick={() => {
              setPreset('simple_box');
              setMission(DEFAULT_MISSION);
              resetAlgorithm();
            }}
            title="Reset all values to defaults"
          >
            {'\u21BA'}
          </button>
          <button
            className="theme-toggle"
            onClick={() => setLightMode(!lightMode)}
            title={lightMode ? 'Switch to dark mode' : 'Switch to light mode'}
          >
            {lightMode ? '\u263E' : '\u2600'}
          </button>
        </div>
      </div>
      <DroneInfo />

      {/* ======== BUILDING ======== */}
      <div className="section">
        <h3>Building</h3>
        <div className="source-toggle">
          <button className={`source-btn ${isUploadMode ? 'active' : ''}`}
            onClick={() => { setBuildingSource('upload'); autoGen(); }}>Upload</button>
          <button className={`source-btn ${!isUploadMode ? 'active' : ''}`}
            onClick={() => { setBuildingSource('preset'); setTimeout(autoGen); }}>Preset</button>
        </div>
      </div>

      {/* Upload mode */}
      {isUploadMode && (
        <div className="section">
          <div className="upload-zone" onClick={() => fileRef.current?.click()}
            onDrop={handleDrop} onDragOver={handleDragOver}>
            <input ref={fileRef} type="file" accept=".json,.geojson,.obj,.ply,.stl,.glb,.gltf"
              onChange={handleFileChange} style={{ display: 'none' }} />
            {uploading ? (
              <div className="upload-progress-info">
                <div className="upload-progress-bar" style={{ width: `${Math.round(uploadProgress * 100)}%` }} />
                <span className="upload-text">{uploadMessage || 'Processing…'}</span>
                <span className="upload-hint">{Math.round(uploadProgress * 100)}%</span>
              </div>
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
                <div key={b.id}
                  className={`building-item ${selectedBuildingId === b.id ? 'active' : ''}`}
                  onClick={() => { selectBuilding(b.id); autoGen(); }}>
                  <div className="building-item-info">
                    <span className="building-name">{b.name}</span>
                    <span className="building-meta">
                      {b.lat.toFixed(4)}, {b.lon.toFixed(4)} &middot; {b.height}m
                    </span>
                  </div>
                  <span className="del" onClick={(e) => { e.stopPropagation(); deleteBuilding(b.id); }}>x</span>
                </div>
              ))}
            </div>
          )}
          {selectedBuilding && (
            <div className="building-details">
              <div className="building-dims">
                {selectedBuilding.width > 0
                  ? `${selectedBuilding.width} \u00D7 ${selectedBuilding.depth} \u00D7 ${selectedBuilding.height} m`
                  : `${selectedBuilding.height} m tall`}
              </div>
              <div className="building-source">{selectedBuilding.source_type}</div>
              <Field label="Lat" value={building.lat} step={0.0001}
                onChange={(v) => setBuilding({ lat: v })} onCommit={autoGen} />
              <Field label="Lon" value={building.lon} step={0.0001}
                onChange={(v) => setBuilding({ lon: v })} onCommit={autoGen} />
            </div>
          )}
        </div>
      )}

      {/* Preset mode */}
      {!isUploadMode && (
        <>
          <div className="section">
            <select value={preset || ''}
              onChange={(e) => { setPreset(e.target.value || null); autoGen(); }}>
              <option value="">Custom building</option>
              {Object.keys(PRESETS).map((k) => (
                <option key={k} value={k}>{PRESET_LABELS[k] || k}</option>
              ))}
            </select>
          </div>
          {showBoxParams && (
            <Section title="Dimensions" defaultOpen>
              <Field label="Width (m)" value={building.width} min={1} max={200} step={1}
                onChange={(v) => setBuilding({ width: v })} onCommit={autoGen} />
              <Field label="Depth (m)" value={building.depth} min={1} max={200} step={1}
                onChange={(v) => setBuilding({ depth: v })} onCommit={autoGen} />
              <Field label="Height (m)" value={building.height} min={1} max={100} step={0.5}
                onChange={(v) => setBuilding({ height: v })} onCommit={autoGen} />
              <SliderField label="Heading" value={building.heading_deg} min={0} max={360} step={5}
                format={(v) => `${v}\u00B0`}
                onChange={(v) => setBuilding({ heading_deg: v })} onCommit={autoGen} />
              <div className="field">
                <label>Roof</label>
                <select value={building.roof_type}
                  onChange={(e) => { setBuilding({ roof_type: e.target.value as 'flat' | 'pitched' }); autoGen(); }}>
                  <option value="flat">Flat</option>
                  <option value="pitched">Pitched</option>
                </select>
              </div>
              {building.roof_type === 'pitched' && (
                <SliderField label="Pitch" value={building.roof_pitch_deg} min={5} max={60} step={5}
                  format={(v) => `${v}\u00B0`}
                  onChange={(v) => setBuilding({ roof_pitch_deg: v })} onCommit={autoGen} />
              )}
              <Field label="Lat" value={building.lat} step={0.0001}
                onChange={(v) => setBuilding({ lat: v })} onCommit={autoGen} />
              <Field label="Lon" value={building.lon} step={0.0001}
                onChange={(v) => setBuilding({ lon: v })} onCommit={autoGen} />
            </Section>
          )}
        </>
      )}

      {/* Facade detection (upload only) */}
      {isUploadMode && (
        <Section title="Facade Detection" defaultOpen>
          <div className="field">
            <label>Waypoint Strategy</label>
            <select value={waypointStrategy}
              onChange={(e) => { setWaypointStrategy(e.target.value); autoGen(); }}>
              <option value="facade_grid">Facade Grid</option>
              <option value="surface_sampling">Surface Sampling</option>
            </select>
          </div>
          <div className="field">
            <label>Method</label>
            <select value={extractionMethod}
              onChange={(e) => { setExtractionMethod(e.target.value); autoGen(); }}>
              <option value="region_growing">Region Growing</option>
              <option value="meshlab">MeshLab</option>
              <option value="convex_hull">Convex Hull</option>
            </select>
          </div>
          <SliderField label="Min surface" value={minFacadeArea}
            min={0.5} max={10} step={0.5}
            format={(v) => `${v} m\u00B2`}
            onChange={(v) => setMinFacadeArea(v)} onCommit={autoGen} />
          {waypointStrategy === 'surface_sampling' && (
            <>
              <SliderField label="Sample density" value={algorithm.surface_sample_count}
                min={500} max={10000} step={500}
                format={(v) => `${v}`}
                tooltip="Target number of Poisson-disk sample points on the mesh surface. Higher = denser coverage."
                defaultValue={DEFAULT_ALGORITHM.surface_sample_count}
                onReset={() => setAlgorithm({ surface_sample_count: DEFAULT_ALGORITHM.surface_sample_count })}
                onChange={(v) => setAlgorithm({ surface_sample_count: v })} onCommit={autoGen} />
              <SliderField label="Dedup radius" value={algorithm.surface_dedup_radius_m}
                min={0.1} max={3} step={0.1}
                format={(v) => `${v}m`}
                tooltip="Merge cameras closer than this distance with similar headings."
                defaultValue={DEFAULT_ALGORITHM.surface_dedup_radius_m}
                onReset={() => setAlgorithm({ surface_dedup_radius_m: DEFAULT_ALGORITHM.surface_dedup_radius_m })}
                onChange={(v) => setAlgorithm({ surface_dedup_radius_m: v })} onCommit={autoGen} />
              <SliderField label="Dedup angle" value={algorithm.surface_dedup_max_angle_deg}
                min={5} max={90} step={5}
                format={(v) => `${v}\u00B0`}
                tooltip="Max heading difference for two nearby cameras to be merged."
                defaultValue={DEFAULT_ALGORITHM.surface_dedup_max_angle_deg}
                onReset={() => setAlgorithm({ surface_dedup_max_angle_deg: DEFAULT_ALGORITHM.surface_dedup_max_angle_deg })}
                onChange={(v) => setAlgorithm({ surface_dedup_max_angle_deg: v })} onCommit={autoGen} />
            </>
          )}
        </Section>
      )}

      {/* ======== INSPECTION ======== */}
      <Section title="Inspection" defaultOpen>
        <SliderField label="GSD (mm/px)" value={mission.target_gsd_mm_per_px}
          min={1} max={5} step={0.25}
          onChange={(v) => setMission({ target_gsd_mm_per_px: v })} onCommit={autoGen} />
        <div className="field">
          <label>Camera</label>
          <select value={mission.camera}
            onChange={(e) => { setMission({ camera: e.target.value as 'wide' | 'medium_tele' | 'telephoto' }); autoGen(); }}>
            {Object.entries(CAMERA_LABELS).map(([k, v]) => (
              <option key={k} value={k}>{v}</option>
            ))}
          </select>
        </div>
        <SliderField label="Front overlap" value={mission.front_overlap}
          min={0.6} max={0.95} step={0.05}
          format={(v) => `${Math.round(v * 100)}%`}
          onChange={(v) => setMission({ front_overlap: v })} onCommit={autoGen} />
        <SliderField label="Side overlap" value={mission.side_overlap}
          min={0.5} max={0.9} step={0.05}
          format={(v) => `${Math.round(v * 100)}%`}
          onChange={(v) => setMission({ side_overlap: v })} onCommit={autoGen} />
      </Section>

      {/* ======== FLIGHT ======== */}
      <Section title="Flight" defaultOpen>
        <SliderField label="Inspect speed" value={mission.flight_speed_ms}
          min={0.5} max={5} step={0.5}
          format={(v) => `${v} m/s`}
          onChange={(v) => setMission({ flight_speed_ms: v })} onCommit={autoGen} />
        <ToggleField label="Stop at waypoints"
          value={mission.stop_at_waypoint}
          tooltip="Off = fly-through (faster, M4E mech shutter prevents blur). On = stop at each waypoint (slower but guaranteed sharp)."
          onChange={(v) => { setMission({ stop_at_waypoint: v }); autoGen(); }} />
        <SliderField label="Clearance" value={mission.obstacle_clearance_m}
          min={1} max={10} step={0.5}
          format={(v) => `${v} m`}
          onChange={(v) => setMission({ obstacle_clearance_m: v })} onCommit={autoGen} />
        {result && (
          <div className="field-hint">
            Camera at {result.summary.camera_distance_m}m (from GSD).
            {mission.obstacle_clearance_m < result.summary.camera_distance_m
              ? ' Clearance has no effect at this distance.'
              : ''}
          </div>
        )}
      </Section>

      {/* ======== PATH OPTIMIZATION ======== */}
      <Section title="Path Optimization" defaultOpen>
        <SliderField label="Grid density" value={algorithm.grid_density}
          min={0.25} max={4} step={0.25}
          format={(v) => `${v}x`}
          tooltip="Photo point density multiplier. 1x = standard from GSD/overlap. 2x = double points. 0.5x = half points (faster flight)."
          defaultValue={DEFAULT_ALGORITHM.grid_density}
          onReset={() => setAlgorithm({ grid_density: DEFAULT_ALGORITHM.grid_density })}
          onChange={(v) => setAlgorithm({ grid_density: v })} onCommit={autoGen} />
        <ToggleField label="TSP ordering"
          value={algorithm.enable_path_tsp}
          tooltip="Optimize facade visit order to minimize transit distance between facades."
          onChange={(v) => { setAlgorithm({ enable_path_tsp: v }); autoGen(); }} />
        {algorithm.enable_path_tsp && (
          <div className="field">
            <label>TSP method</label>
            <select value={algorithm.tsp_method}
              onChange={(e) => { setAlgorithm({ tsp_method: e.target.value as AlgorithmParams['tsp_method'] }); autoGen(); }}>
              <option value="auto">Auto (best of all)</option>
              <option value="nearest_neighbor">Nearest Neighbor</option>
              <option value="greedy">Greedy</option>
              <option value="simulated_annealing">Simulated Annealing</option>
              <option value="threshold_accepting">Threshold Accepting</option>
            </select>
          </div>
        )}
        <ToggleField label="Sweep reversal"
          value={algorithm.enable_sweep_reversal}
          tooltip="For each facade, choose forward or reversed boustrophedon direction to minimize entry distance from previous facade's exit."
          onChange={(v) => { setAlgorithm({ enable_sweep_reversal: v }); autoGen(); }} />
        <ToggleField label="Cross-facade dedup"
          value={algorithm.enable_path_dedup}
          tooltip="Merge near-coincident waypoints from adjacent facades. Only merges when gimbal angles are similar (NEN-2767 perpendicular-to-surface preserved)."
          onChange={(v) => { setAlgorithm({ enable_path_dedup: v }); autoGen(); }} />
        {algorithm.enable_path_dedup && (
          <>
            <SliderField label="Merge radius" value={mission.min_photo_distance_m}
              min={0.5} max={5} step={0.5}
              format={(v) => `${v}m`}
              tooltip="Waypoints within this distance from different facades are merge candidates."
              defaultValue={DEFAULT_MISSION.min_photo_distance_m}
              onReset={() => setMission({ min_photo_distance_m: DEFAULT_MISSION.min_photo_distance_m })}
              onChange={(v) => setMission({ min_photo_distance_m: v })} onCommit={autoGen} />
            <SliderField label="Gimbal tolerance" value={algorithm.dedup_max_gimbal_diff_deg}
              min={5} max={45} step={5}
              format={(v) => `${v}\u00B0`}
              tooltip="Max gimbal angle difference for merge eligibility. Lower = stricter. Must respect NEN-2767 perpendicular-to-surface."
              defaultValue={DEFAULT_ALGORITHM.dedup_max_gimbal_diff_deg}
              onReset={() => setAlgorithm({ dedup_max_gimbal_diff_deg: DEFAULT_ALGORITHM.dedup_max_gimbal_diff_deg })}
              onChange={(v) => setAlgorithm({ dedup_max_gimbal_diff_deg: v })} onCommit={autoGen} />
          </>
        )}
      </Section>

      {/* ======== WAYPOINT LOS ======== */}
      <Section title="Line-of-Sight" defaultOpen>
        <ToggleField label="LOS occlusion check"
          value={algorithm.enable_waypoint_los}
          tooltip="Ray-cast from each waypoint to the facade surface through the mesh. Skips waypoints where the building blocks the camera view."
          onChange={(v) => { setAlgorithm({ enable_waypoint_los: v }); autoGen(); }} />
        {algorithm.enable_waypoint_los && (
          <>
            <SliderField label="Hit tolerance" value={algorithm.los_tolerance_m}
              min={0.1} max={2} step={0.1}
              format={(v) => `${v}m`}
              tooltip="A ray hit closer than (target distance - tolerance) counts as occluded. Increase if mesh doesn't align perfectly with facade planes."
              defaultValue={DEFAULT_ALGORITHM.los_tolerance_m}
              onReset={() => setAlgorithm({ los_tolerance_m: DEFAULT_ALGORITHM.los_tolerance_m })}
              onChange={(v) => setAlgorithm({ los_tolerance_m: v })} onCommit={autoGen} />
            <SliderField label="Min visible ratio" value={algorithm.los_min_visible_ratio}
              min={0.1} max={1} step={0.1}
              format={(v) => `${Math.round(v * 100)}%`}
              tooltip="Minimum fraction of 5 sample rays that must reach the facade. Lower = keep more waypoints, higher = stricter occlusion filtering."
              defaultValue={DEFAULT_ALGORITHM.los_min_visible_ratio}
              onReset={() => setAlgorithm({ los_min_visible_ratio: DEFAULT_ALGORITHM.los_min_visible_ratio })}
              onChange={(v) => setAlgorithm({ los_min_visible_ratio: v })} onCommit={autoGen} />
          </>
        )}
      </Section>

      {/* ======== PATH COLLISION ======== */}
      <Section title="Path Collision" defaultOpen>
        <ToggleField label="Path collision check"
          value={algorithm.enable_path_collision_check}
          tooltip="Check flight path segments between waypoints for building collisions. Inserts detour waypoints to route around obstacles."
          onChange={(v) => { setAlgorithm({ enable_path_collision_check: v }); autoGen(); }} />
        {algorithm.enable_path_collision_check && (
          <SliderField label="Collision margin" value={algorithm.path_collision_margin_m}
            min={0.1} max={3} step={0.1}
            format={(v) => `${v}m`}
            tooltip="Buffer distance for segment collision test. A hit within this distance of a waypoint is ignored (the drone is already near the surface for inspection)."
            defaultValue={DEFAULT_ALGORITHM.path_collision_margin_m}
            onReset={() => setAlgorithm({ path_collision_margin_m: DEFAULT_ALGORITHM.path_collision_margin_m })}
            onChange={(v) => setAlgorithm({ path_collision_margin_m: v })} onCommit={autoGen} />
        )}
      </Section>

      {/* ======== SAFETY ======== */}
      <Section title="Safety">
        <SliderField label="Gimbal pitch margin" value={mission.gimbal_pitch_margin_deg}
          min={0} max={15} step={1}
          format={(v) => `${v}\u00B0`}
          tooltip="Safety margin from hardware pitch limits (-90\u00B0/+35\u00B0)."
          defaultValue={DEFAULT_MISSION.gimbal_pitch_margin_deg}
          onReset={() => setMission({ gimbal_pitch_margin_deg: DEFAULT_MISSION.gimbal_pitch_margin_deg })}
          onChange={(v) => setMission({ gimbal_pitch_margin_deg: v })} onCommit={autoGen} />
        <SliderField label="Min altitude" value={algorithm.min_altitude_m}
          min={0.5} max={10} step={0.5}
          format={(v) => `${v}m`}
          tooltip="Safety floor. Waypoints below this height are clamped up."
          defaultValue={DEFAULT_ALGORITHM.min_altitude_m}
          onReset={() => setAlgorithm({ min_altitude_m: DEFAULT_ALGORITHM.min_altitude_m })}
          onChange={(v) => setAlgorithm({ min_altitude_m: v })} onCommit={autoGen} />
        <SliderField label="Min KMZ height" value={algorithm.min_waypoint_height_m}
          min={0.5} max={10} step={0.5}
          format={(v) => `${v}m`}
          tooltip="Minimum waypoint height in exported KMZ. DJI Pilot 2 safety clamp."
          defaultValue={DEFAULT_ALGORITHM.min_waypoint_height_m}
          onReset={() => setAlgorithm({ min_waypoint_height_m: DEFAULT_ALGORITHM.min_waypoint_height_m })}
          onChange={(v) => setAlgorithm({ min_waypoint_height_m: v })} onCommit={autoGen} />
        <SliderField label="Edge inset" value={algorithm.facade_edge_inset_m}
          min={0} max={1} step={0.05}
          format={(v) => `${v}m`}
          tooltip="Margin from facade edges for waypoint placement."
          defaultValue={DEFAULT_ALGORITHM.facade_edge_inset_m}
          onReset={() => setAlgorithm({ facade_edge_inset_m: DEFAULT_ALGORITHM.facade_edge_inset_m })}
          onChange={(v) => setAlgorithm({ facade_edge_inset_m: v })} onCommit={autoGen} />
      </Section>

      {/* ======== FLIGHT TIME ESTIMATION ======== */}
      <Section title="Flight Time">
        <SliderField label="Hover per WP" value={algorithm.hover_time_per_wp_s}
          min={0} max={5} step={0.5}
          format={(v) => `${v}s`}
          tooltip="Assumed hover time per waypoint for photo capture."
          defaultValue={DEFAULT_ALGORITHM.hover_time_per_wp_s}
          onReset={() => setAlgorithm({ hover_time_per_wp_s: DEFAULT_ALGORITHM.hover_time_per_wp_s })}
          onChange={(v) => setAlgorithm({ hover_time_per_wp_s: v })} onCommit={autoGen} />
        <SliderField label="T/O overhead" value={algorithm.takeoff_landing_overhead_s}
          min={0} max={180} step={10}
          format={(v) => `${v}s`}
          tooltip="Fixed time for takeoff/landing sequence."
          defaultValue={DEFAULT_ALGORITHM.takeoff_landing_overhead_s}
          onReset={() => setAlgorithm({ takeoff_landing_overhead_s: DEFAULT_ALGORITHM.takeoff_landing_overhead_s })}
          onChange={(v) => setAlgorithm({ takeoff_landing_overhead_s: v })} onCommit={autoGen} />
        <SliderField label="Yaw rate" value={mission.yaw_rate_deg_per_s}
          min={30} max={120} step={10}
          format={(v) => `${v}\u00B0/s`}
          tooltip="Assumed yaw rotation speed for time estimates."
          defaultValue={DEFAULT_MISSION.yaw_rate_deg_per_s}
          onReset={() => setMission({ yaw_rate_deg_per_s: DEFAULT_MISSION.yaw_rate_deg_per_s })}
          onChange={(v) => setMission({ yaw_rate_deg_per_s: v })} onCommit={autoGen} />
        <SliderField label="Batt warn" value={algorithm.battery_warning_threshold}
          min={0.5} max={0.99} step={0.05}
          format={(v) => `${Math.round(v * 100)}%`}
          tooltip="Flight time fraction that triggers RTH warning."
          defaultValue={DEFAULT_ALGORITHM.battery_warning_threshold}
          onReset={() => setAlgorithm({ battery_warning_threshold: DEFAULT_ALGORITHM.battery_warning_threshold })}
          onChange={(v) => setAlgorithm({ battery_warning_threshold: v })} onCommit={autoGen} />
        <SliderField label="Batt info" value={algorithm.battery_info_threshold}
          min={0.3} max={0.95} step={0.05}
          format={(v) => `${Math.round(v * 100)}%`}
          tooltip="Flight time fraction that triggers info message."
          defaultValue={DEFAULT_ALGORITHM.battery_info_threshold}
          onReset={() => setAlgorithm({ battery_info_threshold: DEFAULT_ALGORITHM.battery_info_threshold })}
          onChange={(v) => setAlgorithm({ battery_info_threshold: v })} onCommit={autoGen} />
        <SliderField label="Gimbal warn" value={algorithm.gimbal_near_limit_deg}
          min={-90} max={0} step={1}
          format={(v) => `${v}\u00B0`}
          tooltip="Pitch below this triggers 'near nadir' info."
          defaultValue={DEFAULT_ALGORITHM.gimbal_near_limit_deg}
          onReset={() => setAlgorithm({ gimbal_near_limit_deg: DEFAULT_ALGORITHM.gimbal_near_limit_deg })}
          onChange={(v) => setAlgorithm({ gimbal_near_limit_deg: v })} onCommit={autoGen} />
      </Section>

      {/* ======== GEOMETRY (advanced) ======== */}
      <Section title="Geometry">
        <SliderField label="Transit margin" value={algorithm.transition_altitude_margin_m}
          min={0} max={10} step={0.5}
          format={(v) => `${v}m`}
          tooltip="Extra altitude during facade-to-facade transitions."
          defaultValue={DEFAULT_ALGORITHM.transition_altitude_margin_m}
          onReset={() => setAlgorithm({ transition_altitude_margin_m: DEFAULT_ALGORITHM.transition_altitude_margin_m })}
          onChange={(v) => setAlgorithm({ transition_altitude_margin_m: v })} onCommit={autoGen} />
        <SliderField label="Roof threshold" value={algorithm.roof_normal_threshold}
          min={0.1} max={0.9} step={0.05}
          format={(v) => `${v}`}
          tooltip="Normal Z above this = roof surface."
          defaultValue={DEFAULT_ALGORITHM.roof_normal_threshold}
          onReset={() => setAlgorithm({ roof_normal_threshold: DEFAULT_ALGORITHM.roof_normal_threshold })}
          onChange={(v) => setAlgorithm({ roof_normal_threshold: v })} onCommit={autoGen} />
      </Section>

      {/* ======== MESH IMPORT (upload only) ======== */}
      {isUploadMode && (
        <Section title="Mesh Import">
          {result?.perf?.extraction && (
            <div className="field-hint" style={{ marginBottom: 6, color: 'var(--accent2)' }}>
              {result.perf.extraction.facades_extracted} facades extracted ({result.perf.extraction.walls}W {result.perf.extraction.roofs}R) from {result.perf.extraction.regions_found} regions
            </div>
          )}
          <SliderField label="Default height" value={algorithm.default_building_height_m}
            min={1} max={100} step={1}
            format={(v) => `${v}m`}
            tooltip="Fallback height when auto-scaling kicks in."
            defaultValue={DEFAULT_ALGORITHM.default_building_height_m}
            onReset={() => setAlgorithm({ default_building_height_m: DEFAULT_ALGORITHM.default_building_height_m })}
            onChange={(v) => setAlgorithm({ default_building_height_m: v })} onCommit={autoGen} />
          <SliderField label="Region angle" value={algorithm.region_growing_angle_deg}
            min={1} max={45} step={1}
            format={(v) => `${v}\u00B0`}
            tooltip="Max angle between adjacent normals for region growing."
            defaultValue={DEFAULT_ALGORITHM.region_growing_angle_deg}
            onReset={() => setAlgorithm({ region_growing_angle_deg: DEFAULT_ALGORITHM.region_growing_angle_deg })}
            onChange={(v) => setAlgorithm({ region_growing_angle_deg: v })} onCommit={autoGen} />
          <SliderField label="Wall nz" value={algorithm.wall_normal_threshold}
            min={0.01} max={0.9} step={0.05}
            format={(v) => `${v}`}
            tooltip="Normal Z below this = wall."
            defaultValue={DEFAULT_ALGORITHM.wall_normal_threshold}
            onReset={() => setAlgorithm({ wall_normal_threshold: DEFAULT_ALGORITHM.wall_normal_threshold })}
            onChange={(v) => setAlgorithm({ wall_normal_threshold: v })} onCommit={autoGen} />
          <SliderField label="Flat roof nz" value={algorithm.flat_roof_normal_threshold}
            min={0.5} max={1} step={0.05}
            format={(v) => `${v}`}
            tooltip="Normal Z above this = flat roof."
            defaultValue={DEFAULT_ALGORITHM.flat_roof_normal_threshold}
            onReset={() => setAlgorithm({ flat_roof_normal_threshold: DEFAULT_ALGORITHM.flat_roof_normal_threshold })}
            onChange={(v) => setAlgorithm({ flat_roof_normal_threshold: v })} onCommit={autoGen} />
          <SliderField label="Occlusion frac" value={algorithm.occlusion_hit_fraction}
            min={0.1} max={1} step={0.1}
            format={(v) => `${v}`}
            tooltip="Interior wall detection threshold."
            defaultValue={DEFAULT_ALGORITHM.occlusion_hit_fraction}
            onReset={() => setAlgorithm({ occlusion_hit_fraction: DEFAULT_ALGORITHM.occlusion_hit_fraction })}
            onChange={(v) => setAlgorithm({ occlusion_hit_fraction: v })} onCommit={autoGen} />
          <SliderField label="Ground filter" value={algorithm.ground_level_threshold_m}
            min={0} max={5} step={0.1}
            format={(v) => `${v}m`}
            tooltip="Surfaces below this Z are filtered as ground."
            defaultValue={DEFAULT_ALGORITHM.ground_level_threshold_m}
            onReset={() => setAlgorithm({ ground_level_threshold_m: DEFAULT_ALGORITHM.ground_level_threshold_m })}
            onChange={(v) => setAlgorithm({ ground_level_threshold_m: v })} onCommit={autoGen} />
          <SliderField label="Auto-scale" value={algorithm.auto_scale_height_threshold_m}
            min={10} max={500} step={10}
            format={(v) => `>${v}m`}
            tooltip="Mesh height above this triggers auto-rescale."
            defaultValue={DEFAULT_ALGORITHM.auto_scale_height_threshold_m}
            onReset={() => setAlgorithm({ auto_scale_height_threshold_m: DEFAULT_ALGORITHM.auto_scale_height_threshold_m })}
            onChange={(v) => setAlgorithm({ auto_scale_height_threshold_m: v })} onCommit={autoGen} />
        </Section>
      )}

      {/* ======== RESET ======== */}
      <div className="section" style={{ paddingTop: 0 }}>
        <button className="btn-secondary" style={{ fontSize: 11, padding: '3px 10px', opacity: 0.7 }}
          onClick={resetAll}>
          Reset all to defaults
        </button>
      </div>

      {/* ======== RESULTS ======== */}
      <div className="section">
        {result && result.can_export && (
          <a className="btn-secondary" href={kmzDownloadUrl(result.version_id)} download>
            Download KMZ
          </a>
        )}
        {result && !result.can_export && (
          <div className="validation-error">Cannot export — fix errors first</div>
        )}
        {result && result.validation && result.validation.length > 0 && (
          <div className="validation-list">
            {result.validation.map((v, i) => (
              <div key={i} className={`validation-item ${v.severity}`}>
                <span className="validation-icon">
                  {v.severity === 'error' ? '\u2716' : v.severity === 'warning' ? '\u26A0' : '\u2139'}
                </span>
                <span>{v.message}</span>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* ======== SIMULATE RECONSTRUCTION ======== */}
      {result && result.can_export && (
        <Section title="Simulate Reconstruction" defaultOpen={false}>
          {simStatus && simStatus !== 'complete' && simStatus !== 'error' ? (
            /* Active run — show progress stepper */
            <div>
              {(() => {
                const steps = [
                  { key: 'rendering', label: 'Rendering photos + TSDF fusion' },
                  { key: 'importing', label: 'Decimating & importing mesh' },
                  { key: 'generating', label: 'Generating new mission' },
                ];
                const currentIdx = steps.findIndex(s => s.key === simStatus);
                return (
                  <div style={{ display: 'flex', flexDirection: 'column', gap: 3, fontSize: 11 }}>
                    {steps.map((step, i) => {
                      const done = i < currentIdx;
                      const active = i === currentIdx;
                      return (
                        <div key={step.key} style={{ display: 'flex', alignItems: 'center', gap: 6, color: done ? 'var(--accent2)' : active ? 'var(--text-primary)' : 'var(--text-secondary)', opacity: done || active ? 1 : 0.4 }}>
                          <span style={{ width: 14, textAlign: 'center', fontSize: 10 }}>
                            {done ? '\u2713' : active ? '\u25CF' : '\u25CB'}
                          </span>
                          <span>{step.label}{active && simStatus === 'rendering' && simProgress > 0.1 ? ` (${Math.round((simProgress - 0.1) / 0.6 * 100)}%)` : ''}</span>
                        </div>
                      );
                    })}
                  </div>
                );
              })()}
              <div style={{ height: 3, background: 'var(--bg-secondary)', borderRadius: 2, overflow: 'hidden', marginTop: 8 }}>
                <div style={{
                  height: '100%',
                  width: `${Math.round(simProgress * 100)}%`,
                  background: 'var(--accent)',
                  transition: 'width 0.3s ease',
                }} />
              </div>
            </div>
          ) : (
            /* Ready — show quality presets */
            <div>
              <div className="field-hint" style={{ marginBottom: 8 }}>
                Render photos, reconstruct via TSDF, reimport. View results in Simulation tab.
              </div>
              {simStatus === 'error' && (
                <div className="validation-error" style={{ marginBottom: 6 }}>{simMessage}</div>
              )}
              <div style={{ display: 'flex', gap: 4 }}>
                {([
                  ['Super Fast', 0.05, 0.08],
                  ['Fast',       0.08, 0.05],
                  ['Medium',     0.12, 0.03],
                  ['High',       0.20, 0.02],
                ] as const).map(([label, scale, voxel]) => (
                  <button key={label} className="btn-primary"
                    style={{ flex: 1, fontSize: 10, padding: '5px 2px' }}
                    onClick={() => startSimulation(scale, voxel)}>
                    {label}
                  </button>
                ))}
              </div>
            </div>
          )}
        </Section>
      )}

      {/* ======== FACADE TOGGLE ======== */}
      {result && result.viewer_data.threejs.facades.length > 0 && (
        <Section title="Facades" defaultOpen>
          <div className="field-hint" style={{ marginBottom: 6 }}>
            Click facades in the 3D view or toggle here
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
            {result.viewer_data.threejs.facades.map((f) => {
              const disabled = disabledFacades.has(f.index);
              return (
                <label key={f.index} style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 11, cursor: 'pointer', opacity: disabled ? 0.5 : 1 }}>
                  <input type="checkbox" checked={!disabled}
                    onChange={() => toggleFacade(f.index)} />
                  <span style={{ width: 10, height: 10, borderRadius: 2, background: f.color, flexShrink: 0 }} />
                  <span>{f.label}</span>
                </label>
              );
            })}
          </div>
          {disabledFacades.size > 0 && (
            <div className="field-hint" style={{ marginTop: 6, color: 'var(--accent2)' }}>
              {disabledFacades.size} facade(s) disabled — regenerate to apply
            </div>
          )}
        </Section>
      )}

      {/* ======== ZONES ======== */}
      {exclusionZones.length > 0 && (
        <Section title={`Zones (${exclusionZones.length})`} defaultOpen>
          <div className="field-hint" style={{ marginBottom: 6 }}>
            Draw on map tab
          </div>
          {exclusionZones.map((zone) => {
            const dotColor = zone.zone_type === 'no_fly' ? '#f44'
              : zone.zone_type === 'no_inspect' ? '#fa3'
              : '#22cc55';
            return (
              <div key={zone.id} style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 11, marginBottom: 3 }}>
                <span style={{ width: 8, height: 8, borderRadius: 2, flexShrink: 0, background: dotColor }} />
                <span style={{ flex: 1, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                  {zone.label}
                </span>
                <select value={zone.zone_type} style={{ fontSize: 10, padding: '1px 2px', width: 76 }}
                  onChange={(e) => updateExclusionZone(zone.id, { zone_type: e.target.value as ExclusionZone['zone_type'] })}>
                  <option value="inclusion">Geofence</option>
                  <option value="no_fly">No-Fly</option>
                  <option value="no_inspect">No-Insp</option>
                </select>
                <button style={{ fontSize: 10, padding: '1px 4px', cursor: 'pointer', opacity: 0.6 }}
                  onClick={() => removeExclusionZone(zone.id)}>&times;</button>
              </div>
            );
          })}
        </Section>
      )}

      <div className="section">
        <h3>History</h3>
        <VersionList />
      </div>
    </aside>
  );
}

// --- Layout components ---

function Section({ title, defaultOpen, children }: {
  title: string; defaultOpen?: boolean; children: React.ReactNode;
}) {
  const key = `section:${title}`;
  const { sectionState, setSectionOpen } = useStore();
  const isOpen = sectionState[key] ?? (defaultOpen !== false);
  return (
    <details className="section" open={isOpen}
      onToggle={(e) => setSectionOpen(key, (e.target as HTMLDetailsElement).open)}>
      <summary><h3 style={{ display: 'inline', cursor: 'pointer' }}>{title}</h3></summary>
      <div style={{ marginTop: 8 }}>{children}</div>
    </details>
  );
}

// --- Field components ---

function Field({ label, value, min, max, step, onChange, onCommit }: {
  label: string; value: number; min?: number; max?: number; step?: number;
  onChange: (v: number) => void;
  onCommit?: () => void;
}) {
  return (
    <div className="field">
      <label>{label}</label>
      <input type="number" value={value} min={min} max={max} step={step}
        onChange={(e) => onChange(parseFloat(e.target.value) || 0)}
        onBlur={onCommit}
        onKeyDown={(e) => { if (e.key === 'Enter') onCommit?.(); }} />
    </div>
  );
}

function ToggleField({ label, value, onChange, tooltip }: {
  label: string; value: boolean;
  onChange: (v: boolean) => void;
  tooltip?: string;
}) {
  return (
    <div className="field">
      <label>
        {label}
        {tooltip && (
          <span className="tooltip-wrap">
            <span className="tooltip-icon">?</span>
            <span className="tooltip-popup">{tooltip}</span>
          </span>
        )}
      </label>
      <input type="checkbox" checked={value}
        onChange={(e) => onChange(e.target.checked)}
        style={{ width: 'auto', marginLeft: 'auto' }} />
    </div>
  );
}

function SliderField({ label, value, min, max, step, format, onChange, onCommit, tooltip, defaultValue, onReset }: {
  label: string; value: number; min: number; max: number; step: number;
  format?: (v: number) => string;
  onChange: (v: number) => void;
  onCommit?: () => void;
  tooltip?: string;
  defaultValue?: number;
  onReset?: () => void;
}) {
  const isModified = defaultValue !== undefined && value !== defaultValue;
  return (
    <div className="field">
      <label>
        {label}
        {tooltip && (
          <span className="tooltip-wrap">
            <span className="tooltip-icon">?</span>
            <span className="tooltip-popup">{tooltip}</span>
          </span>
        )}
      </label>
      <input type="range" value={value} min={min} max={max} step={step}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        onPointerUp={onCommit} />
      <span className="val">{format ? format(value) : value}</span>
      {isModified && onReset && (
        <span className="reset-icon"
          onClick={() => { onReset(); onCommit?.(); }}>&#x21BA;</span>
      )}
    </div>
  );
}
