import { useEffect, useRef, useState } from 'react';
import { useStore, DEFAULT_ALGORITHM, DEFAULT_MISSION } from '../store';
import { kmzDownloadUrl } from '../api/client';
import type { AlgorithmParams, ExclusionZone, GimbalStats, GimbalDiffEntry, FacadeCoverageEntry } from '../api/types';
import { DroneInfo } from './DroneInfo';
import { VersionList } from './VersionList';

const CAMERA_LABELS: Record<string, string> = {
  wide: 'Wide 24mm',
  medium_tele: 'Medium tele 70mm',
  telephoto: 'Telephoto 168mm',
};

// Backend defaults for the CGAL Shape-Detection facade extractor
// (src/flight_planner/kmz_import.py:facades_from_pointcloud_cgal).
// Single source of truth for "what the slider shows when the store is null".
const FD_DEFAULTS = {
  epsilon: 0.05,          // m — plane-fit ε
  clusterEpsilon: 0.25,   // m — max inlier gap
  minPoints: 40,          // region seed threshold
  normalThreshold: 0.92,  // cos(θ) agreement
  minWallArea: 0.5,       // m²
  minRoofArea: 0.5,       // m²
  minDensity: 25,         // pts/m²
} as const;

export function Sidebar() {
  const {
    selectedBuildingId, uploading, uploadProgress, uploadMessage,
    mission, algorithm, result, lightMode, setLightMode,
    disabledFacades, exclusionZones, toggleFacade, removeExclusionZone, updateExclusionZone,
    setMission, setAlgorithm, resetAlgorithm,
    importKmz, optimizeKmz, cancelOptimize, triggerRefine,
    kmzOptimizing, kmzOptimizeMessage, kmzAutoRefine, setKmzAutoRefine,
    refineRunning,
    kmzOptimizeMin, kmzOptimizeMax, kmzOptimizeSteps, kmzAwAlpha, kmzAwOffset, setKmzReconParams,
    kmzFdEpsilon, kmzFdClusterEpsilon, kmzFdMinPoints,
    kmzFdMinWallArea, kmzFdMinRoofArea, kmzFdMinDensity, kmzFdNormalThreshold,
    setKmzFacadeParams,
    kmzMode, switchMode,
    buildings, refreshBuildings, selectBuilding, deleteBuilding, stripRosetteOnly,
    simStatus, simProgress, simMessage, startSimulation,
    rewriteGimbals, generateInspectionMission, toggleKmzFacades, lastKmzFile,
    triggerMissionUpdate,
  } = useStore();

  useEffect(() => { void refreshBuildings(); }, [refreshBuildings]);

  const isKmz = !!result?.summary?.source?.startsWith('kmz_');
  const inspectionMode = isKmz ? kmzMode === 'inspection' : true;

  const kmzRef = useRef<HTMLInputElement>(null);

  // Mission-param sliders (GSD, overlap, speed, camera, gimbal margin, …)
  // feed into the coalescing mission-update trigger. In inspection mode this
  // re-runs generateInspectionMission with the latest params; in DJI mode it
  // is a no-op (the DJI path comes straight from the imported KMZ).
  const autoGen = () => { triggerMissionUpdate(); };

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
  };

  const handleKmzChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      importKmz(file);
      e.target.value = '';
    }
  };

  const handleKmzDrop = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    const file = e.dataTransfer.files[0];
    if (file && file.name.toLowerCase().endsWith('.kmz')) {
      importKmz(file);
    }
  };

  const resetAll = () => {
    resetAlgorithm();
    setMission({
      gimbal_pitch_margin_deg: DEFAULT_MISSION.gimbal_pitch_margin_deg,
      min_photo_distance_m: DEFAULT_MISSION.min_photo_distance_m,
      yaw_rate_deg_per_s: DEFAULT_MISSION.yaw_rate_deg_per_s,
    });
  };

  return (
    <aside className="sidebar">
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', paddingRight: 12, borderBottom: '1px solid var(--border)' }}>
        <h1>AeroScan Flight Planner</h1>
        <div style={{ display: 'flex', gap: 4 }}>
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

      {/* DJI KMZ import — always visible */}
      <div className="section">
        <div className="upload-zone" onClick={() => kmzRef.current?.click()}
          onDrop={handleKmzDrop} onDragOver={handleDragOver}>
          <input ref={kmzRef} type="file" accept=".kmz"
            onChange={handleKmzChange} style={{ display: 'none' }} />
          {uploading ? (
            <div className="upload-progress-info">
              <div className="upload-progress-bar" style={{ width: `${Math.round(uploadProgress * 100)}%` }} />
              <span className="upload-text">{uploadMessage || 'Processing…'}</span>
              <span className="upload-hint">{Math.round(uploadProgress * 100)}%</span>
            </div>
          ) : (
            <>
              <span className="upload-icon">+</span>
              <span className="upload-text">Import DJI KMZ</span>
              <span className="upload-hint">Smart3D mission with point cloud</span>
            </>
          )}
        </div>
        {result?.version_id && lastKmzFile && (result.summary.source === 'kmz_raw' || result.summary.source === 'kmz_import') && (
          <button
            className="btn-secondary"
            style={{ marginTop: 8, width: '100%', fontSize: 11, padding: '5px 10px' }}
            onClick={() => toggleKmzFacades()}
            disabled={uploading}
            title={result.summary.source === 'kmz_raw'
              ? 'Run mesh reconstruction + facade extraction on the point cloud. Slower, but enables inspection-grid generation.'
              : 'Discard facade extraction and show only the raw point cloud + original flight plan.'}
          >
            {result.summary.source === 'kmz_raw' ? 'Detect facades' : 'Hide facades (raw view)'}
          </button>
        )}
        {result?.version_id && result.summary.facade_count > 0 &&
         (result.summary.gimbal_before || result.summary.gimbal_after) && (
          <GimbalDiff
            before={result.summary.gimbal_before}
            after={result.summary.gimbal_after}
            isRewritten={!!result.summary.parent_version_id}
            diff={result.summary.gimbal_diff}
          />
        )}
      </div>

      {/* ======== SAVED BUILDINGS (always expanded) ======== */}
      {buildings.length > 0 && (
        <div className="section" style={{ padding: 10 }}>
          <div style={{
            display: 'flex', alignItems: 'center', justifyContent: 'space-between',
            marginBottom: 8,
          }}>
            <h3 style={{ margin: 0 }}>Saved buildings</h3>
            <span style={{ fontSize: 11, color: 'var(--text-dim)' }}>
              {buildings.length}
            </span>
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
            {buildings.map((b) => {
              const isActive = selectedBuildingId === b.id;
              const modes = b.available_modes ?? [];
              return (
                <div
                  key={b.id}
                  style={{
                    display: 'flex', alignItems: 'stretch', gap: 0,
                    background: isActive ? 'rgba(59,130,246,0.18)' : 'var(--bg-elev, rgba(255,255,255,0.03))',
                    border: `1px solid ${isActive ? 'rgba(59,130,246,0.6)' : 'var(--border)'}`,
                    borderRadius: 6, overflow: 'hidden',
                  }}
                >
                  <button
                    onClick={() => void selectBuilding(b.id)}
                    disabled={isActive}
                    title={isActive ? 'Currently loaded' : `Open "${b.name}"`}
                    style={{
                      flex: 1, textAlign: 'left', background: 'transparent',
                      border: 'none', padding: '8px 10px', cursor: isActive ? 'default' : 'pointer',
                      color: 'inherit', display: 'flex', flexDirection: 'column',
                      gap: 2, minWidth: 0,
                    }}
                  >
                    <span style={{
                      fontWeight: 600, fontSize: 13,
                      overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                    }}>
                      {b.name}
                    </span>
                    <span style={{ fontSize: 10, color: 'var(--text-dim)', display: 'flex', gap: 6, alignItems: 'center' }}>
                      <span>{b.source_type}</span>
                      <span>·</span>
                      <span>{new Date(b.created_at).toLocaleDateString()}</span>
                      {modes.includes('dji') && (
                        <span style={{
                          background: 'rgba(59,130,246,0.25)', color: 'rgb(147,197,253)',
                          padding: '1px 5px', borderRadius: 3, fontSize: 9, fontWeight: 600,
                        }}>DJI</span>
                      )}
                      {modes.includes('inspection') && (
                        <span style={{
                          background: 'rgba(6,182,212,0.25)', color: 'rgb(103,232,249)',
                          padding: '1px 5px', borderRadius: 3, fontSize: 9, fontWeight: 600,
                        }}>NEN</span>
                      )}
                    </span>
                  </button>
                  <button
                    onClick={() => void deleteBuilding(b.id)}
                    title={`Delete "${b.name}"`}
                    style={{
                      background: 'transparent', border: 'none',
                      borderLeft: '1px solid var(--border)',
                      cursor: 'pointer', color: 'var(--text-dim)',
                      padding: '0 12px', fontSize: 16, lineHeight: 1,
                      display: 'flex', alignItems: 'center', justifyContent: 'center',
                    }}
                    onMouseEnter={(e) => { (e.currentTarget as HTMLButtonElement).style.color = '#f87171'; }}
                    onMouseLeave={(e) => { (e.currentTarget as HTMLButtonElement).style.color = 'var(--text-dim)'; }}
                  >
                    ×
                  </button>
                </div>
              );
            })}
          </div>
        </div>
      )}

      {/* ======== PATH MODE SELECTOR (KMZ only) ========
          Mutually-exclusive choice of which flight path is active.
          Each mode is a persisted snapshot on the selected building — click
          switches instantly. If a mode hasn't been generated yet, its
          "Generate" action appears inside the mode-specific panel below. */}
      {isKmz && selectedBuildingId && (() => {
        const selected = buildings.find((b) => b.id === selectedBuildingId);
        const availableModes = selected?.available_modes ?? [];
        const hasDji = availableModes.includes('dji');
        const hasInsp = availableModes.includes('inspection');
        const MODES: Array<{
          key: 'dji' | 'inspection';
          label: string;
          subtitle: string;
          has: boolean;
          tooltip: string;
        }> = [
          {
            key: 'dji', label: 'DJI path', subtitle: hasDji ? 'saved' : 'default',
            has: hasDji,
            tooltip: "Original DJI flight path. Tune mesh + facade detection to drive gimbal rewrites. Waypoints stay as DJI flew them.",
          },
          {
            key: 'inspection', label: 'Custom path', subtitle: hasInsp ? 'saved' : 'not generated',
            has: hasInsp,
            tooltip: "Fresh NEN-2767 inspection mission from detected facades. Per-facade boustrophedon, perpendicular gimbals, stop-and-shoot.",
          },
        ];
        return (
          <div className="section" style={{ padding: 10 }}>
            <h3 style={{ margin: 0, marginBottom: 8, fontSize: 12, letterSpacing: 0.5, textTransform: 'uppercase', color: 'var(--text-dim)' }}>
              Flight path
            </h3>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6 }}>
              {MODES.map((m) => {
                const active = kmzMode === m.key;
                return (
                  <button
                    key={m.key}
                    onClick={() => void switchMode(m.key)}
                    title={m.tooltip}
                    style={{
                      background: active
                        ? (m.key === 'inspection'
                            ? 'linear-gradient(135deg, #0891b2 0%, #06b6d4 100%)'
                            : 'linear-gradient(135deg, #2563eb 0%, #3b82f6 100%)')
                        : 'transparent',
                      color: active ? 'white' : 'inherit',
                      border: `1px solid ${active ? 'transparent' : 'var(--border)'}`,
                      borderRadius: 6, padding: '8px 10px', cursor: 'pointer',
                      display: 'flex', flexDirection: 'column', alignItems: 'flex-start', gap: 2,
                      opacity: m.has || active ? 1 : 0.75,
                    }}
                  >
                    <span style={{ fontWeight: 600, fontSize: 13 }}>{m.label}</span>
                    <span style={{ fontSize: 10, opacity: 0.8, display: 'flex', gap: 4, alignItems: 'center' }}>
                      <span style={{
                        width: 6, height: 6, borderRadius: '50%',
                        background: m.has ? '#10b981' : '#6b7280',
                        display: 'inline-block',
                      }} />
                      {m.subtitle}
                    </span>
                  </button>
                );
              })}
            </div>
            {/* Mode-specific action buttons */}
            {kmzMode === 'dji' && result?.version_id && result.summary.facade_count > 0 && (
              <div style={{ marginTop: 8, display: 'flex', flexDirection: 'column', gap: 6 }}>
                <button
                  className="btn-primary"
                  style={{ width: '100%', fontSize: 12, padding: '6px 10px', fontWeight: 600 }}
                  onClick={() => rewriteGimbals()}
                  title="Rewrite gimbals perpendicular to each facade, cap speed to 3 m/s, strip SmartOblique rosette. Saves a new DJI snapshot."
                >
                  Rewrite gimbals
                </button>
                <button
                  className="btn-secondary"
                  style={{ width: '100%', fontSize: 12, padding: '6px 10px' }}
                  onClick={() => stripRosetteOnly()}
                  title="Keep DJI gimbals EXACTLY. Strip only the 5-pose rosette, cap speed to 3 m/s. Saves a new DJI snapshot."
                >
                  Strip rosette only
                </button>
              </div>
            )}
            {kmzMode === 'inspection' && (
              <div style={{ marginTop: 8 }}>
                <button
                  className="btn-primary"
                  style={{
                    width: '100%', fontSize: 12, padding: '6px 10px', fontWeight: 600,
                    background: 'linear-gradient(135deg, #3b82f6 0%, #06b6d4 100%)',
                  }}
                  onClick={() => generateInspectionMission()}
                  title="Generate a fresh NEN-2767 inspection mission from the detected facades. Persists as the inspection snapshot."
                >
                  {hasInsp ? 'Regenerate NEN-2767 path' : 'Generate NEN-2767 path'}
                </button>
              </div>
            )}
          </div>
        );
      })()}

      {/* ======== Mesh reconstruction (shared by both modes) ======== */}
      {isKmz && selectedBuildingId && (
        <Section title="Mesh reconstruction" defaultOpen>
          <button
            onClick={() => kmzOptimizing ? cancelOptimize() : optimizeKmz()}
            disabled={uploading}
            className="btn-primary"
            style={{ width: '100%', background: kmzOptimizing ? '#dc2626' : undefined }}
            title={`Re-reconstruct from ${kmzOptimizeMax.toFixed(2)}m → ${kmzOptimizeMin.toFixed(2)}m in ${kmzOptimizeSteps} passes (geometric ramp).`}>
            {kmzOptimizing ? 'Cancel optimize' : `Optimize (${kmzOptimizeMax.toFixed(2)} → ${kmzOptimizeMin.toFixed(2)}m, ${kmzOptimizeSteps}×)`}
          </button>
          {(kmzOptimizeMessage || refineRunning) && (
            <div className="field-hint" style={{ marginTop: 4, color: (kmzOptimizing || refineRunning) ? 'var(--accent)' : undefined }}>
              {refineRunning && !kmzOptimizing ? 'Refining…' : kmzOptimizeMessage}
            </div>
          )}
          <ToggleField label="Auto-optimize on import" value={kmzAutoRefine}
            tooltip="Run the optimization chain automatically after each KMZ import."
            onChange={setKmzAutoRefine} />
          <SliderField label="Voxel max" value={kmzOptimizeMax}
            min={0.05} max={0.30} step={0.01} format={(v) => `${v.toFixed(2)} m`}
            tooltip="Coarsest voxel (first pass). Larger = faster but less detail."
            defaultValue={0.16}
            onReset={() => setKmzReconParams({ kmzOptimizeMax: 0.16 })}
            onChange={(v) => setKmzReconParams({ kmzOptimizeMax: v })}
            onCommit={triggerRefine} />
          <SliderField label="Voxel min" value={kmzOptimizeMin}
            min={0.05} max={0.25} step={0.01} format={(v) => `${v.toFixed(2)} m`}
            tooltip="Finest voxel (last pass). Smaller = more detail but noisier + slower. Floor ~0.06m."
            defaultValue={0.08}
            onReset={() => setKmzReconParams({ kmzOptimizeMin: 0.08 })}
            onChange={(v) => setKmzReconParams({ kmzOptimizeMin: v })}
            onCommit={triggerRefine} />
          <SliderField label="Passes" value={kmzOptimizeSteps}
            min={1} max={6} step={1} format={(v) => `${v}`}
            tooltip="Number of passes from max → min (geometric ramp)."
            defaultValue={3}
            onReset={() => setKmzReconParams({ kmzOptimizeSteps: 3 })}
            onChange={(v) => setKmzReconParams({ kmzOptimizeSteps: v })} />
          <SliderField label={`α (probe)${kmzAwAlpha == null ? ' (auto)' : ''}`}
            value={kmzAwAlpha ?? Math.max(0.06, Math.min(0.60, kmzOptimizeMin * 2))}
            min={0.02} max={0.80} step={0.02}
            format={(v) => `${v.toFixed(2)} m`}
            tooltip="CGAL alpha_wrap_3 α (probe size). Default 2× voxel. Smaller hugs small features."
            defaultValue={Math.max(0.06, Math.min(0.60, kmzOptimizeMin * 2))}
            onReset={() => setKmzReconParams({ kmzAwAlpha: null })}
            onChange={(v) => setKmzReconParams({ kmzAwAlpha: v })}
            onCommit={triggerRefine} />
          <SliderField label={`offset (wrap)${kmzAwOffset == null ? ' (auto)' : ''}`}
            value={kmzAwOffset ?? Math.max(0.03, Math.min(0.20, kmzOptimizeMin * 1))}
            min={0.01} max={0.40} step={0.01}
            format={(v) => `${v.toFixed(2)} m`}
            tooltip="CGAL alpha_wrap_3 offset (wrap distance). Default 1× voxel. Smaller = tighter."
            defaultValue={Math.max(0.03, Math.min(0.20, kmzOptimizeMin * 1))}
            onReset={() => setKmzReconParams({ kmzAwOffset: null })}
            onChange={(v) => setKmzReconParams({ kmzAwOffset: v })}
            onCommit={triggerRefine} />
        </Section>
      )}

      {/* ======== Facade detection (shared by both modes) ======== */}
      {isKmz && selectedBuildingId && (
        <Section title="Facade detection" defaultOpen={false}>
          <div className="field-hint" style={{ marginBottom: 6 }}>
            CGAL Shape-Detection (region growing). Tuned for many small facets over few large walls — gimbal targets dormers, sills, balconies. Changes re-run on release.
          </div>
          <SliderField label={`ε (plane tol)${kmzFdEpsilon == null ? ' (auto)' : ''}`}
            value={kmzFdEpsilon ?? FD_DEFAULTS.epsilon}
            min={0.005} max={0.25} step={0.005}
            format={(v) => `${v.toFixed(3)} m`}
            tooltip="Max distance from inlier to fitted plane. Smaller = stricter planarity (more facets). Default 0.05m."
            defaultValue={FD_DEFAULTS.epsilon}
            onReset={() => setKmzFacadeParams({ kmzFdEpsilon: null })}
            onChange={(v) => setKmzFacadeParams({ kmzFdEpsilon: v })}
            onCommit={triggerRefine} />
          <SliderField label={`cluster ε (gap)${kmzFdClusterEpsilon == null ? ' (auto)' : ''}`}
            value={kmzFdClusterEpsilon ?? FD_DEFAULTS.clusterEpsilon}
            min={0.02} max={1.2} step={0.02}
            format={(v) => `${v.toFixed(2)} m`}
            tooltip="Max allowed gap between neighboring inliers. Smaller = split wall at small gaps. Default 0.25m."
            defaultValue={FD_DEFAULTS.clusterEpsilon}
            onReset={() => setKmzFacadeParams({ kmzFdClusterEpsilon: null })}
            onChange={(v) => setKmzFacadeParams({ kmzFdClusterEpsilon: v })}
            onCommit={triggerRefine} />
          <SliderField label={`min points${kmzFdMinPoints == null ? ' (auto)' : ''}`}
            value={kmzFdMinPoints ?? FD_DEFAULTS.minPoints}
            min={5} max={400} step={5}
            format={(v) => `${v} pts`}
            tooltip="Smallest region accepted. Lower = more small facets survive. Default 40."
            defaultValue={FD_DEFAULTS.minPoints}
            onReset={() => setKmzFacadeParams({ kmzFdMinPoints: null })}
            onChange={(v) => setKmzFacadeParams({ kmzFdMinPoints: Math.round(v) })}
            onCommit={triggerRefine} />
          <SliderField label={`normal agreement${kmzFdNormalThreshold == null ? ' (auto)' : ''}`}
            value={kmzFdNormalThreshold ?? FD_DEFAULTS.normalThreshold}
            min={0.5} max={0.999} step={0.005}
            format={(v) => v.toFixed(3)}
            tooltip="Cosine-similarity between seed normal and candidate neighbor. Higher = stricter. Default 0.92."
            defaultValue={FD_DEFAULTS.normalThreshold}
            onReset={() => setKmzFacadeParams({ kmzFdNormalThreshold: null })}
            onChange={(v) => setKmzFacadeParams({ kmzFdNormalThreshold: v })}
            onCommit={triggerRefine} />
          <SliderField label={`min wall area${kmzFdMinWallArea == null ? ' (auto)' : ''}`}
            value={kmzFdMinWallArea ?? FD_DEFAULTS.minWallArea}
            min={0.05} max={10} step={0.05}
            format={(v) => `${v.toFixed(2)} m²`}
            tooltip="Drop walls smaller than this. Lower = keep tiny panels / pilasters. Default 0.5 m²."
            defaultValue={FD_DEFAULTS.minWallArea}
            onReset={() => setKmzFacadeParams({ kmzFdMinWallArea: null })}
            onChange={(v) => setKmzFacadeParams({ kmzFdMinWallArea: v })}
            onCommit={triggerRefine} />
          <SliderField label={`min roof area${kmzFdMinRoofArea == null ? ' (auto)' : ''}`}
            value={kmzFdMinRoofArea ?? FD_DEFAULTS.minRoofArea}
            min={0.05} max={10} step={0.05}
            format={(v) => `${v.toFixed(2)} m²`}
            tooltip="Drop roof facets smaller than this. Lower = keep dormers. Default 0.5 m²."
            defaultValue={FD_DEFAULTS.minRoofArea}
            onReset={() => setKmzFacadeParams({ kmzFdMinRoofArea: null })}
            onChange={(v) => setKmzFacadeParams({ kmzFdMinRoofArea: v })}
            onCommit={triggerRefine} />
          <SliderField label={`min density${kmzFdMinDensity == null ? ' (auto)' : ''}`}
            value={kmzFdMinDensity ?? FD_DEFAULTS.minDensity}
            min={1} max={200} step={1}
            format={(v) => `${v} pts/m²`}
            tooltip="Reject 'ghost planes' whose inlier density falls below this. Higher = stricter. Default 25."
            defaultValue={FD_DEFAULTS.minDensity}
            onReset={() => setKmzFacadeParams({ kmzFdMinDensity: null })}
            onChange={(v) => setKmzFacadeParams({ kmzFdMinDensity: Math.round(v) })}
            onCommit={triggerRefine} />
          <button
            className="btn-secondary"
            style={{ width: '100%', fontSize: 11, marginTop: 6 }}
            onClick={() => setKmzFacadeParams({
              kmzFdEpsilon: null, kmzFdClusterEpsilon: null, kmzFdMinPoints: null,
              kmzFdMinWallArea: null, kmzFdMinRoofArea: null, kmzFdMinDensity: null,
              kmzFdNormalThreshold: null,
            })}>
            Reset facade detection
          </button>
        </Section>
      )}

      {/* ======== INSPECTION + FLIGHT + PATH OPT (mission-generation only) ======== */}
      {inspectionMode && (<>
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
      </>)}

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
              const cov = result.summary.facade_coverage?.find((c) => c.facade_index === f.index);
              return (
                <div key={f.index} style={{ opacity: disabled ? 0.5 : 1 }}>
                  <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 11, cursor: 'pointer' }}>
                    <input type="checkbox" checked={!disabled}
                      onChange={() => toggleFacade(f.index)} />
                    <span style={{ width: 10, height: 10, borderRadius: 2, background: f.color, flexShrink: 0 }} />
                    <span style={{ flex: 1 }}>{f.label}</span>
                    {cov && <span style={{ fontSize: 10, opacity: 0.7 }}>{cov.waypoint_count} wp · {cov.area_m2} m²</span>}
                  </label>
                  {cov && cov.waypoint_count > 0 && <FacadeCoverageBar cov={cov} />}
                </div>
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

// --- Per-facade inspection coverage bar ---

function FacadeCoverageBar({ cov }: { cov: FacadeCoverageEntry }) {
  const perp = cov.mean_perpendicularity;
  const pct = perp === null ? 0 : Math.max(0, Math.min(100, perp * 100));
  // Colour: perpendicularity above 0.95 is great (green), 0.7-0.95 is ok
  // (amber), below 0.7 is poor (red). 0.0 is camera is parallel to wall,
  // negative values mean camera is pointing AWAY from the wall.
  const color = perp === null ? '#555' : perp >= 0.95 ? '#22cc55' : perp >= 0.7 ? '#eab308' : '#dc2626';
  return (
    <div style={{ marginLeft: 22, marginTop: 1, marginBottom: 2, display: 'flex', alignItems: 'center', gap: 4, fontSize: 9, color: 'var(--muted)' }}>
      <div style={{ flex: 1, height: 4, background: 'var(--bg-secondary)', borderRadius: 2, overflow: 'hidden' }}>
        <div style={{ width: `${pct}%`, height: '100%', background: color }} />
      </div>
      <span style={{ minWidth: 92, textAlign: 'right' }}>
        ⟂ {perp === null ? '–' : perp.toFixed(2)} · {cov.mean_pitch_abs_deg === null ? '–' : `${cov.mean_pitch_abs_deg.toFixed(0)}°`} · {cov.mean_distance_m === null ? '–' : `${cov.mean_distance_m.toFixed(1)}m`}
      </span>
    </div>
  );
}

// --- Gimbal diff ---

function GimbalDiff({ before, after, isRewritten, diff }: {
  before?: GimbalStats;
  after?: GimbalStats;
  isRewritten: boolean;
  diff?: GimbalDiffEntry[];
}) {
  const [showAll, setShowAll] = useState(false);
  const { showOriginalGimbals, setShowOriginalGimbals } = useStore();
  const fmt = (v: number | undefined) => v === undefined ? '–' : `${v.toFixed(1)}°`;
  const delta = (b: number | undefined, a: number | undefined) => {
    if (b === undefined || a === undefined) return null;
    const d = a - b;
    if (Math.abs(d) < 0.05) return null;
    return <span style={{ color: d > 0 ? '#22cc55' : '#f8a', fontSize: 9, marginLeft: 3 }}>
      {d > 0 ? '+' : ''}{d.toFixed(1)}°
    </span>;
  };
  const hasBoth = before && after;

  return (
    <div style={{ marginTop: 8, padding: 6, background: 'var(--bg-secondary)', borderRadius: 4, fontSize: 10 }}>
      <div style={{ fontWeight: 600, marginBottom: 4, color: 'var(--muted)', textTransform: 'uppercase', letterSpacing: 0.5 }}>
        Gimbal pitch {isRewritten ? '(rewritten)' : hasBoth ? 'preview' : 'current'}
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: hasBoth ? '60px 1fr 1fr' : '60px 1fr', gap: 2, alignItems: 'center' }}>
        <span style={{ color: 'var(--muted)' }}></span>
        {hasBoth && <span style={{ color: 'var(--muted)', fontSize: 9 }}>Before</span>}
        <span style={{ color: 'var(--muted)', fontSize: 9 }}>{hasBoth ? 'After' : ''}</span>
        <span style={{ color: 'var(--muted)' }}>Mean</span>
        {hasBoth && <span>{fmt(before?.pitch_mean)}</span>}
        <span>{fmt(after?.pitch_mean ?? before?.pitch_mean)}{hasBoth && delta(before?.pitch_mean, after?.pitch_mean)}</span>
        <span style={{ color: 'var(--muted)' }}>Median</span>
        {hasBoth && <span>{fmt(before?.pitch_median)}</span>}
        <span>{fmt(after?.pitch_median ?? before?.pitch_median)}{hasBoth && delta(before?.pitch_median, after?.pitch_median)}</span>
        <span style={{ color: 'var(--muted)' }}>Min</span>
        {hasBoth && <span>{fmt(before?.pitch_min)}</span>}
        <span>{fmt(after?.pitch_min ?? before?.pitch_min)}</span>
        <span style={{ color: 'var(--muted)' }}>Max</span>
        {hasBoth && <span>{fmt(before?.pitch_max)}</span>}
        <span>{fmt(after?.pitch_max ?? before?.pitch_max)}</span>
        <span style={{ color: 'var(--muted)' }}>Yaws</span>
        {hasBoth && <span>{before?.yaw_unique ?? '–'}</span>}
        <span>{after?.yaw_unique ?? before?.yaw_unique ?? '–'}</span>
      </div>
      {isRewritten && diff && diff.length > 0 && (
        <div style={{ marginTop: 6 }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 4 }}>
            <label style={{ fontSize: 10, color: 'var(--muted)', cursor: 'pointer' }}>
              <input
                type="checkbox"
                checked={showOriginalGimbals}
                onChange={(e) => setShowOriginalGimbals(e.target.checked)}
                style={{ marginRight: 4, verticalAlign: 'middle' }}
              />
              Show original gimbal directions (red) in 3D
            </label>
          </div>
          <div style={{ fontWeight: 600, color: 'var(--muted)', fontSize: 9, textTransform: 'uppercase', letterSpacing: 0.5, marginBottom: 2 }}>
            Per-waypoint diff ({diff.length} waypoints)
          </div>
          <div style={{
            maxHeight: showAll ? 240 : 140,
            overflowY: 'auto',
            background: 'var(--bg-primary)',
            borderRadius: 3,
            padding: '4px 6px',
            fontFamily: 'var(--font-mono, monospace)',
            fontSize: 10,
            lineHeight: 1.35,
          }}>
            <div style={{ display: 'grid', gridTemplateColumns: '28px 1fr 1fr', columnGap: 6, color: 'var(--muted)', fontSize: 9, paddingBottom: 2, borderBottom: '1px solid var(--border, #333)' }}>
              <span>#</span>
              <span>pitch (° before → after)</span>
              <span>yaw (° before → after)</span>
            </div>
            {(showAll ? diff : diff.slice(0, 15)).map((d) => {
              const dp = d.pitch_after - d.pitch_before;
              const dy = ((d.yaw_after - d.yaw_before + 540) % 360) - 180;
              const dpColor = Math.abs(dp) < 0.1 ? 'var(--muted)' : (dp > 0 ? '#22cc55' : '#f88');
              const dyColor = Math.abs(dy) < 0.1 ? 'var(--muted)' : (dy > 0 ? '#22cc55' : '#f88');
              return (
                <div key={d.index} style={{ display: 'grid', gridTemplateColumns: '28px 1fr 1fr', columnGap: 6, padding: '1px 0' }}>
                  <span style={{ color: 'var(--muted)' }}>{d.index}</span>
                  <span>
                    {d.pitch_before.toFixed(1)} → {d.pitch_after.toFixed(1)}
                    <span style={{ color: dpColor, marginLeft: 4 }}>
                      ({dp >= 0 ? '+' : ''}{dp.toFixed(1)})
                    </span>
                  </span>
                  <span>
                    {d.yaw_before.toFixed(0)} → {d.yaw_after.toFixed(0)}
                    <span style={{ color: dyColor, marginLeft: 4 }}>
                      ({dy >= 0 ? '+' : ''}{dy.toFixed(0)})
                    </span>
                  </span>
                </div>
              );
            })}
          </div>
          {diff.length > 15 && (
            <button
              onClick={() => setShowAll(!showAll)}
              style={{ marginTop: 4, background: 'none', border: 'none', color: 'var(--accent, #6af)', fontSize: 10, cursor: 'pointer', padding: 0 }}
            >
              {showAll ? 'Show less' : `Show all ${diff.length}…`}
            </button>
          )}
        </div>
      )}
    </div>
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
