import { useRef } from 'react';
import type { PerfStats } from '../api/types';
import { useStore } from '../store';

export function PerfPanel({ perf }: { perf: PerfStats | null | undefined }) {
  const prevRef = useRef<PerfStats | null>(null);
  const minPhotoDist = useStore((s) => s.mission.min_photo_distance_m);

  if (!perf) return null;

  const prev = prevRef.current;
  // Update ref after reading prev — so next render has current as prev
  if (perf !== prev) prevRef.current = perf;

  const g = perf.generation;
  const e = perf.extraction;
  const v = perf.validation_counts;

  const totalFiltered = e
    ? e.filtered_by_area + e.filtered_by_occlusion + e.filtered_by_ground + e.filtered_by_normal
    : 0;

  return (
    <div className="perf-panel">
      <div className="perf-header">Performance</div>

      {/* Timing bar */}
      <div className="perf-section">
        <span className="perf-section-title">Timing</span>
        <Row label="Total" value={fmt_ms(perf.total_ms)} delta={delta_ms(prev?.total_ms, perf.total_ms)} />
        <TimingBar building={perf.building_ms} waypoints={perf.waypoints_ms} validate={perf.validate_ms} total={perf.total_ms} />
      </div>

      {/* Generation pipeline */}
      <div className="perf-section">
        <span className="perf-section-title">Generation</span>
        <Row label="Facades inspected" value={`${g.facades_with_waypoints} / ${g.facades_total}`}
          delta={delta_n(prev?.generation.facades_with_waypoints, g.facades_with_waypoints)} />
        <Row label="Waypoints" value={String(g.waypoints_after_dedup)}
          delta={delta_n(prev?.generation.waypoints_after_dedup, g.waypoints_after_dedup)} />
        {g.waypoints_deduped > 0 && (
          <Row label={`Deduped (<${minPhotoDist}m)`} value={`-${g.waypoints_deduped}`} dim />
        )}
        {g.optimization && g.optimization.waypoints_merged > 0 && (
          <Row label="Cross-facade merged" value={`-${g.optimization.waypoints_merged}`} dim />
        )}
      </div>

      {/* Path optimization */}
      {g.optimization && g.optimization.transit_saved_m > 0 && (
        <div className="perf-section">
          <span className="perf-section-title">Path optimization</span>
          <Row label="Transit distance" value={`${g.optimization.transit_distance_after_m.toFixed(1)}m`}
            delta={`-${g.optimization.transit_saved_m.toFixed(1)}m`} />
          {g.optimization.two_opt_improvements > 0 && (
            <Row label="2-opt improvements" value={String(g.optimization.two_opt_improvements)} dim />
          )}
          {g.optimization.facades_reversed.length > 0 && (
            <Row label="Sweeps reversed" value={String(g.optimization.facades_reversed.length)} dim />
          )}
        </div>
      )}

      {/* Extraction funnel (mesh uploads only) */}
      {e && (
        <div className="perf-section">
          <span className="perf-section-title">Mesh extraction</span>
          <Row label="Input faces" value={String(e.input_faces)} />
          <div className="perf-funnel">
            <FunnelStep label="Regions" value={e.regions_found} total={e.regions_found} />
            <FunnelStep label="After filters" value={e.facades_extracted} total={e.regions_found} />
          </div>
          <Row label="Result" value={`${e.walls} walls + ${e.roofs} roofs`}
            delta={delta_n(prev?.extraction?.facades_extracted, e.facades_extracted)} />
          {totalFiltered > 0 && (
            <div className="perf-filters">
              {e.filtered_by_area > 0 && (
                <span className="perf-tag" title={`min_facade_area: surfaces below area threshold removed`}>
                  area -{e.filtered_by_area}
                </span>
              )}
              {e.filtered_by_occlusion > 0 && (
                <span className="perf-tag" title={`occlusion_hit_fraction: interior walls blocked by other geometry`}>
                  occluded -{e.filtered_by_occlusion}
                </span>
              )}
              {e.filtered_by_ground > 0 && (
                <span className="perf-tag" title={`ground_level_threshold: surfaces near ground filtered`}>
                  ground -{e.filtered_by_ground}
                </span>
              )}
              {e.filtered_by_normal > 0 && (
                <span className="perf-tag" title={`downward_face_threshold / degenerate normals`}>
                  normal -{e.filtered_by_normal}
                </span>
              )}
            </div>
          )}
        </div>
      )}

      {/* Validation */}
      <div className="perf-section">
        <span className="perf-section-title">Validation</span>
        <div className="perf-badges">
          <span className={`perf-badge ${v.errors > 0 ? 'error' : 'ok'}`}>{v.errors} err</span>
          <span className={`perf-badge ${v.warnings > 0 ? 'warn' : 'ok'}`}>{v.warnings} warn</span>
          <span className="perf-badge ok">{v.info} info</span>
        </div>
      </div>
    </div>
  );
}

// --- Helpers ---

function fmt_ms(ms: number): string {
  return ms >= 1000 ? `${(ms / 1000).toFixed(2)}s` : `${ms}ms`;
}

function delta_n(prev: number | undefined, curr: number): string | undefined {
  if (prev === undefined || prev === curr) return undefined;
  const d = curr - prev;
  return d > 0 ? `+${d}` : String(d);
}

function delta_ms(prev: number | undefined, curr: number): string | undefined {
  if (prev === undefined) return undefined;
  const d = curr - prev;
  if (Math.abs(d) < 1) return undefined;
  return d > 0 ? `+${d.toFixed(0)}ms` : `${d.toFixed(0)}ms`;
}

function Row({ label, value, dim, delta }: {
  label: string; value: string; dim?: boolean; delta?: string;
}) {
  return (
    <div className={`perf-row${dim ? ' dim' : ''}`}>
      <span>{label}</span>
      <span>
        {value}
        {delta && <span className={`perf-delta ${delta.startsWith('+') ? 'up' : 'down'}`}> {delta}</span>}
      </span>
    </div>
  );
}

function TimingBar({ building, waypoints, validate, total }: {
  building: number; waypoints: number; validate: number; total: number;
}) {
  if (total <= 0) return null;
  const bPct = (building / total) * 100;
  const wPct = (waypoints / total) * 100;
  const vPct = (validate / total) * 100;
  return (
    <div className="perf-timing-bar" title={`Building ${building.toFixed(0)}ms | Waypoints ${waypoints.toFixed(0)}ms | Validate ${validate.toFixed(0)}ms`}>
      <div className="perf-bar-seg building" style={{ width: `${bPct}%` }} />
      <div className="perf-bar-seg waypoints" style={{ width: `${wPct}%` }} />
      <div className="perf-bar-seg validate" style={{ width: `${vPct}%` }} />
    </div>
  );
}

function FunnelStep({ label, value, total }: { label: string; value: number; total: number }) {
  const pct = total > 0 ? Math.round((value / total) * 100) : 0;
  return (
    <div className="perf-funnel-step">
      <span>{label}</span>
      <span>{value} <span className="perf-pct">({pct}%)</span></span>
    </div>
  );
}
