import type { Summary } from '../api/types';

export function Stats({ summary }: { summary: Summary }) {
  const flightMin = Math.round(summary.estimated_flight_time_s / 60 * 10) / 10;

  return (
    <div className="stats-panel">
      <div className="stats">
        <StatBox value={summary.inspection_waypoints} label="Photos" />
        <StatBox value={summary.facade_count} label="Facades" />
        <StatBox value={summary.camera_distance_m} label="Offset (m)" />
        <StatBox value={`${flightMin}m`} label="Est. flight" />
      </div>
      <div className="stats-detail">
        <div className="stat-row">
          <span>Total path</span>
          <span>{summary.total_path_m}m</span>
        </div>
        <div className="stat-row">
          <span>Transit waypoints</span>
          <span>{summary.transition_waypoints}</span>
        </div>
        <div className="stat-row">
          <span>Photo footprint</span>
          <span>{summary.photo_footprint_m[0]}m × {summary.photo_footprint_m[1]}m</span>
        </div>
        {summary.transitions.length > 0 && (
          <div className="stat-section">
            <span className="stat-section-title">Transitions</span>
            {summary.transitions.map((t, i) => (
              <div key={i} className="stat-row sub">
                <span>F{t.from_facade} → F{t.to_facade}</span>
                <span>{t.heading_change_deg}° turn</span>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}

function StatBox({ value, label }: { value: number | string; label: string }) {
  return (
    <div className="stat-box">
      <div className="num">{value}</div>
      <div className="lbl">{label}</div>
    </div>
  );
}
