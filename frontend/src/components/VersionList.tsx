import { useStore } from '../store';

export function VersionList() {
  const { versions, result, loadVersion, deleteVersion } = useStore();

  if (versions.length === 0) {
    return <div className="empty-text">No versions yet</div>;
  }

  return (
    <div className="version-list">
      {versions.map((v) => (
        <div
          key={v.version_id}
          className={`version-item ${result?.version_id === v.version_id ? 'active' : ''}`}
          onClick={() => loadVersion(v.version_id)}
        >
          <span className="ts">{v.timestamp.split('T')[1]?.substring(0, 8)}</span>
          <span className="wp">{v.waypoint_count} wp</span>
          <span
            className="del"
            onClick={(e) => { e.stopPropagation(); deleteVersion(v.version_id); }}
          >
            ×
          </span>
        </div>
      ))}
    </div>
  );
}
