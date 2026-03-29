import { useEffect, useState } from 'react';
import { useStore } from './store';
import { Sidebar } from './components/Sidebar';
import { Viewer3D } from './components/Viewer3D';
import { MapView } from './components/MapView';
import { PerfPanel } from './components/PerfPanel';
import * as api from './api/client';
import type { SimulationStatus, ViewerData } from './api/types';
import './app.css';

function SimulationTab() {
  const [tasks, setTasks] = useState<SimulationStatus[]>([]);
  const [activeTaskId, setActiveTaskId] = useState<string | null>(null);
  const [viewerData, setViewerData] = useState<ViewerData | null>(null);
  const [activeResult, setActiveResult] = useState<SimulationStatus['result']>(null);
  const [loading, setLoading] = useState(false);

  const refreshTasks = async () => {
    try {
      const { tasks: t } = await api.listSimulations();
      setTasks(t);
      if (!activeTaskId) {
        const first = t.find(x => x.status === 'complete');
        if (first) setActiveTaskId(first.task_id);
      }
    } catch { /* ignore */ }
  };

  // Check URL for a specific sim task
  useEffect(() => {
    const params = new URLSearchParams(window.location.search);
    const urlTask = params.get('sim_task');
    if (urlTask) setActiveTaskId(urlTask);
  }, []);

  // Fetch task list
  useEffect(() => {
    refreshTasks();
    const interval = setInterval(refreshTasks, 5000);
    return () => clearInterval(interval);
  }, [activeTaskId]);

  // Load selected task result
  useEffect(() => {
    if (!activeTaskId) return;
    setLoading(true);
    api.getSimulationStatus(activeTaskId).then(status => {
      if (status.status === 'complete' && status.result) {
        setViewerData(status.result.viewer_data);
        setActiveResult(status.result);
      }
      setLoading(false);
    }).catch(() => setLoading(false));
  }, [activeTaskId]);

  const handleDelete = async (taskId: string) => {
    await api.deleteSimulation(taskId);
    if (activeTaskId === taskId) {
      setActiveTaskId(null);
      setViewerData(null);
      setActiveResult(null);
    }
    refreshTasks();
  };

  const completedTasks = tasks.filter(t => t.status === 'complete');
  const c = activeResult?.comparison;

  if (completedTasks.length === 0 && !loading) {
    return (
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100%', color: 'var(--text-secondary)', fontSize: 13 }}>
        No simulation results yet. Run a simulation from the sidebar.
      </div>
    );
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%' }}>
      {/* Run switcher bar with delete per run */}
      {completedTasks.length > 0 && (
        <div style={{ display: 'flex', gap: 4, padding: '6px 8px', background: 'var(--bg-secondary)', borderBottom: '1px solid var(--border)', alignItems: 'center', flexWrap: 'wrap' }}>
          <span style={{ fontSize: 10, color: 'var(--text-secondary)', marginRight: 4 }}>Runs:</span>
          {completedTasks.map(t => {
            const isActive = t.task_id === activeTaskId;
            const label = t.task_id.replace('sim_', '');
            const voxel = (t as any).comparison?.voxel_size_m;
            const photos = (t as any).comparison?.num_photos;
            return (
              <div key={t.task_id} style={{ display: 'flex', alignItems: 'center', gap: 0 }}>
                <button
                  onClick={() => setActiveTaskId(t.task_id)}
                  style={{
                    fontSize: 10, padding: '2px 8px', borderRadius: '3px 0 0 3px', cursor: 'pointer',
                    background: isActive ? 'var(--accent)' : 'transparent',
                    color: isActive ? '#fff' : 'var(--text-secondary)',
                    border: isActive ? 'none' : '1px solid var(--border)',
                    borderRight: 'none',
                  }}>
                  {label}{voxel ? ` ${Math.round(voxel * 100)}cm` : ''}{photos ? ` ${photos}ph` : ''}
                </button>
                <button
                  onClick={() => handleDelete(t.task_id)}
                  title="Delete this run"
                  style={{
                    fontSize: 9, padding: '2px 4px', borderRadius: '0 3px 3px 0', cursor: 'pointer',
                    background: isActive ? 'var(--accent)' : 'transparent',
                    color: isActive ? 'rgba(255,255,255,0.6)' : 'var(--text-secondary)',
                    border: isActive ? 'none' : '1px solid var(--border)',
                    opacity: 0.7,
                  }}>
                  &times;
                </button>
              </div>
            );
          })}
        </div>
      )}
      {/* Comparison table */}
      {c && (
        <div style={{ display: 'flex', gap: 0, padding: '6px 8px', fontSize: 10, borderBottom: '1px solid var(--border)', background: 'var(--bg-primary)' }}>
          <table style={{ width: '100%', borderCollapse: 'collapse' }}>
            <thead>
              <tr style={{ color: 'var(--text-secondary)' }}>
                <th style={{ textAlign: 'left', fontWeight: 500, padding: '1px 8px 1px 0' }}></th>
                <th style={{ textAlign: 'right', fontWeight: 500, padding: '1px 8px' }}>Original</th>
                <th style={{ textAlign: 'right', fontWeight: 500, padding: '1px 8px' }}>Reconstructed</th>
                <th style={{ textAlign: 'right', fontWeight: 500, padding: '1px 8px' }}>Diff</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td style={{ padding: '1px 8px 1px 0', color: 'var(--text-secondary)' }}>Facades</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.original.facade_count}</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.reconstructed.facade_count}</td>
                <td style={{ textAlign: 'right', padding: '1px 8px', color: c.diff.facade_count === 0 ? 'var(--text-secondary)' : '#f66' }}>
                  {c.diff.facade_count > 0 ? '+' : ''}{c.diff.facade_count}
                </td>
              </tr>
              <tr>
                <td style={{ padding: '1px 8px 1px 0', color: 'var(--text-secondary)' }}>Waypoints</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.original.inspection_waypoints}</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.reconstructed.inspection_waypoints}</td>
                <td style={{ textAlign: 'right', padding: '1px 8px', color: c.diff.waypoint_diff === 0 ? 'var(--text-secondary)' : '#f66' }}>
                  {c.diff.waypoint_diff > 0 ? '+' : ''}{c.diff.waypoint_diff}
                </td>
              </tr>
              <tr>
                <td style={{ padding: '1px 8px 1px 0', color: 'var(--text-secondary)' }}>Width</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.original.dimensions[0]}m</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.reconstructed.dimensions[0]}m</td>
                <td style={{ textAlign: 'right', padding: '1px 8px', color: Math.abs(c.diff.width_m) < 0.1 ? 'var(--text-secondary)' : '#f66' }}>
                  {c.diff.width_m > 0 ? '+' : ''}{c.diff.width_m}m
                </td>
              </tr>
              <tr>
                <td style={{ padding: '1px 8px 1px 0', color: 'var(--text-secondary)' }}>Depth</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.original.dimensions[1]}m</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.reconstructed.dimensions[1]}m</td>
                <td style={{ textAlign: 'right', padding: '1px 8px', color: Math.abs(c.diff.depth_m) < 0.1 ? 'var(--text-secondary)' : '#f66' }}>
                  {c.diff.depth_m > 0 ? '+' : ''}{c.diff.depth_m}m
                </td>
              </tr>
              <tr>
                <td style={{ padding: '1px 8px 1px 0', color: 'var(--text-secondary)' }}>Height</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.original.dimensions[2]}m</td>
                <td style={{ textAlign: 'right', padding: '1px 8px' }}>{c.reconstructed.dimensions[2]}m</td>
                <td style={{ textAlign: 'right', padding: '1px 8px', color: Math.abs(c.diff.height_m) < 0.1 ? 'var(--text-secondary)' : '#f66' }}>
                  {c.diff.height_m > 0 ? '+' : ''}{c.diff.height_m}m
                </td>
              </tr>
            </tbody>
          </table>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 1, marginLeft: 12, whiteSpace: 'nowrap', color: 'var(--text-secondary)' }}>
            <span>Method: <b style={{ color: 'var(--text-primary)' }}>{c.method}</b></span>
            <span>Photos: <b style={{ color: 'var(--text-primary)' }}>{c.num_photos}</b></span>
            <span>Voxel: <b style={{ color: 'var(--text-primary)' }}>{c.voxel_size_m * 100}cm</b></span>
            <span>Scale: <b style={{ color: 'var(--text-primary)' }}>{Math.round(c.render_scale * 100)}%</b></span>
          </div>
        </div>
      )}
      {/* 3D Viewer */}
      <div style={{ flex: 1, position: 'relative' }}>
        {loading && <div className="view-loading-overlay"><div className="view-loading-spinner" /></div>}
        {viewerData && <Viewer3D data={viewerData.threejs} cameraFov={null} defaultViewMode="flight" />}
      </div>
    </div>
  );
}

export default function App() {
  const { result, activeTab, setActiveTab, loading, generate, refreshBuildings, lightMode } = useStore();

  useEffect(() => {
    refreshBuildings();
    generate();
  }, []);

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', lightMode ? 'light' : 'dark');
  }, [lightMode]);

  // Auto-switch to sim tab if opened via URL
  useEffect(() => {
    if (new URLSearchParams(window.location.search).has('sim_task')) {
      setActiveTab('sim');
    }
  }, []);

  const s = result?.summary;

  return (
    <div className="layout">
      <Sidebar />
      <main className="main">
        <div className="topbar">
          <div className="tabs">
            <button
              className={`tab ${activeTab === '3d' ? 'active' : ''}`}
              onClick={() => setActiveTab('3d')}
            >
              3D Viewer
            </button>
            <button
              className={`tab ${activeTab === 'map' ? 'active' : ''}`}
              onClick={() => setActiveTab('map')}
            >
              2D Satellite
            </button>
            <button
              className={`tab ${activeTab === 'sim' ? 'active' : ''}`}
              onClick={() => setActiveTab('sim')}
            >
              Simulation
            </button>
          </div>
          <div className="topbar-stats">
            {activeTab !== 'sim' && loading && <span className="topbar-loading">Generating...</span>}
            {activeTab !== 'sim' && !loading && s && (
              <>
                <span className="topbar-stat"><b>{s.inspection_waypoints}</b> photos</span>
                <span className="topbar-divider" />
                <span className="topbar-stat"><b>{s.facade_count}</b> facades</span>
                <span className="topbar-divider" />
                <span className="topbar-stat"><b>{s.camera_distance_m}m</b> offset</span>
                <span className="topbar-divider" />
                <span className="topbar-stat"><b>{Math.round(s.total_path_m)}m</b> path</span>
                <span className="topbar-divider" />
                <span className="topbar-stat"><b>{Math.round(s.estimated_flight_time_s / 60 * 10) / 10}min</b> est.</span>
                <span className="topbar-divider" />
                <span className="topbar-stat">{s.photo_footprint_m[0]}×{s.photo_footprint_m[1]}m footprint</span>
              </>
            )}
          </div>
        </div>
        <div className="view-container">
          {activeTab !== 'sim' && loading && <div className="view-loading-overlay"><div className="view-loading-spinner" /></div>}
          <div className={`view ${activeTab === '3d' ? 'active' : ''}`}>
            <Viewer3D data={result?.viewer_data.threejs ?? null} cameraFov={result?.summary.camera} />
          </div>
          <div className={`view ${activeTab === 'map' ? 'active' : ''}`}>
            <MapView data={result?.viewer_data.leaflet ?? null} />
          </div>
          <div className={`view ${activeTab === 'sim' ? 'active' : ''}`}>
            <SimulationTab />
          </div>
          {activeTab !== 'sim' && <PerfPanel perf={result?.perf} />}
        </div>
      </main>
    </div>
  );
}
