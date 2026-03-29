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
  const [comparison, setComparison] = useState<SimulationStatus['result']>(null);
  const [loading, setLoading] = useState(false);

  // Check URL for a specific sim task
  useEffect(() => {
    const params = new URLSearchParams(window.location.search);
    const urlTask = params.get('sim_task');
    if (urlTask) setActiveTaskId(urlTask);
  }, []);

  // Fetch task list
  useEffect(() => {
    const load = async () => {
      try {
        const { tasks: t } = await api.listSimulations();
        setTasks(t);
        // Auto-select first completed if none active
        if (!activeTaskId) {
          const first = t.find(x => x.status === 'complete');
          if (first) setActiveTaskId(first.task_id);
        }
      } catch { /* ignore */ }
    };
    load();
    const interval = setInterval(load, 5000);
    return () => clearInterval(interval);
  }, [activeTaskId]);

  // Load selected task result
  useEffect(() => {
    if (!activeTaskId) return;
    setLoading(true);
    api.getSimulationStatus(activeTaskId).then(status => {
      if (status.status === 'complete' && status.result) {
        setViewerData(status.result.viewer_data);
        setComparison(status.result);
      }
      setLoading(false);
    }).catch(() => setLoading(false));
  }, [activeTaskId]);

  const completedTasks = tasks.filter(t => t.status === 'complete');
  const c = comparison?.comparison;

  if (completedTasks.length === 0 && !loading) {
    return (
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100%', color: 'var(--text-secondary)', fontSize: 13 }}>
        No simulation results yet. Run a simulation from the sidebar.
      </div>
    );
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%' }}>
      {/* Run switcher bar */}
      {completedTasks.length > 0 && (
        <div style={{ display: 'flex', gap: 4, padding: '6px 8px', background: 'var(--bg-secondary)', borderBottom: '1px solid var(--border)', alignItems: 'center', flexWrap: 'wrap' }}>
          <span style={{ fontSize: 10, color: 'var(--text-secondary)', marginRight: 4 }}>Runs:</span>
          {completedTasks.map(t => {
            const isActive = t.task_id === activeTaskId;
            const label = t.task_id.replace('sim_', '');
            const method = (t as any).comparison?.method === 'tsdf_fusion' ? 'TSDF' : 'fallback';
            const voxel = (t as any).comparison?.voxel_size_m;
            return (
              <button key={t.task_id}
                onClick={() => setActiveTaskId(t.task_id)}
                style={{
                  fontSize: 10, padding: '2px 8px', borderRadius: 3, cursor: 'pointer',
                  background: isActive ? 'var(--accent)' : 'transparent',
                  color: isActive ? '#fff' : 'var(--text-secondary)',
                  border: isActive ? 'none' : '1px solid var(--border)',
                }}>
                {label} {method}{voxel ? ` ${voxel * 100}cm` : ''}
              </button>
            );
          })}
        </div>
      )}
      {/* Comparison stats */}
      {c && (
        <div style={{ display: 'flex', gap: 12, padding: '4px 8px', fontSize: 10, color: 'var(--text-secondary)', borderBottom: '1px solid var(--border)' }}>
          <span>Method: <b>{c.method}</b></span>
          <span><b>{c.num_photos}</b> photos</span>
          <span>Facades: <b>{c.original.facade_count}</b> → <b>{c.reconstructed.facade_count}</b></span>
          <span>Waypoints: <b>{c.original.inspection_waypoints}</b> → <b>{c.reconstructed.inspection_waypoints}</b></span>
          <span>Diff: {c.diff.width_m}w / {c.diff.depth_m}d / {c.diff.height_m}h m</span>
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
