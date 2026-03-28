import { useEffect } from 'react';
import { useStore } from './store';
import { Sidebar } from './components/Sidebar';
import { Viewer3D } from './components/Viewer3D';
import { MapView } from './components/MapView';
import './app.css';

export default function App() {
  const { result, activeTab, setActiveTab, generate, refreshBuildings } = useStore();

  useEffect(() => {
    refreshBuildings();
    generate();
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
          </div>
          {s && (
            <div className="topbar-stats">
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
            </div>
          )}
        </div>
        <div className="view-container">
          <div className={`view ${activeTab === '3d' ? 'active' : ''}`}>
            <Viewer3D data={result?.viewer_data.threejs ?? null} cameraFov={result?.summary.camera} />
          </div>
          <div className={`view ${activeTab === 'map' ? 'active' : ''}`}>
            <MapView data={result?.viewer_data.leaflet ?? null} />
          </div>
        </div>
      </main>
    </div>
  );
}
