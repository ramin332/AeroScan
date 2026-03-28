import { useEffect } from 'react';
import { useStore } from './store';
import { Sidebar } from './components/Sidebar';
import { Viewer3D } from './components/Viewer3D';
import { MapView } from './components/MapView';
import './app.css';

export default function App() {
  const { result, activeTab, setActiveTab, generate, refreshBuildings } = useStore();

  // Load buildings and auto-generate on first load
  useEffect(() => {
    refreshBuildings();
    generate();
  }, []);

  return (
    <div className="layout">
      <Sidebar />
      <main className="main">
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
        <div className="view-container">
          <div className={`view ${activeTab === '3d' ? 'active' : ''}`}>
            <Viewer3D data={result?.viewer_data.threejs ?? null} />
          </div>
          <div className={`view ${activeTab === 'map' ? 'active' : ''}`}>
            <MapView data={result?.viewer_data.leaflet ?? null} />
          </div>
        </div>
      </main>
    </div>
  );
}
