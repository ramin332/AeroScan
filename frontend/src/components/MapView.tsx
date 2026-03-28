import { useEffect } from 'react';
import { MapContainer, TileLayer, Polygon, CircleMarker, Polyline, Popup, useMap } from 'react-leaflet';
import type { LeafletData } from '../api/types';
import 'leaflet/dist/leaflet.css';

function MapController({ data }: { data: LeafletData }) {
  const map = useMap();

  // Invalidate size when the container becomes visible (tab switch)
  useEffect(() => {
    const container = map.getContainer();
    const observer = new ResizeObserver(() => {
      map.invalidateSize();
    });
    observer.observe(container);

    // Also handle initial visibility
    const timer = setInterval(() => {
      if (container.offsetHeight > 0) {
        map.invalidateSize();
        // Fit bounds
        const pts: [number, number][] = [];
        Object.values(data.facadeGroups).forEach((wps) => {
          wps.forEach((wp) => pts.push([wp.lat, wp.lon]));
        });
        if (pts.length > 0) {
          map.fitBounds(pts, { padding: [30, 30] });
        }
        clearInterval(timer);
      }
    }, 200);

    return () => {
      observer.disconnect();
      clearInterval(timer);
    };
  }, [map, data]);

  // Fit bounds when data changes
  useEffect(() => {
    const pts: [number, number][] = [];
    Object.values(data.facadeGroups).forEach((wps) => {
      wps.forEach((wp) => pts.push([wp.lat, wp.lon]));
    });
    if (pts.length > 0) {
      map.invalidateSize();
      map.fitBounds(pts, { padding: [30, 30] });
    }
  }, [data, map]);

  return null;
}

export function MapView({ data }: { data: LeafletData | null }) {
  if (!data) {
    return <div className="empty-state">Click "Generate Mission" to start</div>;
  }

  const buildingCoords = data.buildingPoly.map(
    (c) => [c[1], c[0]] as [number, number],
  );

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <MapContainer
        center={data.center}
        zoom={19}
        style={{ width: '100%', height: '100%' }}
        zoomControl
      >
        <TileLayer
          url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
          attribution="Esri"
          maxZoom={21}
        />
        <MapController data={data} />

        {buildingCoords.length > 0 && (
          <Polygon
            positions={buildingCoords}
            pathOptions={{ color: '#fff', weight: 2, fillColor: '#334155', fillOpacity: 0.4, dashArray: '4' }}
          >
            <Popup>{data.buildingLabel}<br />{data.buildingDims}</Popup>
          </Polygon>
        )}

        {Object.entries(data.facadeGroups).map(([fi, wps]) => {
          const meta = data.facadeMeta[fi];
          const color = meta?.color || '#888';
          return wps.map((wp) => (
            <CircleMarker
              key={wp.index}
              center={[wp.lat, wp.lon]}
              radius={4}
              pathOptions={{ color, fillColor: color, fillOpacity: 0.8, weight: 1 }}
            >
              <Popup>
                <b>WP {wp.index}</b><br />
                Facade: {meta?.label}<br />
                Alt: {wp.alt}m<br />
                Heading: {wp.heading}°<br />
                Gimbal: {wp.gimbal_pitch}°<br />
                Component: {wp.component}<br />
                <small>{wp.lat.toFixed(6)}, {wp.lon.toFixed(6)}</small>
              </Popup>
            </CircleMarker>
          ));
        })}

        <Polyline
          positions={data.flightPath.map((c) => [c[1], c[0]] as [number, number])}
          pathOptions={{ color: '#fff', weight: 1, opacity: 0.4, dashArray: '3 6' }}
        />
      </MapContainer>
    </div>
  );
}
