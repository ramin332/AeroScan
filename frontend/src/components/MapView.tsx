import { useEffect, useState } from 'react';
import { MapContainer, TileLayer, Polygon, CircleMarker, Polyline, Popup, Rectangle, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet-draw';
import type { LeafletData, ExclusionZone } from '../api/types';
import { useStore } from '../store';
import 'leaflet/dist/leaflet.css';
import 'leaflet-draw/dist/leaflet.draw.css';

// WGS84 constants matching backend models.py
const WGS84_LAT_A = 111132.92;
const WGS84_LAT_B = -559.82;
const WGS84_LAT_C = 1.175;
const WGS84_LON_A = 111412.84;
const WGS84_LON_B = -93.5;

function metersPerDeg(refLatRad: number): [number, number] {
  const mLat = WGS84_LAT_A + WGS84_LAT_B * Math.cos(2 * refLatRad) + WGS84_LAT_C * Math.cos(4 * refLatRad);
  const mLon = WGS84_LON_A * Math.cos(refLatRad) + WGS84_LON_B * Math.cos(3 * refLatRad);
  return [mLat, mLon];
}

function wgs84ToEnu(lat: number, lon: number, refLat: number, refLon: number): { x: number; y: number } {
  const refLatRad = refLat * Math.PI / 180;
  const [mPerDegLat, mPerDegLon] = metersPerDeg(refLatRad);
  return {
    x: (lon - refLon) * mPerDegLon,
    y: (lat - refLat) * mPerDegLat,
  };
}

function enuToWgs84(x: number, y: number, refLat: number, refLon: number): { lat: number; lon: number } {
  const refLatRad = refLat * Math.PI / 180;
  const [mPerDegLat, mPerDegLon] = metersPerDeg(refLatRad);
  return {
    lat: refLat + y / mPerDegLat,
    lon: refLon + x / mPerDegLon,
  };
}

function MapController({ data }: { data: LeafletData }) {
  const map = useMap();

  useEffect(() => {
    const container = map.getContainer();
    const observer = new ResizeObserver(() => {
      map.invalidateSize();
    });
    observer.observe(container);

    const timer = setInterval(() => {
      if (container.offsetHeight > 0) {
        map.invalidateSize();
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

const ZONE_COLORS: Record<ExclusionZone['zone_type'], string> = {
  no_fly: '#ff2222',
  no_inspect: '#ff8800',
  inclusion: '#22cc55',
};

const ZONE_LABELS: Record<ExclusionZone['zone_type'], string> = {
  no_fly: 'No-Fly',
  no_inspect: 'No-Inspect',
  inclusion: 'Geofence',
};

/** Leaflet.draw integration — draws rectangles/polygons, stores actual vertices */
function DrawControl({
  refLat, refLon, zoneType,
}: { refLat: number; refLon: number; zoneType: ExclusionZone['zone_type'] }) {
  const map = useMap();
  const addExclusionZone = useStore((s) => s.addExclusionZone);

  // Recreate draw control when zone type changes so draw color updates
  useEffect(() => {
    const featureGroup = new L.FeatureGroup();
    map.addLayer(featureGroup);

    const color = ZONE_COLORS[zoneType];
    const drawControl = new L.Control.Draw({
      position: 'topright',
      draw: {
        rectangle: false,
        polygon: { shapeOptions: { color, weight: 2, fillOpacity: 0.15 }, allowIntersection: false },
        polyline: false,
        circle: false,
        circlemarker: false,
        marker: false,
      },
      edit: { featureGroup, edit: false, remove: false },
    });
    map.addControl(drawControl);

    const handleCreated = (e: L.LeafletEvent) => {
      const evt = e as L.DrawEvents.Created;
      const layer = evt.layer;

      const zoneCount = useStore.getState().exclusionZones.length;

      // Polygon — store actual vertices, not bounding box
      const rings = (layer as L.Polygon).getLatLngs();
      const latlngs = (Array.isArray(rings[0]) ? rings[0] : rings) as L.LatLng[];
      const enuVerts = latlngs.map((ll) => {
        const e = wgs84ToEnu(ll.lat, ll.lng, refLat, refLon);
        return [e.x, e.y] as [number, number];
      });
      const xs = enuVerts.map((v) => v[0]);
      const ys = enuVerts.map((v) => v[1]);
      const cx = (Math.min(...xs) + Math.max(...xs)) / 2;
      const cy = (Math.min(...ys) + Math.max(...ys)) / 2;
      const zone: ExclusionZone = {
        id: `zone-${Date.now()}`,
        label: `${ZONE_LABELS[zoneType]} ${zoneCount + 1}`,
        center_x: cx,
        center_y: cy,
        center_z: 15,
        size_x: Math.max(...xs) - Math.min(...xs),
        size_y: Math.max(...ys) - Math.min(...ys),
        size_z: 30,
        zone_type: zoneType,
        polygon_vertices: enuVerts,
      };

      addExclusionZone(zone);
      featureGroup.removeLayer(layer);
    };

    map.on(L.Draw.Event.CREATED, handleCreated);

    return () => {
      map.off(L.Draw.Event.CREATED, handleCreated);
      map.removeControl(drawControl);
      map.removeLayer(featureGroup);
    };
  }, [map, refLat, refLon, addExclusionZone, zoneType]);

  return null;
}

/** Popup with type switcher and delete for a zone */
function ZonePopup({ zone }: { zone: ExclusionZone }) {
  const removeExclusionZone = useStore((s) => s.removeExclusionZone);
  const updateExclusionZone = useStore((s) => s.updateExclusionZone);
  return (
    <div style={{ minWidth: 140, fontSize: 12 }}>
      <div style={{ fontWeight: 600, marginBottom: 6 }}>{zone.label}</div>
      <div style={{ marginBottom: 6 }}>
        <select
          value={zone.zone_type}
          style={{ width: '100%', fontSize: 12, padding: '2px 4px' }}
          onChange={(e) => updateExclusionZone(zone.id, { zone_type: e.target.value as ExclusionZone['zone_type'] })}
        >
          <option value="inclusion">Geofence (fly within)</option>
          <option value="no_fly">No-Fly</option>
          <option value="no_inspect">No-Inspect</option>
        </select>
      </div>
      <button
        style={{ width: '100%', fontSize: 12, padding: '3px 0', cursor: 'pointer', background: '#dc2626', color: '#fff', border: 'none', borderRadius: 4 }}
        onClick={() => removeExclusionZone(zone.id)}
      >
        Delete zone
      </button>
    </div>
  );
}

/** Renders zones on the map — polygons as <Polygon>, boxes as <Rectangle> */
function ZoneOverlays({ refLat, refLon }: { refLat: number; refLon: number }) {
  const exclusionZones = useStore((s) => s.exclusionZones);

  return (
    <>
      {exclusionZones.map((zone) => {
        const color = ZONE_COLORS[zone.zone_type] ?? '#ff2222';
        const popup = <Popup><ZonePopup zone={zone} /></Popup>;

        if (zone.polygon_vertices && zone.polygon_vertices.length >= 3) {
          const positions = zone.polygon_vertices.map(([x, y]) => {
            const wgs = enuToWgs84(x, y, refLat, refLon);
            return [wgs.lat, wgs.lon] as [number, number];
          });
          return (
            <Polygon
              key={zone.id}
              positions={positions}
              pathOptions={{ color, weight: 2, fillColor: color, fillOpacity: 0.15, dashArray: '4' }}
            >
              {popup}
            </Polygon>
          );
        }

        const halfX = zone.size_x / 2;
        const halfY = zone.size_y / 2;
        const sw = enuToWgs84(zone.center_x - halfX, zone.center_y - halfY, refLat, refLon);
        const ne = enuToWgs84(zone.center_x + halfX, zone.center_y + halfY, refLat, refLon);
        return (
          <Rectangle
            key={zone.id}
            bounds={[[sw.lat, sw.lon], [ne.lat, ne.lon]]}
            pathOptions={{ color, weight: 2, fillColor: color, fillOpacity: 0.15, dashArray: '4' }}
          >
            {popup}
          </Rectangle>
        );
      })}
    </>
  );
}

export function MapView({ data }: { data: LeafletData | null }) {
  const [zoneType, setZoneType] = useState<ExclusionZone['zone_type']>('no_fly');

  if (!data) {
    return <div className="empty-state">Click "Generate Mission" to start</div>;
  }

  const buildingCoords = data.buildingPoly.map(
    (c) => [c[1], c[0]] as [number, number],
  );

  const refLat = data.center[0];
  const refLon = data.center[1];

  const zoneTypes: ExclusionZone['zone_type'][] = ['inclusion', 'no_fly', 'no_inspect'];

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      {/* Zone type selector — top center */}
      <div style={{
        position: 'absolute', top: 8, left: '50%', transform: 'translateX(-50%)', zIndex: 1000,
        display: 'flex', gap: 4, background: 'rgba(15,23,42,0.85)',
        borderRadius: 6, padding: '4px 6px', backdropFilter: 'blur(4px)',
        border: '1px solid rgba(255,255,255,0.1)',
      }}>
        <span style={{ fontSize: 11, color: '#94a3b8', alignSelf: 'center', marginRight: 4 }}>Draw zone:</span>
        {zoneTypes.map((t) => (
          <button
            key={t}
            onClick={() => setZoneType(t)}
            style={{
              fontSize: 11, padding: '2px 8px', borderRadius: 4, border: 'none', cursor: 'pointer',
              background: zoneType === t ? ZONE_COLORS[t] : 'rgba(255,255,255,0.08)',
              color: zoneType === t ? '#fff' : '#94a3b8',
              fontWeight: zoneType === t ? 600 : 400,
            }}
          >
            {ZONE_LABELS[t]}
          </button>
        ))}
      </div>
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
        <DrawControl refLat={refLat} refLon={refLon} zoneType={zoneType} />
        <ZoneOverlays refLat={refLat} refLon={refLon} />

        {buildingCoords.length > 0 && (
          <Polygon
            positions={buildingCoords}
            pathOptions={{ color: '#fff', weight: 2, fillColor: '#334155', fillOpacity: 0.4, dashArray: '4' }}
          >
            <Popup>{data.buildingLabel}<br />{data.buildingDims}</Popup>
          </Polygon>
        )}

        {data.missionAreaPoly && data.missionAreaPoly.length > 2 && (
          <Polygon
            positions={data.missionAreaPoly as [number, number][]}
            pathOptions={{ color: '#22d3ee', weight: 2, fill: false, dashArray: '6 4' }}
          >
            <Popup>DJI mission area</Popup>
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
