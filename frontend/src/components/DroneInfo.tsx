import { useEffect, useState } from 'react';
import { getDrone } from '../api/client';
import type { DroneSpec } from '../api/types';

const CAMERA_LABELS: Record<string, string> = {
  wide: 'Wide',
  medium_tele: 'Tele',
  telephoto: 'Zoom',
};

export function DroneInfo() {
  const [drone, setDrone] = useState<DroneSpec | null>(null);
  const [open, setOpen] = useState(false);

  useEffect(() => {
    getDrone().then(setDrone).catch(console.error);
  }, []);

  if (!drone) return null;

  return (
    <div className="section drone-info">
      <h3 className="collapsible" onClick={() => setOpen(!open)}>
        <span className={`collapse-arrow ${open ? 'open' : ''}`}>&#9654;</span>
        Drone: {drone.name}
      </h3>
      {open && (
        <div className="drone-specs">
          {Object.entries(drone.cameras).map(([key, cam]) => (
            <span key={key}>
              <b>{CAMERA_LABELS[key] || key}</b>{' '}
              {cam.focal_length_mm}mm · {Math.round(cam.image_width_px * cam.image_height_px / 1e6)}MP · {cam.fov_deg}° FOV
            </span>
          ))}
          <span>
            Gimbal: {drone.gimbal.tilt_min_deg}° to +{drone.gimbal.tilt_max_deg}° tilt · ±{Math.abs(drone.gimbal.pan_max_deg)}° pan
          </span>
          <span>
            Max speed: {drone.flight.max_speed_ms} m/s · Min alt: {drone.flight.min_altitude_m}m
          </span>
          <span>
            Flight time w/ Manifold 3: ~{drone.flight.max_flight_time_manifold_min} min
          </span>
          <span>
            Max waypoints: {drone.flight.max_waypoints.toLocaleString()}
          </span>
        </div>
      )}
    </div>
  );
}
