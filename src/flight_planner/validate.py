"""Mission validation: checks generated waypoints against hardware constraints.

Returns structured warnings/errors that the API surfaces to the frontend.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from .models import (
    AlgorithmConfig,
    ExclusionZone,
    GIMBAL_PAN_MAX_DEG,
    GIMBAL_PAN_MIN_DEG,
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN,
    MAX_SPEED_MS,
    MAX_WAYPOINTS_PER_MISSION,
    MIN_ALTITUDE_M,
    OBSTACLE_CLEARANCE_M,
    Building,
    MissionConfig,
    Waypoint,
)


class Severity(str, Enum):
    ERROR = "error"      # cannot fly — KMZ should not be generated
    WARNING = "warning"  # can fly but quality is degraded
    INFO = "info"        # informational


@dataclass
class ValidationIssue:
    severity: Severity
    code: str
    message: str
    waypoint_indices: list[int] = field(default_factory=list)
    facade_index: int | None = None


def validate_mission(
    waypoints: list[Waypoint],
    config: MissionConfig,
    building: Building | None = None,
    algo: AlgorithmConfig | None = None,
    exclusion_zones: list[ExclusionZone] | None = None,
    generation_stats: dict | None = None,
) -> list[ValidationIssue]:
    """Validate a generated mission against hardware and quality constraints.

    Returns a list of issues sorted by severity (errors first).
    """
    if algo is None:
        algo = AlgorithmConfig()

    issues: list[ValidationIssue] = []

    if not waypoints:
        issues.append(ValidationIssue(
            severity=Severity.ERROR,
            code="no_waypoints",
            message="Mission has no waypoints",
        ))
        return issues

    inspection_wps = [wp for wp in waypoints if not wp.is_transition]

    # --- Numeric validity (NaN / Infinity would corrupt the KMZ) ---

    _NUMERIC_FIELDS = ("x", "y", "z", "lat", "lon", "alt", "heading_deg", "gimbal_pitch_deg", "speed_ms")
    for wp in waypoints:
        for attr in _NUMERIC_FIELDS:
            val = getattr(wp, attr, None)
            if val is not None and (math.isnan(val) or math.isinf(val)):
                issues.append(ValidationIssue(
                    severity=Severity.ERROR,
                    code="invalid_waypoint_value",
                    message=f"WP{wp.index}: {attr} is {val} (NaN or infinity) — degenerate geometry?",
                    waypoint_indices=[wp.index],
                ))
                break  # one error per waypoint is enough

    # --- Hard constraints (errors) ---

    if len(waypoints) > MAX_WAYPOINTS_PER_MISSION:
        issues.append(ValidationIssue(
            severity=Severity.ERROR,
            code="too_many_waypoints",
            message=f"Mission has {len(waypoints)} waypoints, exceeding limit of {MAX_WAYPOINTS_PER_MISSION}",
        ))

    # Check altitude
    low_alt_wps = [wp for wp in waypoints if wp.z < MIN_ALTITUDE_M]
    if low_alt_wps:
        issues.append(ValidationIssue(
            severity=Severity.ERROR,
            code="altitude_below_min",
            message=f"{len(low_alt_wps)} waypoints below minimum altitude ({MIN_ALTITUDE_M}m)",
            waypoint_indices=[wp.index for wp in low_alt_wps],
        ))

    # Check speed
    fast_wps = [wp for wp in waypoints if wp.speed_ms > MAX_SPEED_MS]
    if fast_wps:
        issues.append(ValidationIssue(
            severity=Severity.ERROR,
            code="speed_exceeds_max",
            message=f"{len(fast_wps)} waypoints exceed max speed ({MAX_SPEED_MS} m/s)",
            waypoint_indices=[wp.index for wp in fast_wps],
        ))

    # --- Soft constraints (warnings) ---

    # Gimbal pitch clamped from ideal
    for wp in inspection_wps:
        ideal_pitch = wp.gimbal_pitch_deg
        if ideal_pitch < GIMBAL_TILT_MIN_DEG or ideal_pitch > GIMBAL_TILT_MAX_DEG:
            issues.append(ValidationIssue(
                severity=Severity.WARNING,
                code="gimbal_pitch_clamped",
                message=f"Gimbal pitch {ideal_pitch:.0f}° was clamped to [{GIMBAL_TILT_MIN_DEG}°, {GIMBAL_TILT_MAX_DEG}°] — photo will not be perpendicular",
                waypoint_indices=[wp.index],
                facade_index=wp.facade_index,
            ))
            break  # don't repeat for every waypoint on same facade

    # Gimbal at safety margin (pitch near nadir limit)
    near_limit_wps = [wp for wp in inspection_wps if wp.gimbal_pitch_deg <= algo.gimbal_near_limit_deg]
    if near_limit_wps:
        issues.append(ValidationIssue(
            severity=Severity.INFO,
            code="gimbal_near_limit",
            message=f"{len(near_limit_wps)} waypoints use gimbal pitch near nadir limit ({near_limit_wps[0].gimbal_pitch_deg:.0f}°) — {config.gimbal_pitch_margin_deg}° safety margin applied",
            waypoint_indices=[wp.index for wp in near_limit_wps[:5]],
        ))

    # Photo interval check
    from .camera import get_camera
    camera_spec = get_camera(config.camera)
    min_interval = camera_spec.min_interval_s
    too_close_wps = []
    for i in range(1, len(inspection_wps)):
        wp_a, wp_b = inspection_wps[i - 1], inspection_wps[i]
        if wp_a.facade_index != wp_b.facade_index:
            continue  # skip cross-facade pairs
        dist = math.sqrt(
            (wp_b.x - wp_a.x) ** 2 + (wp_b.y - wp_a.y) ** 2 + (wp_b.z - wp_a.z) ** 2
        )
        time_between = dist / config.flight_speed_ms if config.flight_speed_ms > 0 else 0
        if time_between < min_interval and dist < config.min_photo_distance_m:
            too_close_wps.append(wp_b.index)
    if too_close_wps:
        issues.append(ValidationIssue(
            severity=Severity.WARNING,
            code="photo_interval_too_short",
            message=f"{len(too_close_wps)} photo waypoints are too close for camera interval ({min_interval}s at {config.flight_speed_ms}m/s)",
            waypoint_indices=too_close_wps[:5],
        ))

    # Flight time estimate (including yaw time at heading changes)
    total_dist = sum(
        math.sqrt(
            (waypoints[i].x - waypoints[i - 1].x) ** 2 +
            (waypoints[i].y - waypoints[i - 1].y) ** 2 +
            (waypoints[i].z - waypoints[i - 1].z) ** 2
        )
        for i in range(1, len(waypoints))
    )
    yaw_time_s = 0.0
    for i in range(1, len(waypoints)):
        heading_diff = abs(waypoints[i].heading_deg - waypoints[i - 1].heading_deg)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        if heading_diff > 1 and config.yaw_rate_deg_per_s > 0:
            yaw_time_s += heading_diff / config.yaw_rate_deg_per_s

    # Add overhead for takeoff/landing sequence and per-waypoint hover
    est_time_s = total_dist / config.flight_speed_ms + len(inspection_wps) * algo.hover_time_per_wp_s + yaw_time_s + algo.takeoff_landing_overhead_s
    est_time_min = est_time_s / 60

    if est_time_min > MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN:
        issues.append(ValidationIssue(
            severity=Severity.WARNING,
            code="exceeds_flight_time",
            message=f"Estimated flight time {est_time_min:.0f}min exceeds battery limit ({MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN}min) — plan battery swaps or split into sorties",
        ))
    elif est_time_min > MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN * algo.battery_warning_threshold:
        issues.append(ValidationIssue(
            severity=Severity.WARNING,
            code="exceeds_flight_time",
            message=f"Estimated flight time {est_time_min:.0f}min exceeds {algo.battery_warning_threshold:.0%} of battery limit — insufficient RTH reserve",
        ))
    elif est_time_min > MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN * algo.battery_info_threshold:
        issues.append(ValidationIssue(
            severity=Severity.INFO,
            code="near_flight_time_limit",
            message=f"Estimated flight time {est_time_min:.0f}min is {est_time_min/MAX_FLIGHT_TIME_WITH_MANIFOLD_MIN*100:.0f}% of battery limit",
        ))

    # Surface clearance check
    if building:
        for wp in inspection_wps:
            if wp.facade_index >= 0 and wp.facade_index < len(building.facades):
                facade = building.facades[wp.facade_index]
                import numpy as np
                wp_pos = np.array([wp.x, wp.y, wp.z])
                to_wp = wp_pos - facade.center
                dist = abs(float(np.dot(to_wp, facade.normal)))
                if dist < OBSTACLE_CLEARANCE_M:
                    issues.append(ValidationIssue(
                        severity=Severity.WARNING,
                        code="too_close_to_surface",
                        message=f"WP{wp.index} is {dist:.1f}m from surface (min clearance: {OBSTACLE_CLEARANCE_M}m)",
                        waypoint_indices=[wp.index],
                        facade_index=wp.facade_index,
                    ))
                    break  # don't repeat

    # Exclusion zone info
    zones = exclusion_zones or []
    stats = generation_stats or {}
    zone_removed = stats.get("waypoints_removed_by_zones", 0)
    if zone_removed > 0:
        no_fly_count = sum(1 for z in zones if z.zone_type == "no_fly")
        no_inspect_count = sum(1 for z in zones if z.zone_type == "no_inspect")
        inclusion_count = sum(1 for z in zones if z.zone_type == "inclusion")
        zone_desc = []
        if no_fly_count:
            zone_desc.append(f"{no_fly_count} no-fly")
        if no_inspect_count:
            zone_desc.append(f"{no_inspect_count} no-inspect")
        if inclusion_count:
            zone_desc.append(f"{inclusion_count} geofence")
        issues.append(ValidationIssue(
            severity=Severity.INFO,
            code="exclusion_zone_filtered",
            message=f"{zone_removed} waypoints removed by {', '.join(zone_desc)} zone(s)",
        ))

    # Mapping-polygon clip (KMZ-imported missions only)
    poly_clipped = stats.get("mapping_polygon_clipped_waypoints", 0)
    if poly_clipped > 0:
        affected = stats.get("mapping_polygon_clipped_facades") or []
        facade_suffix = f" (facades: {', '.join(str(i) for i in affected)})" if affected else ""
        issues.append(ValidationIssue(
            severity=Severity.WARNING,
            code="mapping_polygon_clipped",
            message=(
                f"{poly_clipped} waypoint(s) dropped for falling outside the DJI mapping "
                f"polygon{facade_suffix} — reduce standoff or accept coverage loss near the polygon edge"
            ),
        ))

    # Path collision checks
    path_unresolved = stats.get("path_collisions_unresolved", 0)
    if path_unresolved > 0:
        issues.append(ValidationIssue(
            severity=Severity.WARNING,
            code="path_collision",
            message=f"{path_unresolved} flight path segment(s) clip through the building — increase clearance or adjust waypoints",
        ))
    path_resolved = stats.get("path_collisions_resolved", 0)
    if path_resolved > 0:
        issues.append(ValidationIssue(
            severity=Severity.INFO,
            code="path_collision_resolved",
            message=f"{path_resolved} path collision(s) resolved by inserting detour waypoints",
        ))

    disabled = stats.get("disabled_facades", [])
    if disabled:
        issues.append(ValidationIssue(
            severity=Severity.INFO,
            code="facades_disabled",
            message=f"{len(disabled)} facade(s) disabled by user — no waypoints generated for them",
        ))

    # Sort: errors first, then warnings, then info
    issues.sort(key=lambda i: {"error": 0, "warning": 1, "info": 2}[i.severity])
    return issues
