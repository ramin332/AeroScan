"""Mission validation: checks generated waypoints against hardware constraints.

Returns structured warnings/errors that the API surfaces to the frontend.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from .models import (
    CAMERAS,
    AlgorithmConfig,
    CameraName,
    ExclusionZone,
    Facade,
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


def _aimed_standoffs_m(waypoints: list[Waypoint], facades: list[Facade]) -> list[float]:
    """3D distance from each aimed inspection waypoint to its facade's centroid."""
    out: list[float] = []
    for wp in waypoints:
        if wp.is_transition or wp.facade_index is None or wp.facade_index < 0:
            continue
        if wp.facade_index >= len(facades):
            continue
        c = facades[wp.facade_index].center
        out.append(math.sqrt((wp.x - c[0]) ** 2 + (wp.y - c[1]) ** 2 + (wp.z - c[2]) ** 2))
    return out


def median_gsd_mm_per_px(
    waypoints: list[Waypoint],
    facades: list[Facade],
    camera: CameraName,
) -> float | None:
    """Median GSD the mission will actually shoot at, from each aimed waypoint's
    standoff to its facade. None when no waypoint is aimed at a facade.

    This is the number the 2026-07-10 cards never showed: 18.34 m on the WIDE
    lens is 5.01 mm/px against a 2.0 target, and 33.8 m is 9.23 mm/px."""
    from .camera import compute_gsd, get_camera
    d = _aimed_standoffs_m(waypoints, facades)
    if not d:
        return None
    d.sort()
    return float(compute_gsd(get_camera(camera), d[len(d) // 2]))


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

    # --- Fly-through execution gates ---
    #
    # In fly-through mode (WPML toPointAndPassWithContinuityCurvature, "the
    # aircraft will not stop at the point") each waypoint's action chain and
    # heading turn have only the leg time to complete. Measured 2026-07-10:
    # 0.58 s legs lost 104/398 photos and left the nose 14° behind its command.
    # In stop mode (toPointAndStop*) the aircraft stops, so both always finish.
    if not config.stop_at_waypoint:
        short_dwell: list[int] = []
        unreachable: list[int] = []
        worst_step = 0.0
        for i in range(1, len(waypoints)):
            a, b = waypoints[i - 1], waypoints[i]
            leg_m = math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)
            speed = b.speed_ms if b.speed_ms and b.speed_ms > 0 else config.flight_speed_ms
            leg_s = leg_m / speed if speed > 0 else float("inf")
            if not b.is_transition and b.actions and leg_s < config.min_action_dwell_s:
                short_dwell.append(b.index)
            step = abs(b.heading_deg - a.heading_deg) % 360.0
            if step > 180.0:
                step = 360.0 - step
            if config.yaw_rate_deg_per_s > 0 and step / config.yaw_rate_deg_per_s > leg_s:
                unreachable.append(b.index)
                worst_step = max(worst_step, step)
        if short_dwell:
            issues.append(ValidationIssue(
                severity=Severity.WARNING,
                code="action_dwell_too_short",
                message=(
                    f"{len(short_dwell)} waypoint(s) give the camera less than "
                    f"{config.min_action_dwell_s:.1f}s to rotate + shoot before the next waypoint "
                    f"— photos will be skipped (2026-07-10: 104 of 398 lost at 0.58s). "
                    f"Enable stop-at-waypoint or fly slower."
                ),
                waypoint_indices=short_dwell,
            ))
        if unreachable:
            issues.append(ValidationIssue(
                severity=Severity.WARNING,
                code="heading_step_unreachable",
                message=(
                    f"{len(unreachable)} waypoint(s) demand a heading change the airframe cannot "
                    f"finish within the leg at {config.yaw_rate_deg_per_s:.0f}°/s (worst step "
                    f"{worst_step:.0f}°) — the nose, and with it the camera, arrives late. "
                    f"Enable stop-at-waypoint or fly slower."
                ),
                waypoint_indices=unreachable,
            ))

    # --- GSD vs target ---
    # The planner sets standoff from the target GSD, but KMZ-imported and
    # augmented missions fly DJI's trajectory at whatever standoff it has.
    # Nothing flagged 9.23 mm/px (33.8 m) or 5.01 mm/px (18.34 m) against a
    # 2.0 target on 2026-07-10. Warn at 25% over target and name the lens that
    # would meet it at the same standoff — zero flight-time cost.
    if building is not None and building.facades:
        from .camera import compute_gsd, get_camera
        standoffs = _aimed_standoffs_m(inspection_wps, building.facades)
        if standoffs:
            standoffs.sort()
            median_d = standoffs[len(standoffs) // 2]
            gsd = compute_gsd(get_camera(config.camera), median_d)
            if gsd > config.target_gsd_mm_per_px * 1.25:
                better = next(
                    (name.value for name, spec in CAMERAS.items()
                     if compute_gsd(spec, median_d) <= config.target_gsd_mm_per_px),
                    None,
                )
                hint = (
                    f"; {better} would give {compute_gsd(CAMERAS[CameraName(better)], median_d):.2f} mm/px at the same standoff"
                    if better else "; no M4E lens meets the target at this standoff — fly closer"
                )
                issues.append(ValidationIssue(
                    severity=Severity.WARNING,
                    code="gsd_out_of_spec",
                    message=(
                        f"Median standoff {median_d:.1f}m on {config.camera.value} shoots "
                        f"{gsd:.1f} mm/px against a {config.target_gsd_mm_per_px:.1f} mm/px target{hint}"
                    ),
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

    # Facade coverage health: flag LARGE facades (≥ 2m²) that ended up with
    # no inspection waypoints after the full pipeline. Small facets (sills,
    # parapets, dormers) often legitimately get zero WPs because a single
    # grid step covers more than the facet's extent — warning about those
    # is noisy. The 2m² threshold keeps walls and significant roof panels
    # in scope while ignoring trim.
    if building is not None and building.facades:
        covered = {wp.facade_index for wp in inspection_wps if wp.facade_index is not None and wp.facade_index >= 0}
        large_uncovered = [
            f.index for f in building.facades
            if f.index not in covered and (f.width * f.height) >= 2.0
        ]
        if large_uncovered:
            issues.append(ValidationIssue(
                severity=Severity.WARNING,
                code="facades_uncovered",
                message=(
                    f"{len(large_uncovered)} facade(s) ≥ 2m² have no inspection waypoints — "
                    f"the mission will ship zero photos of these walls"
                ),
            ))

    # Mapping-polygon clip (KMZ-imported missions only): inspection
    # waypoints dropped because their XY fell outside the DJI
    # `mission_area_wgs84` envelope — the pilot's on-controller scope.
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

    # Point-cloud safety filter (KMZ-imported missions only): waypoints
    # dropped because the raw DJI cloud had an obstacle (tree, wire, fence,
    # adjacent building) inside the WP's clearance ball, outside the target
    # viewing cone.
    pc_rejected = stats.get("pointcloud_rejected_waypoints", 0)
    if pc_rejected > 0:
        affected = stats.get("pointcloud_rejected_facades") or []
        facade_suffix = f" (facades: {', '.join(str(i) for i in affected)})" if affected else ""
        issues.append(ValidationIssue(
            severity=Severity.WARNING,
            code="pointcloud_obstacle",
            message=(
                f"{pc_rejected} waypoint(s) dropped because the raw point cloud had an "
                f"obstacle within {OBSTACLE_CLEARANCE_M}m of the camera "
                f"position{facade_suffix} — reduce standoff or accept coverage loss near site obstacles"
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
